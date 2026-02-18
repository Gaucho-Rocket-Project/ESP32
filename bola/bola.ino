#include <SPI.h>
#include <ESP32Servo.h>
#include "ICM_20948.h"
#include <cmath>
#include <array>  // added for lookup table
// #include "BluetoothSerial.h"
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

// --- SPI pins for VSPI (default) ---
constexpr int spi_sclk_pin = 18;
constexpr int miso_pin = 19;
constexpr int mosi_pin = 23;
constexpr int icm20948_cs_pin = 5;  // Chip‐select for ICM-20948

// --- Main Loop Timing ---
static int stage = 0;
static unsigned long burn1End = 0;
static float apogeeAltitude = 0;
static unsigned long apogeeDetectedTime = 0;
static bool burn2Triggered = false;
static float velocity = 0;


constexpr int status_led_pin = 17;

// --- Reaction‐wheel ESC on GPIO0 ---
constexpr int reaction_wheel_pin = 0;
constexpr int reaction_wheel_freq = 50;  // 50 Hz for typical ESC PWM
constexpr int reaction_wheel_resolution  = 16;   // 16-bit PWM resolution

// --- TVC servos on two GPIOs ---
Servo servoX, servoY;
constexpr int servo_x_pin = 33;
constexpr int servo_y_pin = 32;

// Leg Servo
Servo leg_servo;
constexpr int leg_servo_pin = 15;
bool legs_triggered = false;

int trigger_time;

// --- PID constants for reaction wheel (yaw rate) ---
constexpr float  Kp_rw = 3.3125f, Ki_rw = 0.2f, Kd_rw = 1.3f;
float prevError_rw = 0.0f, integral_rw = 0.0f;
unsigned long rw_prev_time_micros  = 0;


// --- PID constants for TVC (roll/pitch) ---
constexpr float Kp_tvc = 1.5f;
constexpr float Ki_tvc = 0.1f;
constexpr float Kd_tvc = 0.05f;  // START VERY LOW (e.g., 0.0) AND TUNE UP
constexpr float TVC_TIME_STEP_TARGET = 0.01f;
constexpr float tvc_deadzone = 1.0f;
constexpr float LPF_BETA = 0.2f;


// Variables for the LPF-based TVC PID
float current_roll_lpf = 0.0f;
float current_pitch_lpf = 0.0f;
float tvc_error_integral[2] = { 0.0f, 0.0f };
float tvc_prev_error[2] = { 0.0f, 0.0f };
unsigned long tvc_prev_time_micros = 0;


// TVC Limp Mode (Shutdown)
constexpr float TVC_MAX_ANGLE_LIMIT = 45.0f;    // Max filtered angle before TVC enters limp mode
constexpr float TVC_RESET_ANGLE_LIMIT = 35.0f;  // Angle below which TVC can exit limp mode
bool tvc_in_limp_mode = false;


// --- IMU object ---
ICM_20948_SPI imu;

// --- IMU data --- [x, y, z]
struct imu_data{
  int16_t acceleration[3];
  float euler_angles[3];
  uint8_t pressure[3];
  uint8_t temperature[3];
  int isValid;
} data;

// --- Bias offsets ---
float roll_bias = 0, pitch_bias = 0;

// Bluetooth
// BluetoothSerial SerialBT;
// char bt_Cmd;
// bool launch_sequence = false;  // main logic flag

//ESP-NOW
uint8_t ground_station[6] = {0x7C, 0x9E, 0xBD, 0x12, 0x34, 0x56}; //dummy address


// MoI Consts
constexpr float moment_i_rocket = 0.002395f;
constexpr float moment_i_wheel = 0.002051f;


// Motor Constants
unsigned long last_motor_time = 0,
             motor_interval = 1000;  // ms between motor triggers


//COMMUNICATION FUNCTIONS:
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len){
  const uint8_t *mac = info->src_addr;
  if(memcmp(mac, ground_station, 6) == 0 && len >= 1){
    esp_now_send(ground_station, (uint8_t *)&data, sizeof(data));
  } 
}

void readIMU() {
  icm_20948_DMP_data_t dmp_data;
  if(imu.readDMPdataFromFIFO(&dmp_data) == ICM_20948_Stat_Ok){
    if(dmp_data.header == DMP_header_bitmap_Accel){
      data.acceleration[0] = dmp_data.Raw_Accel.Data.X;
      data.acceleration[1] = dmp_data.Raw_Accel.Data.Y;
      data.acceleration[2] = dmp_data.Raw_Accel.Data.Z;
    }
    else if(dmp_data.header == DMP_header_bitmap_Quat6){
      double q1 = static_cast<double>(dmp_data.Quat6.Data.Q1) / 1073741824.0;  // X-axis rotation component
      double q2 = static_cast<double>(dmp_data.Quat6.Data.Q2) / 1073741824.0;  // Y-axis rotation component
      double q3 = static_cast<double>(dmp_data.Quat6.Data.Q3) / 1073741824.0;  // Z-axis rotation component
      double q_sum_sq = q1 * q1 + q2 * q2 + q3 * q3;
      double q0 = (q_sum_sq < 1.0) ? sqrt(1.0 - q_sum_sq) : 0.0;

      //https://ntrs.nasa.gov/api/citations/19770024290/downloads/19770024290.pdf
      //Explanation on using quaternions to represent 3D Rotations

      float roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));

      float pitch_raw = 2 * (q0*q2 - q3*q1);
      float pitch;
      if(pitch_raw <= -1){
        pitch = -M_PI_2;
      } else if(pitch_raw >= 1){
        pitch = M_PI_2;
      } else {
        pitch = asin(pitch_raw);
      }

      float yaw = atan2(2 * (q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));

      data.euler_angles[0] = roll;
      data.euler_angles[1] = pitch;
      data.euler_angles[2] = yaw;   
     }
    else if(dmp_data.header == DMP_header_bitmap_Pressure){
      memcpy(data.pressure, dmp_data.Pressure, 3);
      memcpy(data.temperature, dmp_data.Pressure+3, 3);
    }
    data.isValid = 1;
  }
}


//ACTION FUNCTIONS:

void triggerLegs(){
    // ledcWrite(leg_pin, usToDuty(1500));
}
// void triggerMotor(){
//   // e.g. ledcWrite(reaction_wheel_pin, usToDuty(2000));
//  Serial.println("Trigger motor and legs");
// }
void tvcCycle(){
    // 1) TVC control using DMP Game Rotation Vector

  if (data.isValid) {
    
    //temporarily removed lpf functionality but may need to add back in?
    current_roll_lpf = data.euler_angles[0];
    current_pitch_lpf = data.euler_angles[1];
    // current_yaw_lpf = data.euler_angles[2];
    
    // current_roll_lpf = lpf(current_roll_lpf, current_roll_raw, LPF_BETA) - roll_bias;
    // current_pitch_lpf = lpf(current_pitch_lpf, current_pitch_raw, LPF_BETA) - pitch_bias;

    // ---------- TVC Limp Mode Logic ------------------------------
    if (!tvc_in_limp_mode && (fabs(current_roll_lpf) > TVC_MAX_ANGLE_LIMIT || fabs(current_pitch_lpf) > TVC_MAX_ANGLE_LIMIT)) {
      Serial.println("!!! TVC Entering LIMP MODE: Angle limit exceeded !!!");
      tvc_in_limp_mode = true;
      servoX.write(90);  // Go to neutral
      servoY.write(90);
      tvc_error_integral[0] = 0.0f;  // Reset PID state
      tvc_error_integral[1] = 0.0f;
      tvc_prev_error[0] = 0.0f;  // Reset previous error for D term
      tvc_prev_error[1] = 0.0f;
    }


    if (tvc_in_limp_mode && fabs(current_roll_lpf) < TVC_RESET_ANGLE_LIMIT && fabs(current_pitch_lpf) < TVC_RESET_ANGLE_LIMIT) {
      Serial.println("TVC Exiting LIMP MODE: Angles back in range.");
      tvc_in_limp_mode = false;
      // PID state (integrals, prev_errors) will naturally rebuild on next active PID cycle
    }


   // --- TVC PID Control ---
   if (!tvc_in_limp_mode) {
     unsigned long current_tvc_micros = micros();
     float dt_tvc = (tvc_prev_time_micros == 0) ? TVC_TIME_STEP_TARGET : static_cast<float>(current_tvc_micros - tvc_prev_time_micros) * 1e-6f;
     if (dt_tvc <= 0.00001f) { dt_tvc = TVC_TIME_STEP_TARGET; }
     tvc_prev_time_micros = current_tvc_micros;


     float tvc_error[2];
     tvc_error[0] = 0.0f - current_roll_lpf;
     tvc_error[1] = 0.0f - current_pitch_lpf;


     float servo_command_angle_calculated[2] = { 90.0f, 90.0f };  // Temporary for calculation


     for (int axis = 0; axis < 2; ++axis) {
       if (fabs(tvc_error[axis]) < tvc_deadzone) {
         servo_command_angle_calculated[axis] = 90.0f;
         tvc_error_integral[axis] = 0.0f;
       } else {
         tvc_error_integral[axis] += tvc_error[axis] * dt_tvc;
         // Optional: Clamp tvc_error_integral[axis]
         float derivative = (dt_tvc > 0.00001f) ? (tvc_error[axis] - tvc_prev_error[axis]) / dt_tvc : 0.0f;
         float pid_correction = Kp_tvc * tvc_error[axis] + Ki_tvc * tvc_error_integral[axis] + Kd_tvc * derivative;
         float max_deflection = 30.0f;
         pid_correction = constrain(pid_correction, -max_deflection, max_deflection);
         servo_command_angle_calculated[axis] = 90.0f + round(pid_correction);
       }
       tvc_prev_error[axis] = tvc_error[axis];
     }
     servoX.write(static_cast<int>(servo_command_angle_calculated[0]));
     servoY.write(static_cast<int>(servo_command_angle_calculated[1]));


     Serial.print("ACTIVE Roll: ");
     Serial.print(current_roll_lpf, 1);
     Serial.print(", Pitch: ");
     Serial.print(current_pitch_lpf, 1);
     Serial.print(" | ServoX: ");
     Serial.print(servo_command_angle_calculated[0], 1);
     Serial.print(", ServoY: ");
     Serial.println(servo_command_angle_calculated[1], 1);


   } else {  // TVC is in LIMP MODE
     // Servos should already be at 90 from when limp mode was entered.
     // This block ensures they stay there if no other logic writes to them.
     servoX.write(90);
     servoY.write(90);
     Serial.print("LIMP MODE Roll: ");
     Serial.print(current_roll_lpf, 1);
     Serial.print(", Pitch: ");
     Serial.print(current_pitch_lpf, 1);
     Serial.println(" | Servos at Neutral.");
   }
 }  // End of DMP data processing

}
// void reactionWheelCycle(){
//     // Consider if reaction wheel should also be affected by tvc_in_limp_mode
//   if (!tvc_in_limp_mode && imu.dataReady()) {  // Only run RW PID if TVC is not in limp mode
//     imu.getAGMT();
//     float yawRate = imu.gyrZ();
//     float targetYawRate = 0.0f;
//     unsigned long current_rw_micros = micros();
//     float dt_rw = (prevTime_rw_micros == 0) ? TVC_TIME_STEP_TARGET : static_cast<float>(current_rw_micros - prevTime_rw_micros) * 1e-6f;
//     if (dt_rw <= 0.00001f) { dt_rw = TVC_TIME_STEP_TARGET; }

//     current_yaw_lpf = lpf(current_yaw_lpf, yawRate, LPF_BETA);
//     rw_derivative = (current_yaw_lpf - past_yaw_lfp) / dt_rw;
//     rw_error_integral += (current_yaw_lpf + past_yaw_lfp) * dt_rw;
//     past_yaw_lfp = current_yaw_lpf;

//     // — 2) Reaction-wheel PID function —
//     float u = Kp_rw*current_yaw_lpf + Ki_rw*rw_error_integral + Kd_rw*rw_derivative;  // negative of the current yaw direction 
//     int pulse = constrain(1500 - int(u), 1000, 2000);
//     ledcWrite(reaction_wheel_pin, usToDuty(pulse));


//   } else if (tvc_in_limp_mode) {
//     // If TVC is in limp mode, set reaction wheel to neutral for safety
//     ledcWrite(reaction_wheel_pin, usToDuty(1500));
//     // Serial.println("Reaction Wheel Neutral due to TVC Limp Mode.");
//  }
// }


void setup() {
  pinMode(status_led_pin, OUTPUT); // Set GPIO 17 as an output pin
  data.isValid = 0;

  Serial.begin(115200);
  while (!Serial) Serial.println("Setup starting...");
  SPI.begin(spi_sclk_pin, miso_pin, mosi_pin);
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }


  // Initiate Comms
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, ground_station, sizeof(ground_station));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Initializing IMU DMP...");
  while (imu.begin(icm20948_cs_pin, SPI) != ICM_20948_Stat_Ok) {
    Serial.println("IMU.begin failed; retrying...");
    delay(500);
  }
  if (imu.initializeDMP() != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: initializeDMP failed!");
    while (1);
  }
  if (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: enableDMPSensor failed!");
    while (1);
  }
  if (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 1) != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: setDMPODRrate failed!");
    while (1);
  }
  if (imu.enableFIFO() != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: enableFIFO failed!");
    while (1);
  }
  if (imu.enableDMP() != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: enableDMP failed!");
    while (1);
  }
  if (imu.resetDMP() != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: resetDMP failed!");
    while (1);
  }
  if (imu.resetFIFO() != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: resetFIFO failed!");
    while (1);
  }
  Serial.println("ICM-20948 DMP ready.");


  // Calibrate bias axes (vertical stance)
  Serial.println("Calibrating biases... hold sensor vertical");
  constexpr int num_bias_sample = 200;
  float sum_r = 0, sum_y = 0;
  for(int i=0; i< num_bias_sample ; i++){
    readIMU();
    float current_roll_raw = data.euler_angles[0];
    float current_pitch_raw = data.euler_angles[1];

    current_roll_lpf = current_roll_raw;
    current_pitch_lpf = current_pitch_raw;

    //need to look into lpf
    // current_roll_lpf = lpf(current_roll_lpf, current_roll_raw, lpf_beta);
    // current_pitch_lpf = lpf(current_pitch_lpf, current_pitch_raw, lpf_beta);
    delay(10);
  }
  roll_bias = sum_r/num_bias_sample;
  pitch_bias  = sum_y/num_bias_sample;
  Serial.printf("Biases: roll=%.1f°, yaw=%.1f°\n", roll_bias, pitch_bias);

  servoX.setPeriodHertz(50);
  servoY.setPeriodHertz(50);
  servoX.attach(servo_x_pin, 500, 2400);
  servoY.attach(servo_y_pin, 500, 2400);
  servoX.write(90);
  servoY.write(90);

  leg_servo.setPeriodHertz(50);
  leg_servo.attach(leg_servo_pin, 500, 2400);

  ledcAttach(reaction_wheel_pin, reaction_wheel_freq, reaction_wheel_resolution );
  ledcAttach(leg_servo_pin, 50, 16);
  Serial.println("Arming Reaction Wheel ESC: Sending 1500us. Please wait ~5 seconds...");
  // ledcWrite(reaction_wheel_pin, usToDuty(1500));
  delay(5000);
  Serial.println("ESC presumed armed.");


  tvc_prev_time_micros = micros();
  rw_prev_time_micros  = micros();

  trigger_time = 3000 + millis();

  Serial.println("Setup complete.");
}

float prevAltitude = 0;
float velocity = 0;
unsigned long lastAltitudeTime = 0;

float getAltitude() {
    // return converted pressure data OR test constant for dummy; placeholder below:
    return (float)data.pressure[0]; // FIX later
}


void loop() {

    // Read IMU always
    readIMU();

    //Reaction Wheel always
    reactionWheelCycle();

    // ------- UPDATE ALTITUDE + VELOCITY ------
    unsigned long tNow = millis();
    static unsigned long lastAltitudeTime = millis();
    float dt = (tNow - lastAltitudeTime) / 1000.0f;
    float altitude = getAltitude();
    static float prevAltitude = altitude;
    static float velocity = 0;
    unsigned long burn2Delay = 2000;

    if(dt > 0.02f) {
        velocity = (altitude - prevAltitude) / dt;
        prevAltitude = altitude;
        lastAltitudeTime = tNow;
    }

    // -------- STATE MACHINE --------

    switch(stage) {

        case 0: // Pre-launch
            // Wait for a command or automatically start Burn 1
            Serial.println("Stage 0: Awaiting launch...");
            //add wait command
            //start engine 1
            stage = 1; 
        break;


        case 1: // BURN 1 ACTIVE (~3.5 sec)
            tvcCycle();    
            Serial.println("Stage 1: Burn 1 running...");

            // find when vertical acceleration is -1g
            if (data.acceleration[2] <= 10) {// MAKE SURE TO CHECK ORIENTATION also later on need to test value for gravity
              burn1End = millis();
              Serial.println("Burn 1 complete");
              stage = 2;
            }
        break;


        case 2: // COAST PHASE - Detect Apogee (velocity crosses zero downward)
            Serial.println("Stage 2: Coasting, detecting apogee...");

            if(velocity < 0) {  
                apogeeAltitude = altitude;
                apogeeDetectedTime = millis();
                Serial.printf("Apogee found at %.2fm\n", apogeeAltitude);
                stage = 3;
            }
        break;


        case 3: // COMPUTE WAIT TIME FOR SECOND BURN
            Serial.println("Stage 3: Calculating timing for burn 2...");

            // *** Placeholder physics: replace with algorithm ***
            // unsigned long burn2Delay = 2000;  // Example: fire 2 seconds after apogee
            // do not declare and initialize variable inside switch blocks

            if(millis() >= apogeeDetectedTime + (apogeeDetectedTime - burn1End) - burn2Delay) {
                Serial.println("Firing second burn...");
                burn2Triggered = true;
                stage = 4;
            }
        break;


        case 4: // SECOND BURN ACTIVE (TVC ON)
            tvcCycle();
            Serial.println("Stage 4: Burn 2 active...");

            // Optional: burn until certain velocity/altitude but for now fixed:
            if(altitude <= 20) { // Arbitrary condition or timer
                Serial.println("Burn 2 complete, entering landing mode.");
                stage = 5;
            }
        break;


        case 5: // LANDING MODE
            Serial.println("Stage 5: Landing mode...");

            if(altitude <= 3 && !legs_triggered) {
                Serial.println("Deploying legs...");
                triggerLegs();
                legs_triggered = true;
            }
        break;
    }

    delay(10);
}
