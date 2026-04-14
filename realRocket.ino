#include <array>
#include <SPI.h>
#include <ESP32Servo.h>
#include "ICM_20948.h"
#include <cmath>
#include <cstring>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <Adafruit_BMP3XX.h>

#define DEBUG_MODE  // comment this out to disable debug mode
//ctr f "TUNE THIS" to find all the variables that need to be tuned still

// --- SPI pins for the ICM-20948 ---
constexpr int spi_sclk_pin = 25;
constexpr int miso_pin = 32;
constexpr int mosi_pin = 33;
constexpr int icm20948_cs_pin = 26;

// --- BMP390 pins ---
constexpr int barometer_sda_pin = 17;
constexpr int barometer_scl_pin = 5;
constexpr int barometer_int_pin = 16;
constexpr uint8_t BMP390_I2C_ADDR_PRIMARY = 0x76;
constexpr uint8_t BMP390_I2C_ADDR_SECONDARY = 0x77;

// --- General outputs ---
constexpr int status_led_pin = -1;
constexpr int servo_x_pin = 2;
constexpr int servo_y_pin = 0;
constexpr int leg_servo_pin = 15;
constexpr int pyro_stage1_pin = 18;
constexpr int pyro_stage2_pin = 19;

// --- Reaction‐wheel ESC on GPIO0 ---
constexpr int reaction_wheel_pin = -2;
constexpr int reaction_wheel_freq = 50;        
constexpr int reaction_wheel_resolution = 16;  
// MoI Consts for reaction wheel
constexpr float moment_i_rocket = 0.002395f;
constexpr float moment_i_wheel = 0.002051f;

// --- Bench / safety behavior ---
constexpr bool AUTO_START_SEQUENCE = true;
constexpr unsigned long AUTO_START_DELAY_MS = 3000;

// --- Flight tuning ---
constexpr float beta = .85f;

constexpr float TVC_CENTER_DEG_X = 65.0f;
constexpr float TVC_CENTER_DEG_Y = 90.0f; // TUNE THIS
constexpr float TVC_MAX_DEFLECTION_DEG = 45.0f; // TUNE THIS
constexpr float TVC_MAX_ANGLE_LIMIT_DEG = 45.0f; // this should be the same
constexpr float TVC_RESET_ANGLE_LIMIT_DEG = 45.0f; // should also be the same
constexpr float TVC_DEADZONE_DEG = 1.0f;
constexpr float TVC_TIME_STEP_TARGET_S = 0.01f;
constexpr float TVC_INTEGRAL_LIMIT = 20.0f;

constexpr float Kp_tvc = 1.5f;
constexpr float Ki_tvc = 0.3f;
constexpr float Kd_tvc = 0.05f;  //start very low and tune up

constexpr float BOOST_END_ACCEL_THRESHOLD_G = 0.35f; // min allowed acceleration for 1st engine to be considered off (g)
#ifdef DEBUG_MODE
  constexpr unsigned long BOOST_END_CONFIRM_MS = 1;
#else
  constexpr unsigned long BOOST_END_CONFIRM_MS = 100; // how long to confirm 1st engine is off (ms)
#endif

constexpr unsigned long MIN_STAGE1_BURN_MS = 3000;

#ifdef DEBUG_MODE
  constexpr float APOGEE_DESCENT_THRESHOLD_MPS = -5.0f;
  constexpr float MIN_VALID_RELATIVE_ALTITUDE_M = -30.0f;
#else
  constexpr float APOGEE_DESCENT_THRESHOLD_MPS = -0.5f;
  constexpr float MIN_VALID_RELATIVE_ALTITUDE_M = 10.0f; // meters above the launch baseline
#endif

constexpr float LANDING_ESTIMATED_VEHICLE_MASS_KG = 0.75f; // TUNE THIS DONT FORGET
constexpr float LANDING_MOTOR_TOTAL_IMPULSE_NS = 49.61f;
constexpr float LANDING_MOTOR_THRUST_DURATION_S = 3.0f;
constexpr float LANDING_MOTOR_AVG_THRUST_N = LANDING_MOTOR_TOTAL_IMPULSE_NS / LANDING_MOTOR_THRUST_DURATION_S;
constexpr float LANDING_IGNITION_LATENCY_S = 0.20f; // TUNE THIS DONT FORGET
constexpr float LANDING_BURN_MARGIN_M = .5f; // error in height it can be at before it starts burning
constexpr unsigned long LANDING_BURN_DURATION_MS = static_cast<unsigned long>(LANDING_MOTOR_THRUST_DURATION_S * 1000.0f);
constexpr float LANDING_SIMULATION_DT_S = 0.01f;
constexpr float LANDING_MIN_DESCENT_SPEED_MPS = 0.5f;

constexpr float LEGS_DEPLOY_ALTITUDE_M = 6.0f;
constexpr float TOUCHDOWN_ALTITUDE_M = 0.8f;

constexpr unsigned long SENSOR_STALE_TIMEOUT_MS = 100;
constexpr int MAX_DMP_PACKETS_PER_LOOP = 8;
constexpr unsigned long PYRO_FIRE_HOLD_MS = 50;

constexpr float SEA_LEVEL_PRESSURE_PA = 101325.0f;
constexpr float DEG_PER_RAD = 57.2957795f;
constexpr float RAD_PER_DEG = 0.0174532925f;
constexpr float GRAVITY_MPS2 = 9.80665f;

Servo servoX;
Servo servoY;
Servo leg_servo;

ICM_20948_SPI imu;

Adafruit_BMP3XX barometer;

enum FlightStage : uint8_t {
  STAGE_PRELAUNCH = 0,
  STAGE_ASCENT_BURN = 1,
  STAGE_COAST = 2,
  STAGE_DESCENT_ARMED = 3,
  STAGE_LANDING_BURN = 4,
  STAGE_TOUCHDOWN = 5,
  STAGE_ABORT = 6
};

struct TelemetryData {
  float acceleration_g[3];
  float gyro_dps[3];
  float euler_deg[3];
  float accel_norm_g;
  float pressure_pa;
  float temperature_c;
  float altitude_m;
  float relative_altitude_m;
  float vertical_velocity_mps;
  int32_t stage;
  uint32_t last_imu_ms;
  uint32_t last_baro_ms;
  uint8_t quat_valid;
  uint8_t accel_valid;
  uint8_t raw_imu_valid;
  uint8_t baro_valid;
  uint8_t startup_check_passed;
} data = {};

struct FlightState {
  FlightStage stage = STAGE_PRELAUNCH;
  bool startupCheckPassed = false;
  bool barometerOnline = false;
  bool imuOnline = false;
  bool burn2Triggered = false;
  bool legsTriggered = false;
  bool tvcInLimpMode = false;
  bool stage1PyroActive = false;
  bool stage2PyroActive = false;
  unsigned long autoStartAtMs = 0;
  unsigned long stage1StartMs = 0;
  unsigned long burnoutDetectedAtMs = 0;
  unsigned long apogeeDetectedAtMs = 0;
  unsigned long stage2IgnitionAtMs = 0;
  unsigned long lowAccelSinceMs = 0;
  unsigned long stage1PyroStartMs = 0;
  unsigned long stage2PyroStartMs = 0;
  float launchPressureReferencePa = 0.0f;
  float burnoutAltitudeM = 0.0f;
  float apogeeAltitudeM = 0.0f;
  float launchAltitudeReferenceM = 0.0f;
  float rawRelativeAltitudeM = 0.0f;
  float filteredRelativeAltitudeM = 0.0f;
  float filteredVerticalVelocityMps = 0.0f;
  float lastRelativeAltitudeRawM = 0.0f;
  unsigned long lastAltitudeTimeMs = 0;
  bool altitudeFilterInitialized = false;
} flight;

struct StartupHealthReport {
  bool quatHealthy = false;
  bool rawHealthy = false;
  bool accelScaleHealthy = false;
  bool gyroQuietHealthy = false;
  bool baroHealthy = false;
  bool gravityTiltHealthy = false;
  int quatSamples = 0;
  int rawSamples = 0;
  int baroSamples = 0;
  int gravityTiltSamples = 0;
  float meanAccelNormG = 0.0f;
  float meanGyroNormDps = 0.0f;
  float baroNoiseStdDevM = 0.0f;
  float meanGravityRollDeg = 0.0f;
  float meanGravityPitchDeg = 0.0f;
} startupHealth;

struct ThrustCurvePoint {
  float timeS;
  float thrustN;
};

// approximate Estes F15 thrust shape digitized from the plot on their website
// its scaled below so the total impulse matches LANDING_MOTOR_TOTAL_IMPULSE_NS
constexpr std::array<ThrustCurvePoint, 15> F15_THRUST_CURVE = {{
  {0.00f, 0.0f},
  {0.05f, 5.0f},
  {0.10f, 11.0f},
  {0.16f, 20.0f},
  {0.22f, 28.0f},
  {0.26f, 30.0f},
  {0.34f, 27.0f},
  {0.45f, 21.0f},
  {0.55f, 19.5f},
  {0.80f, 18.3f},
  {1.20f, 17.4f},
  {1.80f, 16.9f},
  {2.40f, 16.3f},
  {2.95f, 15.7f},
  {3.00f, 0.0f},
}};

// --- PID state ---
float roll_bias_deg = 0.0f;
float pitch_bias_deg = 0.0f;
float current_roll_deg = 0.0f;
float current_pitch_deg = 0.0f;
float tvc_error_integral[2] = { 0.0f, 0.0f };
float tvc_prev_error[2] = { 0.0f, 0.0f };
unsigned long tvc_prev_time_micros = 0;

//ESP-NOW
uint8_t ground_station[6] = { 0x7C, 0x9E, 0xBD, 0x12, 0x34, 0x56 };  //TUNE THIS LATER

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

float wrapAngle180Deg(float angleDeg) {
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }

  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }

  return angleDeg;
}

float angleDifferenceDeg(float currentDeg, float referenceDeg) {
  return wrapAngle180Deg(currentDeg - referenceDeg);
}

float circularMeanDeg(float sumSin, float sumCos) {
  if ((fabsf(sumSin) < 1e-6f) && (fabsf(sumCos) < 1e-6f)) {
    return 0.0f;
  }

  return wrapAngle180Deg(atan2f(sumSin, sumCos) * DEG_PER_RAD);
}

bool computeGravityTiltDeg(float axG, float ayG, float azG, float &rollDeg, float &pitchDeg) {
  const float norm = sqrtf(axG * axG + ayG * ayG + azG * azG);
  if (norm < 0.5f) {
    return false;
  }

  const float ax = axG / norm;
  const float ay = ayG / norm;
  const float az = azG / norm;

  rollDeg = wrapAngle180Deg(atan2f(ay, az) * DEG_PER_RAD);
  pitchDeg = atan2f(-ax, sqrtf(ay * ay + az * az)) * DEG_PER_RAD;
  return true;
}

float sampleF15ThrustN(float timeS) {
  if (timeS <= F15_THRUST_CURVE.front().timeS) {
    return F15_THRUST_CURVE.front().thrustN;
  }

  for (size_t i = 1; i < F15_THRUST_CURVE.size(); ++i) {
    if (timeS <= F15_THRUST_CURVE[i].timeS) {
      const float t0 = F15_THRUST_CURVE[i - 1].timeS;
      const float t1 = F15_THRUST_CURVE[i].timeS;
      const float thrust0 = F15_THRUST_CURVE[i - 1].thrustN;
      const float thrust1 = F15_THRUST_CURVE[i].thrustN;
      const float alpha = (timeS - t0) / (t1 - t0);
      return thrust0 + alpha * (thrust1 - thrust0);
    }
  }

  return 0.0f;
}

float integrateF15CurveImpulseNs() {
  float impulseNs = 0.0f;

  for (size_t i = 1; i < F15_THRUST_CURVE.size(); ++i) {
    const float dtS = F15_THRUST_CURVE[i].timeS - F15_THRUST_CURVE[i - 1].timeS;
    const float meanThrustN = 0.5f * (F15_THRUST_CURVE[i - 1].thrustN + F15_THRUST_CURVE[i].thrustN);
    impulseNs += meanThrustN * dtS;
  }

  return impulseNs;
}

float f15ThrustScale() {
  static const float rawImpulseNs = integrateF15CurveImpulseNs();
  static const float thrustScale = (rawImpulseNs > 0.0f) ? (LANDING_MOTOR_TOTAL_IMPULSE_NS / rawImpulseNs) : 1.0f;
  return thrustScale;
}

float scaledF15CurveImpulseNs() {
  return integrateF15CurveImpulseNs() * f15ThrustScale();
}

float scaledF15ThrustN(float timeS) {
  return sampleF15ThrustN(timeS) * f15ThrustScale();
}

float estimateLandingBurnRequiredAltitudeM(float downwardVelocityMps, float vehicleMassKg) {
  if (downwardVelocityMps <= 0.0f) {
    return 0.0f;
  }


  float velocityDownMps = downwardVelocityMps;
  float displacementDownM = 0.0f;
  float maxDisplacementDownM = 0.0f;

  const auto integrateStep = [&](float stepS, float thrustN) {
    const float downwardAccelMps2 = GRAVITY_MPS2 - (thrustN / vehicleMassKg);
    displacementDownM += velocityDownMps * stepS + 0.5f * downwardAccelMps2 * stepS * stepS;
    velocityDownMps += downwardAccelMps2 * stepS;
    if (displacementDownM > maxDisplacementDownM) {
      maxDisplacementDownM = displacementDownM;
    }
  };

  float latencyLeftS = LANDING_IGNITION_LATENCY_S;
  while (latencyLeftS > 0.0f) {
    const float stepS = fminf(LANDING_SIMULATION_DT_S, latencyLeftS);
    integrateStep(stepS, 0.0f);
    latencyLeftS -= stepS;
  }

  for (size_t i = 1; i < F15_THRUST_CURVE.size(); ++i) {
    float segmentStartS = F15_THRUST_CURVE[i - 1].timeS;
    const float segmentEndS = F15_THRUST_CURVE[i].timeS;

    while (segmentStartS < segmentEndS) {
      const float stepS = fminf(LANDING_SIMULATION_DT_S, segmentEndS - segmentStartS);
      const float sampleTimeS = segmentStartS + (0.5f * stepS);
      integrateStep(stepS, scaledF15ThrustN(sampleTimeS));
      segmentStartS += stepS;
    }
  }

  return maxDisplacementDownM;
}

const char *stageName(FlightStage stage) {
  switch (stage) {
    case STAGE_PRELAUNCH:
      return "PRELAUNCH";
    case STAGE_ASCENT_BURN:
      return "ASCENT_BURN";
    case STAGE_COAST:
      return "COAST";
    case STAGE_DESCENT_ARMED:
      return "DESCENT_ARMED";
    case STAGE_LANDING_BURN:
      return "LANDING_BURN";
    case STAGE_TOUCHDOWN:
      return "TOUCHDOWN";
    case STAGE_ABORT:
      return "ABORT";
  }
  return "ya idk man";
}

void setStage(FlightStage nextStage, const char *reason) {
  if (flight.stage == nextStage) {
    return;
  }

  Serial.print("Stage transition: ");
  Serial.print(stageName(flight.stage));
  Serial.print(" -> ");
  Serial.print(stageName(nextStage));
  Serial.print(" | ");
  Serial.println(reason);

  flight.stage = nextStage;
  data.stage = static_cast<int32_t>(nextStage);
}

void setStatusLed(bool on) {
  if (status_led_pin >= 0) {
    digitalWrite(status_led_pin, on ? HIGH : LOW);
  }
}

void toggleStatusLed() {
  if (status_led_pin >= 0) {
    digitalWrite(status_led_pin, !digitalRead(status_led_pin));
  }
}

void neutralizeActuators() {
  servoX.write(static_cast<int>(TVC_CENTER_DEG_X));
  servoY.write(static_cast<int>(TVC_CENTER_DEG_Y));
}

void commandTvcAtLimit() {
  const float limitedServoX = TVC_CENTER_DEG_X + clampFloat(-current_roll_deg, -TVC_MAX_DEFLECTION_DEG, TVC_MAX_DEFLECTION_DEG);
  const float limitedServoY = TVC_CENTER_DEG_Y + clampFloat(-current_pitch_deg, -TVC_MAX_DEFLECTION_DEG, TVC_MAX_DEFLECTION_DEG);

  servoX.write(static_cast<int>(roundf(limitedServoX)));
  servoY.write(static_cast<int>(roundf(limitedServoY)));
}

void triggerLegs() {
  leg_servo.write(165);
}

void commandStage1Ignition() {
  digitalWrite(pyro_stage1_pin, HIGH);
  flight.stage1PyroActive = true;
  flight.stage1PyroStartMs = millis();
  Serial.println("Stage 1 pyro fired.");
  flight.stage1StartMs = millis();
}

void commandStage2Ignition() {
  digitalWrite(pyro_stage2_pin, HIGH);
  flight.stage2PyroActive = true;
  flight.stage2PyroStartMs = millis();
  Serial.println("Stage 2 pyro fired.");
  flight.stage2IgnitionAtMs = millis();
  flight.burn2Triggered = true;
}

void reactionWheelCycle() {
  // no reactionwheel rn
}

void updatePyroOutputs() {
  const unsigned long now = millis();

  if (flight.stage1PyroActive && (now - flight.stage1PyroStartMs) >= PYRO_FIRE_HOLD_MS) {
    digitalWrite(pyro_stage1_pin, LOW);
    flight.stage1PyroActive = false;
  }

  if (flight.stage2PyroActive && (now - flight.stage2PyroStartMs) >= PYRO_FIRE_HOLD_MS) {
    digitalWrite(pyro_stage2_pin, LOW);
    flight.stage2PyroActive = false;
  }
}

//COMMUNICATION FUNCTIONS:
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  (void)incomingData;
  const uint8_t *mac = info->src_addr;
  if (memcmp(mac, ground_station, 6) == 0 && len >= 1) {
    esp_now_send(ground_station, reinterpret_cast<uint8_t *>(&data), sizeof(data));
  }
}

float pressureToAltitudeMeters(float pressurePa) {
  if (pressurePa <= 0.0f) {
    return 0.0f;
  }
  return 44330.0f * (1.0f - powf(pressurePa / SEA_LEVEL_PRESSURE_PA, 0.19029495f));
}

float pressureToRelativeAltitudeMeters(float pressurePa, float referencePressurePa) {
  if (pressurePa <= 0.0f || referencePressurePa <= 0.0f) {
    return 0.0f;
  }

  return 44330.0f * (1.0f - powf(pressurePa / referencePressurePa, 0.19029495f));
}

void updateRawImuData() {
  imu.getAGMT();

  if (imu.status != ICM_20948_Stat_Ok) {
    data.raw_imu_valid = 0;
    return;
  }

  data.acceleration_g[0] = imu.accX() / 1000.0f;
  data.acceleration_g[1] = imu.accY() / 1000.0f;
  data.acceleration_g[2] = imu.accZ() / 1000.0f;

  data.gyro_dps[0] = imu.gyrX();
  data.gyro_dps[1] = imu.gyrY();
  data.gyro_dps[2] = imu.gyrZ();

  data.temperature_c = imu.temp();
  data.accel_norm_g = sqrtf(data.acceleration_g[0] * data.acceleration_g[0] + data.acceleration_g[1] * data.acceleration_g[1] + data.acceleration_g[2] * data.acceleration_g[2]);

  data.raw_imu_valid = 1;
  data.last_imu_ms = millis();
}

void parseDmpPacket(const icm_20948_DMP_data_t &dmp_data) {
  if ((dmp_data.header & DMP_header_bitmap_Quat6) != 0) {
    const double q1 = static_cast<double>(dmp_data.Quat6.Data.Q1) / 1073741824.0;
    const double q2 = static_cast<double>(dmp_data.Quat6.Data.Q2) / 1073741824.0;
    const double q3 = static_cast<double>(dmp_data.Quat6.Data.Q3) / 1073741824.0;

    double q0Squared = 1.0 - (q1 * q1 + q2 * q2 + q3 * q3);
    if (q0Squared < 0.0) {
      q0Squared = 0.0;
    }
    const double q0 = sqrt(q0Squared);

    const double roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    const double pitchTerm = clampFloat(static_cast<float>(2.0 * (q0 * q2 - q3 * q1)), -1.0f, 1.0f);
    const double pitch = asin(pitchTerm);
    const double yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

    data.euler_deg[0] = static_cast<float>(roll * DEG_PER_RAD);
    data.euler_deg[1] = static_cast<float>(pitch * DEG_PER_RAD);
    data.euler_deg[2] = static_cast<float>(yaw * DEG_PER_RAD);
    data.quat_valid = 1;
    data.last_imu_ms = millis();
  }

  if ((dmp_data.header & DMP_header_bitmap_Accel) != 0) {
    data.acceleration_g[0] = static_cast<float>(dmp_data.Raw_Accel.Data.X) / 16384.0f;
    data.acceleration_g[1] = static_cast<float>(dmp_data.Raw_Accel.Data.Y) / 16384.0f;
    data.acceleration_g[2] = static_cast<float>(dmp_data.Raw_Accel.Data.Z) / 16384.0f;
    data.accel_valid = 1;
    data.last_imu_ms = millis();
  }
}

void readIMU() {
  // loop through all packets to prevent data delay
  for (int packet = 0; packet < MAX_DMP_PACKETS_PER_LOOP; ++packet) {
    icm_20948_DMP_data_t dmp_data;
    ICM_20948_Status_e status = imu.readDMPdataFromFIFO(&dmp_data);

    if ((status == ICM_20948_Stat_Ok) || (status == ICM_20948_Stat_FIFOMoreDataAvail)) {
      parseDmpPacket(dmp_data);
      if (status == ICM_20948_Stat_Ok) {
        break;
      }
      continue;
    }

    if (status == ICM_20948_Stat_FIFONoDataAvail || status == ICM_20948_Stat_FIFOIncompleteData) {
      break;
    }

    Serial.print("IMU FIFO read error: ");
    Serial.println(imu.statusString(status));
    break;
  }

  updateRawImuData();
}

bool beginBarometer() {
  Wire.begin(barometer_sda_pin, barometer_scl_pin);
  pinMode(barometer_int_pin, INPUT);

  bool started = barometer.begin_I2C(BMP390_I2C_ADDR_PRIMARY, &Wire);
  uint8_t activeAddress = BMP390_I2C_ADDR_PRIMARY;

  if (!started) {
    started = barometer.begin_I2C(BMP390_I2C_ADDR_SECONDARY, &Wire);
    activeAddress = BMP390_I2C_ADDR_SECONDARY;
  }

  if (!started) {
    Serial.println("BMP390 begin_I2C failed at both addresss");
    return false;
  }

  barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  barometer.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  barometer.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.print("BMP390 detected at 0x");
  if (activeAddress < 0x10) {
    Serial.print('0');
  }
  Serial.println(activeAddress, HEX);
  return true;
}

bool updateBarometer() {
  if (!barometer.performReading()) {
    data.baro_valid = 0;
    return false;
  }

  data.pressure_pa = barometer.pressure;
  data.temperature_c = barometer.temperature;
  data.altitude_m = pressureToAltitudeMeters(data.pressure_pa);
  flight.rawRelativeAltitudeM = pressureToRelativeAltitudeMeters(data.pressure_pa, flight.launchPressureReferencePa);
  data.relative_altitude_m = flight.rawRelativeAltitudeM;
  data.baro_valid = 1;
  data.last_baro_ms = millis();
  return true;
}

bool calibrateBarometerBaseline() {
  if (!flight.barometerOnline) {
    return false;
  }

  constexpr int baselineSamples = 80;
  constexpr int warmupSamples = 40;
  float pressureSum = 0.0f;
  float altitudeSum = 0.0f;
  int validSamples = 0;

  Serial.println("Calibrating barometer baseline...");

  for (int i = 0; i < warmupSamples; ++i) {
    barometer.performReading();
    delay(20);
  }

  for (int i = 0; i < baselineSamples; ++i) {
    if (barometer.performReading()) {
      const float pressurePa = barometer.pressure;
      pressureSum += pressurePa;
      altitudeSum += pressureToAltitudeMeters(pressurePa);
      ++validSamples;
    }
    delay(25);
  }

  if (validSamples < baselineSamples / 2) {
    Serial.println("Barometer baseline calibration failed.");
    return false;
  }

  flight.launchPressureReferencePa = pressureSum / static_cast<float>(validSamples);
  flight.launchAltitudeReferenceM = altitudeSum / static_cast<float>(validSamples);
  flight.rawRelativeAltitudeM = 0.0f;
  flight.filteredRelativeAltitudeM = 0.0f;
  flight.filteredVerticalVelocityMps = 0.0f;
  flight.lastRelativeAltitudeRawM = 0.0f;
  flight.lastAltitudeTimeMs = millis();
  flight.altitudeFilterInitialized = false;
  data.relative_altitude_m = 0.0f;
  data.vertical_velocity_mps = 0.0f;

  Serial.print("Launch pressure reference set to ");
  Serial.print(flight.launchPressureReferencePa, 2);
  Serial.println(" Pa");
  Serial.print("Sea-level altitude estimate at launch: ");
  Serial.print(flight.launchAltitudeReferenceM, 2);
  Serial.println(" m");
  Serial.println("Relative height will now be measured from the launch baseline.");
  return true;
}

bool initializeImuDmp() {
  Serial.println("initializing imu dmp...");

  while (imu.begin(icm20948_cs_pin, SPI) != ICM_20948_Stat_Ok) {
    Serial.println("imu.begin() failed, retrying ts...");
    delay(500);
  }

  if (imu.initializeDMP() != ICM_20948_Stat_Ok) {
    Serial.println("initializeDMP() failed");
    return false;
  }

  if (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) {
    Serial.println("enableDMPSensor(GAME_ROTATION_VECTOR) failed");
    return false;
  }

  if (imu.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) != ICM_20948_Stat_Ok) {
    Serial.println("enableDMPSensor(ACCELEROMETER) failed");
    return false;
  }

  if (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 1) != ICM_20948_Stat_Ok) {
    Serial.println("setDMPODRrate(Quat6) failed");
    return false;
  }

  if (imu.setDMPODRrate(DMP_ODR_Reg_Accel, 1) != ICM_20948_Stat_Ok) {
    Serial.println("setDMPODRrate(Accel) failed");
    return false;
  }

  if (imu.enableFIFO() != ICM_20948_Stat_Ok) {
    Serial.println("enableFIFO failed");
    return false;
  }

  if (imu.enableDMP() != ICM_20948_Stat_Ok) {
    Serial.println("enableDMP failed");
    return false;
  }

  if (imu.resetDMP() != ICM_20948_Stat_Ok) {
    Serial.println("resetDMP failed");
    return false;
  }

  if (imu.resetFIFO() != ICM_20948_Stat_Ok) {
    Serial.println("resetFIFO failed");
    return false;
  }

  Serial.println("ICM-20948 dmp ready");
  return true;
}

bool runImuStartupCheck() {
  constexpr int sampleCount = 250;
  float sumRollSin = 0.0f;
  float sumRollCos = 0.0f;
  float sumPitchDeg = 0.0f;
  float sumGravityRollSin = 0.0f;
  float sumGravityRollCos = 0.0f;
  float sumGravityPitchDeg = 0.0f;
  float sumAccelNormG = 0.0f;
  float sumGyroNormDps = 0.0f;
  float sumBaroRelAlt = 0.0f;
  float sumBaroRelAltSq = 0.0f;

  int quatSamples = 0;
  int rawSamples = 0;
  int baroSamples = 0;
  int gravityTiltSamples = 0;

  Serial.println("Running stationary IMU/barometer startup check -> keep the rocket still; current orientation becomes zero");

  for (int i = 0; i < sampleCount; ++i) {
    readIMU();

    if (data.quat_valid) {
      sumRollSin += sinf(data.euler_deg[0] * RAD_PER_DEG);
      sumRollCos += cosf(data.euler_deg[0] * RAD_PER_DEG);
      sumPitchDeg += data.euler_deg[1];
      ++quatSamples;
    }

    if (data.raw_imu_valid) {
      const float gyroNorm = sqrtf(data.gyro_dps[0] * data.gyro_dps[0] + data.gyro_dps[1] * data.gyro_dps[1] + data.gyro_dps[2] * data.gyro_dps[2]);
      float gravityRollDeg = 0.0f;
      float gravityPitchDeg = 0.0f;

      sumAccelNormG += data.accel_norm_g;
      sumGyroNormDps += gyroNorm;
      ++rawSamples;

      if (computeGravityTiltDeg(data.acceleration_g[0], data.acceleration_g[1], data.acceleration_g[2], gravityRollDeg, gravityPitchDeg)) {
        sumGravityRollSin += sinf(gravityRollDeg * RAD_PER_DEG);
        sumGravityRollCos += cosf(gravityRollDeg * RAD_PER_DEG);
        sumGravityPitchDeg += gravityPitchDeg;
        ++gravityTiltSamples;
      }
    }

    if (updateBarometer()) {
      sumBaroRelAlt += data.relative_altitude_m;
      sumBaroRelAltSq += data.relative_altitude_m * data.relative_altitude_m;
      ++baroSamples;
    }

    delay(10);
  }

  const float meanRollDeg = (quatSamples > 0) ? circularMeanDeg(sumRollSin, sumRollCos) : 0.0f;
  const float meanPitchDeg = (quatSamples > 0) ? (sumPitchDeg / quatSamples) : 0.0f;
  const float meanGravityRollDeg = (gravityTiltSamples > 0) ? circularMeanDeg(sumGravityRollSin, sumGravityRollCos) : 0.0f;
  const float meanGravityPitchDeg = (gravityTiltSamples > 0) ? (sumGravityPitchDeg / gravityTiltSamples) : 0.0f;
  const float meanAccelNormG = (rawSamples > 0) ? (sumAccelNormG / rawSamples) : 0.0f;
  const float meanGyroNormDps = (rawSamples > 0) ? (sumGyroNormDps / rawSamples) : 0.0f;

  float baroNoiseStdDevM = 0.0f;
  if (baroSamples > 1) {
    const float meanBaro = sumBaroRelAlt / baroSamples;
    const float variance = (sumBaroRelAltSq / baroSamples) - (meanBaro * meanBaro);
    baroNoiseStdDevM = sqrtf(fmaxf(variance, 0.0f));
  }

  startupHealth.quatHealthy = quatSamples > (sampleCount / 3);
  startupHealth.rawHealthy = rawSamples > (sampleCount / 3);
  startupHealth.accelScaleHealthy = (meanAccelNormG > 0.8f) && (meanAccelNormG < 1.2f);
  startupHealth.gyroQuietHealthy = meanGyroNormDps < 10.0f;
  startupHealth.baroHealthy = (baroSamples > (sampleCount / 3)) && (baroNoiseStdDevM < 1.5f);
  startupHealth.gravityTiltHealthy = gravityTiltSamples > (sampleCount / 3);
  startupHealth.quatSamples = quatSamples;
  startupHealth.rawSamples = rawSamples;
  startupHealth.baroSamples = baroSamples;
  startupHealth.gravityTiltSamples = gravityTiltSamples;
  startupHealth.meanAccelNormG = meanAccelNormG;
  startupHealth.meanGyroNormDps = meanGyroNormDps;
  startupHealth.baroNoiseStdDevM = baroNoiseStdDevM;
  startupHealth.meanGravityRollDeg = meanGravityRollDeg;
  startupHealth.meanGravityPitchDeg = meanGravityPitchDeg;

  if (startupHealth.gravityTiltHealthy) {
    roll_bias_deg = meanGravityRollDeg;
    pitch_bias_deg = meanGravityPitchDeg;
  } else {
    roll_bias_deg = meanRollDeg;
    pitch_bias_deg = meanPitchDeg;
  }

  Serial.println("Startup check summary:");
  Serial.printf("  DMP attitude samples: %d\n", quatSamples);
  Serial.printf("  Raw IMU samples: %d\n", rawSamples);
  Serial.printf("  Gravity tilt samples: %d\n", gravityTiltSamples);
  Serial.printf("  Mean accel norm: %.3f g\n", meanAccelNormG);
  Serial.printf("  Mean gyro norm: %.3f dps\n", meanGyroNormDps);
  Serial.printf("  DMP attitude mean: roll %.2f deg | pitch %.2f deg\n", meanRollDeg, meanPitchDeg);
  Serial.printf("  Gravity tilt mean: roll %.2f deg | pitch %.2f deg\n", meanGravityRollDeg, meanGravityPitchDeg);
  Serial.printf("  Startup zero source: %s\n", startupHealth.gravityTiltHealthy ? "gravity vector" : "DMP fallback");
  Serial.printf("  Roll bias: %.2f deg | Pitch bias: %.2f deg\n", roll_bias_deg, pitch_bias_deg);
  if (flight.barometerOnline) {
    Serial.printf("  Barometer samples: %d | Relative altitude sigma: %.3f m\n", baroSamples, baroNoiseStdDevM);
  }

  if (!startupHealth.quatHealthy) {
    Serial.println("  FAIL: DMP quaternion updates are missing or too sparse.");
  }
  if (!startupHealth.rawHealthy) {
    Serial.println("  FAIL: raw accelerometer / gyro updates are missing.");
  }
  if (!startupHealth.accelScaleHealthy) {
    Serial.println("  FAIL: accelerometer magnitude is not close to 1 g while stationary.");
  }
  if (!startupHealth.gyroQuietHealthy) {
    Serial.println("  FAIL: gyro magnitude is too large while stationary.");
  }
  if (!startupHealth.baroHealthy) {
    Serial.println("  FAIL: barometer data is missing or too noisy for relative altitude.");
  }
  if (!startupHealth.gravityTiltHealthy) {
    Serial.println("  FAIL: could not derive a stable startup zero from the gravity vector.");
  }

  return startupHealth.quatHealthy &&
         startupHealth.rawHealthy &&
         startupHealth.accelScaleHealthy &&
         startupHealth.gyroQuietHealthy &&
         startupHealth.baroHealthy &&
         startupHealth.gravityTiltHealthy;
}

void updateAltitudeAndVelocity() {
  if (!updateBarometer()) {
    return;
  }
  const unsigned long nowMs = millis();
  const float currentAltitudeM = flight.rawRelativeAltitudeM;

  if (!flight.altitudeFilterInitialized) {
    flight.altitudeFilterInitialized = true;
    flight.lastAltitudeTimeMs = nowMs;
    flight.lastRelativeAltitudeRawM = currentAltitudeM;
    flight.filteredRelativeAltitudeM = currentAltitudeM;
    flight.filteredVerticalVelocityMps = 0.0f;
    data.relative_altitude_m = currentAltitudeM;
    data.vertical_velocity_mps = 0.0f;
    return;
  }

  const float dt = (nowMs - flight.lastAltitudeTimeMs) / 1000.0f;
  if (dt <= 0.0f) {
    return;
  }

  const float rawVelocity = (currentAltitudeM - flight.lastRelativeAltitudeRawM) / dt;
  flight.filteredRelativeAltitudeM = beta * flight.filteredRelativeAltitudeM + (1 - beta) * currentAltitudeM;
  flight.filteredVerticalVelocityMps = beta * flight.filteredVerticalVelocityMps + (1 - beta) * rawVelocity;

  data.relative_altitude_m = flight.filteredRelativeAltitudeM;
  data.vertical_velocity_mps = flight.filteredVerticalVelocityMps;

  flight.lastRelativeAltitudeRawM = currentAltitudeM;
  flight.lastAltitudeTimeMs = nowMs;
}

bool imuDataFresh() {
  return data.raw_imu_valid &&
         data.quat_valid &&
         (millis() - data.last_imu_ms) <= SENSOR_STALE_TIMEOUT_MS;
}

bool barometerDataFresh() {
  return data.baro_valid &&
         (millis() - data.last_baro_ms) <= SENSOR_STALE_TIMEOUT_MS;
}

bool flightSensorsHealthy() {
  if (!imuDataFresh()) {
    return false;
  }

  if (!barometerDataFresh()) {
    return false;
  }

  if (!flight.startupCheckPassed) {
    return false;
  }

  return true;
}

void printPrelaunchBlockers() {
  static unsigned long lastPrintMs = 0;
  const unsigned long now = millis();
  if ((now - lastPrintMs) < 1000)
  {
    return;
  }
  lastPrintMs = now;

  #ifdef DEBUG_MODE
    Serial.println("PRELAUNCH health summary:");
    Serial.print("  startupCheckPassed=");
    Serial.println(flight.startupCheckPassed ? "true" : "false");

    Serial.print("  imuOnline=");
    Serial.print(flight.imuOnline ? "true" : "false");
    Serial.print(" raw_imu_valid=");
    Serial.print(data.raw_imu_valid);
    Serial.print(" quat_valid=");
    Serial.print(data.quat_valid);
    Serial.print(" imuAgeMs=");
    Serial.println(millis() - data.last_imu_ms);

    Serial.print("  barometerOnline=");
    Serial.print(flight.barometerOnline ? "true" : "false");
    Serial.print(" baro_valid=");
    Serial.print(data.baro_valid);
    Serial.print(" baroAgeMs=");
    Serial.println(millis() - data.last_baro_ms);

    Serial.print("  startup quat/raw/baro/gravity healthy=");
    Serial.print(startupHealth.quatHealthy ? "true" : "false");
    Serial.print("/");
    Serial.print(startupHealth.rawHealthy ? "true" : "false");
    Serial.print("/");
    Serial.print(startupHealth.baroHealthy ? "true" : "false");
    Serial.print("/");
    Serial.println(startupHealth.gravityTiltHealthy ? "true" : "false");

    Serial.print("  startup accel/gyro healthy=");
    Serial.print(startupHealth.accelScaleHealthy ? "true" : "false");
    Serial.print("/");
    Serial.print(startupHealth.gyroQuietHealthy ? "true" : "false");
    Serial.print("/");

    Serial.print("  startup samples quat/raw/baro/gravity=");
    Serial.print(startupHealth.quatSamples);
    Serial.print("/");
    Serial.print(startupHealth.rawSamples);
    Serial.print("/");
    Serial.print(startupHealth.baroSamples);
    Serial.print("/");
    Serial.println(startupHealth.gravityTiltSamples);

    Serial.print("  means accelNormG=");
    Serial.print(startupHealth.meanAccelNormG, 3);
    Serial.print(" gyroNormDps=");
    Serial.print(startupHealth.meanGyroNormDps, 3);
    Serial.print(" baroSigmaM=");
    Serial.print(startupHealth.baroNoiseStdDevM, 3);
    Serial.print(" gravityRollDeg=");
    Serial.print(startupHealth.meanGravityRollDeg, 2);
    Serial.print(" gravityPitchDeg=");
    Serial.println(startupHealth.meanGravityPitchDeg, 2);
  #endif
}

bool stage1BurnoutDetected() {
  const unsigned long now = millis();
  const unsigned long burnElapsed = now - flight.stage1StartMs;

  if (burnElapsed < MIN_STAGE1_BURN_MS) {
    flight.lowAccelSinceMs = 0;
    return false;
  }

  if (data.accel_norm_g < BOOST_END_ACCEL_THRESHOLD_G) {
    if (flight.lowAccelSinceMs == 0) {
      flight.lowAccelSinceMs = now;
    }
    return (now - flight.lowAccelSinceMs) >= BOOST_END_CONFIRM_MS;
  }

  flight.lowAccelSinceMs = 0;
  return false;
}

bool landingBurnShouldStart() {
  if (!barometerDataFresh()) {
    return false;
  }

  const float descentSpeedMps = -data.vertical_velocity_mps;
  if (descentSpeedMps <= LANDING_MIN_DESCENT_SPEED_MPS) {
    return false;
  }

  const float requiredAltitudeM = estimateLandingBurnRequiredAltitudeM(descentSpeedMps, LANDING_ESTIMATED_VEHICLE_MASS_KG);
  const float triggerAltitudeM = requiredAltitudeM + LANDING_BURN_MARGIN_M;

  static unsigned long lastPrintMs = 0;
  const unsigned long nowMs = millis();
  if ((nowMs - lastPrintMs) >= 250) {
    static const float scaledCurveImpulseNs = scaledF15CurveImpulseNs();
    lastPrintMs = nowMs;

    #ifdef DEBUG_MODE
    Serial.print("Landing burn check | altitude=");
    Serial.print(data.relative_altitude_m, 2);
    Serial.print(" m descentSpeed=");
    Serial.print(descentSpeedMps, 2);
    Serial.print(" m/s required=");
    Serial.print(requiredAltitudeM, 2);
    Serial.print(" m");
    Serial.print(" trigger=");
    Serial.print(triggerAltitudeM, 2);
    Serial.print(" m mass=");
    Serial.print(LANDING_ESTIMATED_VEHICLE_MASS_KG, 2);
    Serial.print(" kg avgThrust=");
    Serial.print(LANDING_MOTOR_AVG_THRUST_N, 2);
    Serial.print(" N impulse=");
    Serial.print(scaledCurveImpulseNs, 2);
    Serial.println(" Ns");
    #endif
  }

  return data.relative_altitude_m <= triggerAltitudeM;
}

void tvcCycle()
{
  if (!imuDataFresh()) {
    neutralizeActuators();
    return;
  }

  current_roll_deg = angleDifferenceDeg(data.euler_deg[0], roll_bias_deg);
  current_pitch_deg = angleDifferenceDeg(data.euler_deg[1], pitch_bias_deg);

  if (!flight.tvcInLimpMode && (fabs(current_roll_deg) > TVC_MAX_ANGLE_LIMIT_DEG || fabs(current_pitch_deg) > TVC_MAX_ANGLE_LIMIT_DEG)) {
    Serial.println("TVC entering limp mode: filtered angle limit exceeded.");
    flight.tvcInLimpMode = true;
    tvc_error_integral[0] = 0.0f;
    tvc_error_integral[1] = 0.0f;
    tvc_prev_error[0] = 0.0f;
    tvc_prev_error[1] = 0.0f;
  }

  if (flight.tvcInLimpMode && fabs(current_roll_deg) < TVC_RESET_ANGLE_LIMIT_DEG && fabs(current_pitch_deg) < TVC_RESET_ANGLE_LIMIT_DEG){
    Serial.println("TVC exiting limp mode: angle back inside reset window.");
    flight.tvcInLimpMode = false;
  }

  if (flight.tvcInLimpMode) {
    commandTvcAtLimit();
    return;
  }

  const unsigned long nowMicros = micros();
  float dt = (tvc_prev_time_micros == 0) ? TVC_TIME_STEP_TARGET_S : static_cast<float>(nowMicros - tvc_prev_time_micros) * 1e-6f;
  if (dt <= 0.00001f) {
    dt = TVC_TIME_STEP_TARGET_S;
  }
  tvc_prev_time_micros = nowMicros;

  const float error[2] = {-current_roll_deg, -current_pitch_deg};
  float servoCommand[2] = {TVC_CENTER_DEG_X, TVC_CENTER_DEG_Y};

  for (int axis = 0; axis < 2; ++axis) {
    if (fabs(error[axis]) < TVC_DEADZONE_DEG) {
      servoCommand[axis] = axis == 0 ? TVC_CENTER_DEG_X : TVC_CENTER_DEG_Y;
      tvc_error_integral[axis] = 0.0f;
    }
    else {
      tvc_error_integral[axis] += error[axis] * dt;
      tvc_error_integral[axis] = clampFloat(tvc_error_integral[axis], -TVC_INTEGRAL_LIMIT, TVC_INTEGRAL_LIMIT);

      const float derivative = (error[axis] - tvc_prev_error[axis]) / dt;
      float correction = Kp_tvc * error[axis] +
                         Ki_tvc * tvc_error_integral[axis] +
                         Kd_tvc * derivative;
      correction = clampFloat(correction, -TVC_MAX_DEFLECTION_DEG, TVC_MAX_DEFLECTION_DEG);
      servoCommand[axis] = axis == 0 ? TVC_CENTER_DEG_X + correction: TVC_CENTER_DEG_Y + correction;
    }

    tvc_prev_error[axis] = error[axis];
  }

  servoX.write(static_cast<int>(roundf(servoCommand[0])));
  servoY.write(static_cast<int>(roundf(servoCommand[1])));
}

void setupEspNow() { //TUNE THIS no but seriously update this function idk if its right
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, ground_station, sizeof(ground_station));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
}

void debugPrintFlightStatus(){
  static unsigned long lastStatusPrintMs = 0;
  const unsigned long now = millis();

  if ((now - lastStatusPrintMs) < 250) {
    return;
  }
  lastStatusPrintMs = now;

  Serial.print("Stage=");
  Serial.print(stageName(flight.stage));
  Serial.print(" relativeHeight=");
  Serial.print(data.relative_altitude_m, 2);
  Serial.print("m Vel=");
  Serial.print(data.vertical_velocity_mps, 2);
  Serial.print("m/s Roll=");
  Serial.print(current_roll_deg, 2);
  Serial.print("deg Pitch=");
  Serial.print(current_pitch_deg, 2);
  Serial.print("deg AccNorm=");
  Serial.print(data.accel_norm_g, 2);
  Serial.println("g");
}

void sendFlightStatusPacket() {
  //send TelemetryData struct 
  //TUNE THIS
}

void setup() {
  if (status_led_pin >= 0)
  {
    pinMode(status_led_pin, OUTPUT);
    setStatusLed(false);
  }

  Serial.begin(115200);

  Serial.println();
  Serial.println("-----------Rocket flight controller setup-----------");

  SPI.begin(spi_sclk_pin, miso_pin, mosi_pin);
  setupEspNow();

  pinMode(pyro_stage1_pin, OUTPUT);
  pinMode(pyro_stage2_pin, OUTPUT);
  digitalWrite(pyro_stage1_pin, LOW);
  digitalWrite(pyro_stage2_pin, LOW);

  servoX.setPeriodHertz(50);
  servoY.setPeriodHertz(50);
  servoX.attach(servo_x_pin, 500, 2400);
  servoY.attach(servo_y_pin, 500, 2400);
  neutralizeActuators();

  leg_servo.setPeriodHertz(50);
  leg_servo.attach(leg_servo_pin, 500, 2400);
  leg_servo.write(0);

  flight.imuOnline = initializeImuDmp();
  if (!flight.imuOnline) {
    setStage(STAGE_ABORT, "IMU DMP initialization failed");
    while (true) {
      toggleStatusLed();
      delay(250);
    }
  }

  flight.barometerOnline = beginBarometer();
  if (flight.barometerOnline && !calibrateBarometerBaseline()) {
    flight.barometerOnline = false;
  }

  tvc_prev_time_micros = micros();
  flight.autoStartAtMs = millis() + AUTO_START_DELAY_MS;

  flight.startupCheckPassed = runImuStartupCheck();
  data.startup_check_passed = flight.startupCheckPassed ? 1 : 0;

  if (flight.startupCheckPassed) {
    Serial.println("Startup sensor check passed.");
  }
  else {
      Serial.println("Startup sensor check failed");
      while (true) {
        toggleStatusLed();
        delay(250);
      }
  }

  if (!flight.barometerOnline)
  {
    Serial.println("Barometeris not online");
  }

  setStatusLed(true);
  Serial.println("Setup complete.");
}

void loop() {
  readIMU();
  updateAltitudeAndVelocity();
  reactionWheelCycle();
  updatePyroOutputs();

  // if ((flight.stage != STAGE_PRELAUNCH) && (flight.stage != STAGE_ABORT) && !flightSensorsHealthy()) {
  //   setStage(STAGE_ABORT, "Sensor data became invalid");
  // }

  switch (flight.stage)
  {
    case STAGE_PRELAUNCH:
      neutralizeActuators();

      if (!flightSensorsHealthy()) {
        printPrelaunchBlockers();
        break;
      }

      if (AUTO_START_SEQUENCE && millis() >= flight.autoStartAtMs) {
        commandStage1Ignition();
        setStage(STAGE_ASCENT_BURN, "Auto-start enabled");
      }
      break;

    case STAGE_ASCENT_BURN:
      tvcCycle();

      if (stage1BurnoutDetected()) {
        flight.burnoutDetectedAtMs = millis();
        flight.burnoutAltitudeM = data.relative_altitude_m;
        setStage(STAGE_COAST, "Detected sustained low acceleration after boost");
      }
      break;

    case STAGE_COAST:
      if (barometerDataFresh() && data.relative_altitude_m > MIN_VALID_RELATIVE_ALTITUDE_M && data.vertical_velocity_mps < APOGEE_DESCENT_THRESHOLD_MPS) {
        flight.apogeeDetectedAtMs = millis();
        flight.apogeeAltitudeM = data.relative_altitude_m;
        setStage(STAGE_DESCENT_ARMED, "Vertical velocity crossed into descent");
      }
      break;

    case STAGE_DESCENT_ARMED:
      tvcCycle();
      if (landingBurnShouldStart()) {
        commandStage2Ignition();
        setStage(STAGE_LANDING_BURN, "second engine started ");
      }
      break;

    case STAGE_LANDING_BURN:
      tvcCycle();

      if (!flight.legsTriggered && data.relative_altitude_m <= LEGS_DEPLOY_ALTITUDE_M) {
        triggerLegs();
        flight.legsTriggered = true;
      }

      if ((millis() - flight.stage2IgnitionAtMs) >= LANDING_BURN_DURATION_MS) {
        setStage(STAGE_TOUCHDOWN, "Landing burn nominal duration elapsed");
      }
      break;

    case STAGE_TOUCHDOWN:
      neutralizeActuators();

      if (!flight.legsTriggered && data.relative_altitude_m <= LEGS_DEPLOY_ALTITUDE_M) {
        triggerLegs();
        flight.legsTriggered = true;
      }

      if ( data.relative_altitude_m <= TOUCHDOWN_ALTITUDE_M) {
        Serial.println("Touchdown baby");
      }
      break;

    case STAGE_ABORT:
      neutralizeActuators();
      setStatusLed(((millis() / 200) % 2) != 0);
      break;
  }

  #ifdef DEBUG_MODE
    debugPrintFlightStatus();
    delay(10);
  #endif
}
