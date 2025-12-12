#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0x68

// MPU registers
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C

// Scales (MPU6050 default)
static constexpr float GYRO_SCALE = 131.0f;    // LSB/(deg/s)
static constexpr float ACCEL_SCALE = 16384.0f; // LSB/g

// Complementary filter constants
static constexpr float ANGLE_ALPHA = 0.98f;     // roll/pitch complementary
static constexpr float YAW_ALPHA   = 0.98f;     // gyro trust for yaw when using mag fusion

// Orientation struct returned by getOrientation()
struct Orientation {
  float roll;   // degrees
  float pitch;  // degrees
  float yaw;    // degrees
};

// Internal state
static float attitudeRoll  = 0.0f; // degrees (base-swapped setup kept)
static float attitudePitch = 0.0f;
static float attitudeYaw   = 0.0f;

static unsigned long lastMs = 0;
static bool mpuInitialized = false;

// Gyro bias (LSB)
static float gyroBiasX = 0.0f;
static float gyroBiasY = 0.0f;
static float gyroBiasZ = 0.0f;
static bool gyroCalibrated = false;

// Magnetometer fusion variables (optional)
static bool magAvailable = false;
static float magHeadingDeg = 0.0f; // last provided magnetometer heading (degrees)
static bool magUpdated = false;     // set true when setMagHeading called (one-shot until next update)

// --- low-level I2C helpers ---
static void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static void readRegisters(uint8_t reg, uint8_t count, uint8_t *buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, (int)count);
  for (uint8_t i = 0; i < count && Wire.available(); ++i) buf[i] = Wire.read();
}

static inline int16_t toInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

// --- public API ---

// Call at setup()
inline void mpuBegin(bool doAutoCalibrate = true, uint16_t calSamples = 200) {
  Wire.begin();
  delay(50);
  // Wake up
  writeRegister(PWR_MGMT_1, 0x00);
  delay(50);
  // sample rate
  writeRegister(SMPLRT_DIV, 0x07);     // sample rate 125Hz
  writeRegister(CONFIG, 0x03);         // DLPF
  writeRegister(GYRO_CONFIG, 0x00);    // ±250 dps
  writeRegister(ACCEL_CONFIG, 0x00);   // ±2g

  lastMs = millis();
  mpuInitialized = true;

  if (doAutoCalibrate) {
    // basic blocking calibration on begin
    // calibrate gyro bias in LSB units
    // small delay to let sensor settle
    delay(100);
    // call our calibrator (inline below)
    // we use provided sample count
    uint32_t sumX = 0, sumY = 0, sumZ = 0;
    for (uint16_t i = 0; i < calSamples; ++i) {
      uint8_t b[14];
      readRegisters(ACCEL_XOUT_H, 14, b);
      int16_t gx = toInt16(b[8], b[9]);
      int16_t gy = toInt16(b[10], b[11]);
      int16_t gz = toInt16(b[12], b[13]);
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      delay(2); // small pause
    }
    gyroBiasX = (float)sumX / (float)calSamples;
    gyroBiasY = (float)sumY / (float)calSamples;
    gyroBiasZ = (float)sumZ / (float)calSamples;
    gyroCalibrated = true;
  }
}

// Manual gyro calibration function: call from sketch when level & steady
inline void mpuCalibrateGyro(uint16_t samples = 500, uint16_t sampleDelayMs = 2) {
  if (!mpuInitialized) return;
  int64_t sx = 0, sy = 0, sz = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    uint8_t b[14];
    readRegisters(ACCEL_XOUT_H, 14, b);
    int16_t gx = toInt16(b[8], b[9]);
    int16_t gy = toInt16(b[10], b[11]);
    int16_t gz = toInt16(b[12], b[13]);
    sx += gx; sy += gy; sz += gz;
    delay(sampleDelayMs);
  }
  gyroBiasX = (float)sx / (float)samples;
  gyroBiasY = (float)sy / (float)samples;
  gyroBiasZ = (float)sz / (float)samples;
  gyroCalibrated = true;
}

// Provide magnetometer heading (degrees). If you have a HMC5883/AK8963 compute heading before calling.
inline void setMagHeading(float headingDeg) {
  magHeadingDeg = headingDeg;
  magAvailable = true;
  magUpdated = true;
}

// Tell module whether you plan to use mag fusion (keeps explicit)
inline void hasMag(bool available) {
  magAvailable = available;
}

// Reset yaw (zero heading)
inline void resetYaw() {
  attitudeYaw = 0.0f;
}

// Angle normalization helpers
static inline float wrap180(float ang) {
  while (ang <= -180.0f) ang += 360.0f;
  while (ang > 180.0f)   ang -= 360.0f;
  return ang;
}

// Compute shortest signed difference: target - from, in [-180,180]
static inline float shortestAngleDiff(float target, float from) {
  float d = wrap180(target - from);
  return d;
}

// --- core update (reads sensors, updates attitudeRoll/Pitch/Yaw) ---
static void mpuUpdate() {
  if (!mpuInitialized) return;

  unsigned long now = millis();
  float dt = (now - lastMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  if (dt > 0.1f) dt = 0.1f; // clamp
  lastMs = now;

  uint8_t buf[14];
  readRegisters(ACCEL_XOUT_H, 14, buf);

  int16_t ax = toInt16(buf[0], buf[1]);
  int16_t ay = toInt16(buf[2], buf[3]);
  int16_t az = toInt16(buf[4], buf[5]);

  int16_t gx_raw = toInt16(buf[8], buf[9]);
  int16_t gy_raw = toInt16(buf[10], buf[11]);
  int16_t gz_raw = toInt16(buf[12], buf[13]);

  // subtract bias (if calibrated)
  float gx = (float)gx_raw - gyroBiasX;
  float gy = (float)gy_raw - gyroBiasY;
  float gz = (float)gz_raw - gyroBiasZ;

  // convert to physical units (deg/s)
  float gx_dps = gx / GYRO_SCALE;
  float gy_dps = gy / GYRO_SCALE;
  float gz_dps = gz / GYRO_SCALE;

  // accelerometer in g
  float ax_g = (float)ax / ACCEL_SCALE;
  float ay_g = (float)ay / ACCEL_SCALE;
  float az_g = (float)az / ACCEL_SCALE;

  // --- Base-swapped calculation (you requested roll/pitch base swap earlier) ---
  // Original: rollAcc = atan2(ay, az); pitchAcc = atan2(-ax, sqrt(...));
  // We swap them at the base so variable names keep consistent later.
  float pitchAcc = atan2f(ay_g, az_g) * 180.0f / PI;  // used as pitch
  float rollAcc  = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 180.0f / PI; // used as roll

  // integrate gyro to get angle deltas
  float rollGyroDelta  = gy_dps * dt; // note swapped: gx/gy mapping chosen to match earlier swap
  float pitchGyroDelta = gx_dps * dt;
  float yawGyroDelta   = gz_dps * dt;

  // Complementary filter for roll/pitch (swapped base)
  attitudeRoll  = ANGLE_ALPHA * (attitudeRoll + rollGyroDelta)  + (1.0f - ANGLE_ALPHA) * rollAcc;
  attitudePitch = ANGLE_ALPHA * (attitudePitch + pitchGyroDelta) + (1.0f - ANGLE_ALPHA) * pitchAcc;

  // Yaw integration with optional magnetometer fusion:
  if (magAvailable && magUpdated) {
    // Fuse gyro-integrated yaw and magnetometer heading using complementary filter.
    // magHeadingDeg is absolute heading in degrees (0..360 or -180..180).
    // Ensure both angles are in same reference and handle wrapping.
    // 1) integrate gyro
    float yawFromGyro = attitudeYaw + yawGyroDelta;
    // 2) compute shortest difference between mag heading and gyro yaw
    float magHeadingWrapped = wrap180(magHeadingDeg);
    float diff = shortestAngleDiff(magHeadingWrapped, yawFromGyro);
    // 3) apply complementary fusion: pull gyro yaw toward mag heading using diff
    float fusedYaw = yawFromGyro + (1.0f - YAW_ALPHA) * diff;
    attitudeYaw = wrap180(fusedYaw);
    magUpdated = false; // consume the mag update until next setMagHeading call
  } else {
    // No mag: just integrate gyro (after bias subtraction)
    attitudeYaw = wrap180(attitudeYaw + yawGyroDelta);
  }
}

// Public getter
inline Orientation getOrientation() {
  mpuUpdate();
  Orientation o;
  o.roll  = attitudeRoll;
  o.pitch = attitudePitch;
  o.yaw   = attitudeYaw;
  return o;
}

#endif // MPU_H
