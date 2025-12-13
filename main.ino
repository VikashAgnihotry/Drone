#include <Arduino.h>
#include "remotexy_module.h"
#include "battery.h"
#include "mpu.h"
#include "bmp280_module.h"
#include "pid.h"
#include "mixer.h"
#include "esc_driver.h"
#include "arm_failsafe.h"

// ================= ESC PINS =================
#define ESC_FL 26
#define ESC_FR 14
#define ESC_RR 25
#define ESC_RL 27

// ================= OBJECTS =================
MixerQuadX mixer;

// PID controllers
PID pidRoll(4.0, 0.01, 1.5);
PID pidPitch(4.0, 0.01, 1.5);
PID pidYaw(2.0, 0.005, 0.0);
PID pidAltitude(1.5, 0.3, 0.8);

// Altitude state
float altFiltered = 0.0f;
float targetAltitude = 0.0f;
bool altitudeInitialized = false;

// ================= CONSTANTS =================
const float HOVER_THROTTLE = 0.10f;      // tune for your drone
const float MAX_CLIMB_RATE = 1.0f;       // m/s
const float THROTTLE_DEADBAND = 0.05f;   // stick noise

// ================= ARM CALLBACKS =================
void onArm() {
  escArm();
  Serial.println(">>> ESC ARMED <<<");
}

void onDisarm() {
  escDisarm();
  pidAltitude.reset();   // safety
  Serial.println(">>> ESC DISARMED <<<");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // ---------- RemoteXY ----------
  RemoteXYModule::begin();
  Serial.println("RemoteXY started");

  // ---------- MPU ----------
  mpuBegin(true, 250);
  mpuCalibrateGyro();
  Serial.println("MPU ready");

  // ---------- BMP ----------
  if (!bmp280ModuleBegin()) {
    Serial.println("BMP280 not detected!");
    while (1);
  }

  // ---------- PID CONFIG ----------
  pidRoll.setLimits(-0.3f, 0.3f);
  pidPitch.setLimits(-0.3f, 0.3f);
  pidYaw.setLimits(-0.2f, 0.2f);

  pidAltitude.setDeadband(0.05f);
  pidAltitude.setLimits(-0.3f, 0.3f);

  // ---------- Mixer ----------
  mixer.setOutputRange(0.0f, 1.0f);

  // ---------- ESC ----------
  escBegin(ESC_FL, ESC_FR, ESC_RR, ESC_RL);
  Serial.println("ESC initialized (DISARMED)");

  delay(1000);
}

// ================= LOOP =================
void loop() {

  // ---------- RemoteXY ----------
  RemoteXYModule::loop();
  RemoteXYModule::Controls c = RemoteXYModule::getControls();

  // ---------- ARM / FAILSAFE MODULE ----------
  armFailsafeUpdate(
    c.throttle,
    RemoteXYModule::isConnected(),
    onArm,
    onDisarm
  );

  // ---------- HARD FAILSAFE ----------
  if (!RemoteXYModule::isConnected() || !isArmed()) {
    escEmergencyStop();
    return;
  }

  // ---------- Battery ----------
  float voltage = getBatteryVoltage();
  RemoteXYModule::setBattery(voltage);

  // ---------- MPU ----------
  Orientation o = getOrientation();

  // ---------- BMP / ALTITUDE ----------
  BMPData d = getBMP();

  if (!altitudeInitialized) {
    altFiltered = d.altitude;
    targetAltitude = d.altitude;
    altitudeInitialized = true;
  }

  altFiltered = altitudeLPF(d.altitude, altFiltered, 0.90f);

  // ---------- ALTITUDE HOLD ----------
  static unsigned long lastAltTime = millis();
  unsigned long now = millis();
  float dt = (now - lastAltTime) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  lastAltTime = now;

  float throttleCmd = c.throttle / 100.0f;
  if (fabs(throttleCmd) < THROTTLE_DEADBAND) throttleCmd = 0.0f;

  targetAltitude += throttleCmd * MAX_CLIMB_RATE * dt;

  // Clamp target altitude
  targetAltitude = constrain(
    targetAltitude,
    altFiltered - 2.0f,
    altFiltered + 2.0f
  );

  float throttle =
    HOVER_THROTTLE +
    pidAltitude.compute(targetAltitude, altFiltered);

  throttle = constrain(throttle, 0.0f, 1.0f);

  // ---------- ATTITUDE PID ----------
  float rollSet  = c.roll  * 0.3f;
  float pitchSet = c.pitch * 0.3f;
  float yawSet   = c.yaw   * 0.3f;

  float rollOut  = pidRoll.compute(rollSet, o.roll);
  float pitchOut = pidPitch.compute(pitchSet, o.pitch);
  float yawOut   = pidYaw.compute(yawSet, 0.0f); // yaw rate

  // ---------- MIXER ----------
  MotorOutputs motors = mixer.mix(
    throttle,
    rollOut,
    pitchOut,
    yawOut
  );

  // ---------- ESC OUTPUT ----------
  escWriteMotors(
    motors.m[0],
    motors.m[1],
    motors.m[2],
    motors.m[3]
  );

  // ---------- DEBUG ----------
  static unsigned long dbgT = 0;
  if (millis() - dbgT > 200) {
    dbgT = millis();
    Serial.print("ALT:"); Serial.print(altFiltered, 2);
    Serial.print(" TGT:"); Serial.print(targetAltitude, 2);
    Serial.print(" THR:"); Serial.print(throttle, 2);
    Serial.print(" M:");
    Serial.print(motors.m[0], 2); Serial.print(",");
    Serial.print(motors.m[1], 2); Serial.print(",");
    Serial.print(motors.m[2], 2); Serial.print(",");
    Serial.println(motors.m[3], 2);
  }
}
