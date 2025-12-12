#include <Arduino.h>
#include "remotexy_module.h"
#include "battery.h"   // Include your header file
#include "mpu.h"
#include "bmp280_module.h"
#include "pid.h"
#include "mixer.h"


MixerQuadX mixer;


// --- PID Instances (tune later) ---
PID pidRoll(4.0, 0.01, 1.5);   
PID pidPitch(4.0, 0.01, 1.5);
PID pidYaw(2.0, 0.005, 0.0);

// Altitude PID
PID pidAltitude(1.5, 0.3, 0.8);  

// For smoothing noisy barometer altitude
float altFiltered = 0;
float targetAltitude = 0;
bool altHold = false;


void setup() {
  Serial.begin(115200);

//--------------remote setup------------
  RemoteXYModule::begin();
  Serial.println("RemoteXY started");
  

//-----------mpu setting up-----------
  mpuBegin();
  delay(500);
  mpuBegin(true, 250); // auto-calibrate gyro on begin with 250 samples
  mpuCalibrateGyro();
  Serial.println("MPU ready. Use mpuCalibrateGyro() later if needed.");
  delay(2000);
  Serial.println("Battery voltage measurement test");

//-----------------bmp setup---------------

   if (!bmp280ModuleBegin()) {
    Serial.println("BMP280 not detected!");
    while (1);
  }

  // ================= PID CONFIGURATION =================

  // Roll / Pitch / Yaw limits
  pidRoll.setLimits(-0.3f, 0.3f);
  pidPitch.setLimits(-0.3f, 0.3f);
  pidYaw.setLimits(-0.2f, 0.2f);

  // Altitude PID (THIS is what you asked about)
  pidAltitude.setDeadband(0.05f);   // ignore ±5 cm altitude noise
  pidAltitude.setLimits(-0.3f, 0.3f);

  Serial.println("PID configured");
//-------------------mixer range------------------
  mixer.setOutputRange(0.0f, 1.0f);

  delay(1000);

}

void loop() {
  //---------remote controller-------------
  RemoteXYModule::loop();          
   static unsigned long t = 0;
  if (millis() - t > 200) {
    t = millis();
    auto c = RemoteXYModule::getControls();

    Serial.print("P:"); Serial.print(c.pitch);
    Serial.print(" R:"); Serial.print(c.roll);
    Serial.print(" T:"); Serial.print(c.throttle);
    Serial.print(" Y:"); Serial.print(c.yaw);
    Serial.print(" Conn:"); Serial.print(c.connected);


    float rollSet     = c.roll    * 0.3f;    // scale input (-100..100)
    float pitchSet    = c.pitch   * 0.3f;
    float yawSet      = c.yaw     * 0.3f;    // yaw rate
    float throttleSet = (c.throttle + 100) / 200.0f;  // normalize 0..1

  } //
//------------------remote controller-------------------

//--------------------------------battery setup-----------------------------------------------------------
  float voltage = getBatteryVoltage();
  Serial.print(" Battery Voltage: ");
  Serial.print(voltage);
  Serial.print(" V");
  RemoteXYModule::setBattery(voltage);

//--------------------------------battery setup-------------------------------

//--------------------------------MPU setup-------------------------------
  Orientation o = getOrientation();

  Serial.print(" Roll: ");  Serial.print(o.roll);
  Serial.print("  Pitch: "); Serial.print(o.pitch);
  Serial.print("  Yaw: "); Serial.print(o.yaw);

//--------------------------------MPU setup-------------------------------

//--------------------------------BMP setup-------------------------------
  BMPData d = getBMP();

  Serial.print(" T: ");
  Serial.print(d.temperature);
  Serial.print(" C  P: ");
  Serial.print(d.pressure);
  Serial.print(" Pa  Alt: ");
  Serial.println(d.altitude);
//--------------------------------BMP setup-------------------------------
//---- --- Altitude Filtering ---
  static bool firstAlt = true;
  if (firstAlt) {
    altFiltered = d.altitude;
    firstAlt = false;
  }
  altFiltered = altitudeLPF(d.altitude, altFiltered, 0.90); // from pid.h

//
  delay(100);
  //delay(1000); // measure every 1 sec
}
