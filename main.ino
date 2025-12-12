#include <Arduino.h>
#include "remotexy_module.h"
#include "battery.h"   // Include your header file
#include "mpu.h"
#include "bmp280_module.h"

void setup() {
  Serial.begin(115200);
  RemoteXYModule::begin();
  Serial.println("RemoteXY started");
  mpuBegin();
  
 // analogReadResolution(12);   // ESP32 uses 12-bit ADC
  delay(500);
  mpuBegin(true, 250); // auto-calibrate gyro on begin with 250 samples
  mpuCalibrateGyro();
  Serial.println("MPU ready. Use mpuCalibrateGyro() later if needed.");
  delay(2000);
  Serial.println("Battery voltage measurement test");

   if (!bmp280ModuleBegin()) {
    Serial.println("BMP280 not detected!");
    while (1);
  }
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

//
  delay(100);
  //delay(1000); // measure every 1 sec
}
