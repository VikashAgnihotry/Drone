#ifndef REMOTEXY_MODULE_H
#define REMOTEXY_MODULE_H

/******************** RemoteXY connection settings ********************/
#define REMOTEXY_MODE__WIFI_POINT

#include <WiFi.h>

#define REMOTEXY_WIFI_SSID       "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD   "12345678"
#define REMOTEXY_SERVER_PORT     6377

/******************** RemoteXY GUI configuration ********************/
#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 47 bytes V19
{
  255,4,0,4,0,40,0,19,0,0,0,0,31,1,200,80,1,1,3,0,
  5,10,8,60,60,32,2,26,31,
  5,128,7,60,60,32,2,26,31,
  67,82,17,32,13,109,2,26,2
};
#pragma pack(pop)

/******************** RemoteXY variable structure ********************/
struct {
  // INPUTS (spring-centered joysticks)
  int8_t roll_x;       // -100 .. +100
  int8_t pitch_y;      // -100 .. +100
  int8_t yaw_x;        // -100 .. +100
  int8_t throttle_y;   // -100 .. +100 (CENTER = 0 → altitude hold)

  // OUTPUT
  float Battery;

  // STATUS
  uint8_t connect_flag;  // 1 = connected
} RemoteXY;

#include <RemoteXY.h>

/******************** MODULE API ********************/
namespace RemoteXYModule {

  // Unified control struct for flight controller
  struct Controls {
    int8_t roll;        // -100..100
    int8_t pitch;       // -100..100
    int8_t yaw;         // -100..100
    int8_t throttle;    // -100..100 (centered)
    bool connected;
  };

  // Call ONCE in setup()
  inline void begin() {
    RemoteXY_Init();
  }

  // Call EVERY loop()
  inline void loop() {
    RemoteXYEngine.handler();
    RemoteXYEngine.delay(1);   // DO NOT use delay()
  }

  // Read joystick inputs
  inline Controls getControls() {
    Controls c;
    c.roll      = RemoteXY.roll_x;
    c.pitch     = RemoteXY.pitch_y;
    c.yaw       = RemoteXY.yaw_x;
    c.throttle  = RemoteXY.throttle_y;
    c.connected = (RemoteXY.connect_flag != 0);
    return c;
  }

  // Throttle mapped to climb rate logic (for altitude hold)
  inline float getThrottleNormalized() {
    // Converts -100..+100 → -1.0 .. +1.0
    return RemoteXY.throttle_y / 100.0f;
  }

  // Update battery voltage on app
  inline void setBattery(float volts) {
    RemoteXY.Battery = volts;
  }

  inline bool isConnected() {
    return (RemoteXY.connect_flag != 0);
  }

} // namespace RemoteXYModule

#endif // REMOTEXY_MODULE_H
