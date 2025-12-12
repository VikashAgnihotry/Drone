#ifndef REMOTEXY_MODULE_H
#define REMOTEXY_MODULE_H

// ========== RemoteXY Connection Mode (MUST be before <RemoteXY.h>) ==========
#define REMOTEXY_MODE__WIFI_POINT

#include <WiFi.h>
#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

// ========== RemoteXY Generated GUI Configuration ==========
#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 47 bytes V19 
{ 255,4,0,4,0,40,0,19,0,0,0,0,31,1,200,80,1,1,3,0,
  5,10,8,60,60,32,2,26,31,5,128,7,60,60,0,2,26,31,67,82,
  17,32,13,109,2,26,2 };
#pragma pack(pop)

// ========== RemoteXY Variable Structure ==========
struct {
  // Input variables
  int8_t roll_x;      // -100…100
  int8_t pitch_y;     // -100…100
  int8_t yaw_x;       // -100…100
  int8_t throttle_y;  // -100…100

  // Output
  float Battery;

  // Connection flag
  uint8_t connect_flag;  // =1 if connected
} RemoteXY;

#include <RemoteXY.h>

// ========== MODULE API ==========
namespace RemoteXYModule {

  struct Controls {
    int8_t roll;
    int8_t pitch;
    int8_t yaw;
    int8_t throttle;
    bool connected;
  };

  // Initialize RemoteXY
  inline void begin() {
    RemoteXY_Init();
  }

  // Must be called every loop
  inline void loop() {
    RemoteXYEngine.handler();
    RemoteXYEngine.delay(1);
  }

  // Return joystick values in a clean struct
  inline Controls getControls() {
    Controls c;
    c.roll     = RemoteXY.roll_x;
    c.pitch    = RemoteXY.pitch_y;
    c.yaw      = RemoteXY.yaw_x;
    c.throttle = RemoteXY.throttle_y;
    c.connected = (RemoteXY.connect_flag != 0);
    return c;
  }

  // Throttle normalized to 0.0 – 1.0
  inline float getThrottleNormalized() {
    return (RemoteXY.throttle_y + 100) / 200.0f;
  }

  // Set battery voltage to display
  inline void setBattery(float volts) {
    RemoteXY.Battery = volts;
  }

  // Check if phone is connected
  inline bool isConnected() {
    return (RemoteXY.connect_flag != 0);
  }

} // namespace RemoteXYModule

#endif // REMOTEXY_MODULE_H
