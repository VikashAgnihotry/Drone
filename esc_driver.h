#ifndef ESC_DRIVER_H
#define ESC_DRIVER_H

#include <Arduino.h>

/*
  ESP32 ESC Driver (SAFE VERSION)

  - Avoids GCC internal compiler errors
  - Uses LEDC hardware PWM
  - 50Hz, 1000–2000us
*/

#define ESC_PWM_FREQ 50
#define ESC_PWM_RES  16
#define ESC_MIN_US   1000
#define ESC_MAX_US   2000

static uint8_t escPins[4];
static uint8_t escChannels[4] = {0, 1, 2, 3};
static bool escArmed = false;

// Convert microseconds to LEDC duty (integer math only)
static uint32_t escUsToDuty(uint16_t us) {
  const uint32_t maxDuty = (1UL << ESC_PWM_RES) - 1;
  const uint32_t periodUs = 1000000UL / ESC_PWM_FREQ; // 20000us
  return (uint32_t)((uint64_t)us * maxDuty / periodUs);
}

// ---------------- API ----------------

inline void escBegin(uint8_t fl, uint8_t fr, uint8_t rr, uint8_t rl) {
  escPins[0] = fl;
  escPins[1] = fr;
  escPins[2] = rr;
  escPins[3] = rl;

  for (int i = 0; i < 4; i++) {
    ledcSetup(escChannels[i], ESC_PWM_FREQ, ESC_PWM_RES);
    ledcAttachPin(escPins[i], escChannels[i]);
    ledcWrite(escChannels[i], escUsToDuty(ESC_MIN_US));
  }

  escArmed = false;
}

inline void escArm() {
  escArmed = true;
}

inline void escDisarm() {
  escArmed = false;
  for (int i = 0; i < 4; i++) {
    ledcWrite(escChannels[i], escUsToDuty(ESC_MIN_US));
  }
}

// 🔑 IMPORTANT CHANGE:
// write ESCs using **individual values**, not float arrays
inline void escWriteMotors(
  float m0, float m1, float m2, float m3
) {
  if (!escArmed) {
    escDisarm();
    return;
  }

  float m[4] = {m0, m1, m2, m3};

  for (int i = 0; i < 4; i++) {
    if (m[i] < 0.0f) m[i] = 0.0f;
    if (m[i] > 1.0f) m[i] = 1.0f;

    uint16_t us = ESC_MIN_US +
                  (uint16_t)(m[i] * (ESC_MAX_US - ESC_MIN_US));

    ledcWrite(escChannels[i], escUsToDuty(us));
  }
}

inline void escEmergencyStop() {
  escDisarm();
}

#endif
