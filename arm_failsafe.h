#ifndef ARM_FAILSAFE_H
#define ARM_FAILSAFE_H

#include <Arduino.h>

/*
   ARM / DISARM + FAILSAFE LOGIC

   Throttle-based arming:
     - Hold throttle <= ARM_THROTTLE_LOW for ARM_HOLD_TIME ms
     - Requires throttle release (> ARM_THROTTLE_HIGH) before re-trigger
     - Same action toggles ARM <-> DISARM

   Designed for spring-centered throttle (-100..100)
*/

// ================= CONFIG =================
#define ARM_THROTTLE_LOW   -40
#define ARM_THROTTLE_HIGH  -10
#define ARM_HOLD_TIME      2000   // ms

// ================= STATE =================
static bool _armed = false;

static bool _stickReleased = true;
static bool _timerRunning  = false;
static unsigned long _timerStart = 0;

// ================= API =================

// Call this EVERY loop after reading throttle
// throttle : -100 .. +100
// connected: true if RemoteXY connected
// onArm() / onDisarm() are callbacks (ESC control handled outside)
inline void armFailsafeUpdate(
    int throttle,
    bool connected,
    void (*onArm)(),
    void (*onDisarm)()
) {
    // -------- FAILSAFE --------
    if (!connected) {
        if (_armed) {
            _armed = false;
            onDisarm();
        }
        _timerRunning = false;
        _stickReleased = true;
        return;
    }

    // -------- STICK RELEASE DETECTION --------
    if (throttle > ARM_THROTTLE_HIGH) {
        _stickReleased = true;
        _timerRunning = false;
    }

    // -------- ARM / DISARM DETECTION --------
    if (_stickReleased && throttle <= ARM_THROTTLE_LOW) {

        if (!_timerRunning) {
            _timerRunning = true;
            _timerStart = millis();
        }

        if (millis() - _timerStart >= ARM_HOLD_TIME) {

            if (!_armed) {
                _armed = true;
                onArm();
            } else {
                _armed = false;
                onDisarm();
            }

            // lock until stick is released
            _stickReleased = false;
            _timerRunning = false;
        }
    } else {
        _timerRunning = false;
    }
}

// Query armed state
inline bool isArmed() {
    return _armed;
}

// Force disarm (used by low-battery, crash, etc.)
inline void forceDisarm(void (*onDisarm)()) {
    if (_armed) {
        _armed = false;
        onDisarm();
    }
    _timerRunning = false;
    _stickReleased = true;
}

#endif // ARM_FAILSAFE_H
