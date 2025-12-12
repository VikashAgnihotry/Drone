#ifndef MIXER_H
#define MIXER_H

#include <Arduino.h>

/*
   Quad-X Motor Mixer

   Motor order:
     m[0] = Front Left  (FL)
     m[1] = Front Right (FR)
     m[2] = Rear Right  (RR)
     m[3] = Rear Left   (RL)

   Coordinate assumptions:
     +Roll  → right
     +Pitch → nose up
     +Yaw   → clockwise
*/

struct MotorOutputs {
  float m[4];
};

class MixerQuadX {
public:
  MixerQuadX() {
    minOut = 0.0f;
    maxOut = 1.0f;
  }

  // Set output range (normally 0..1)
  void setOutputRange(float minVal, float maxVal) {
    minOut = minVal;
    maxOut = maxVal;
  }

  // Core mixer function
  MotorOutputs mix(float throttle,
                   float roll,
                   float pitch,
                   float yaw) {

    MotorOutputs out;

    // -------- Quad-X mixing --------
    out.m[0] = throttle + pitch + roll - yaw; // Front Left
    out.m[1] = throttle + pitch - roll + yaw; // Front Right
    out.m[2] = throttle - pitch - roll - yaw; // Rear Right
    out.m[3] = throttle - pitch + roll + yaw; // Rear Left

    // -------- Clamp outputs --------
    for (int i = 0; i < 4; i++) {
      if (out.m[i] > maxOut) out.m[i] = maxOut;
      if (out.m[i] < minOut) out.m[i] = minOut;
    }

    return out;
  }

  // Convert normalized value (0..1) to ESC microseconds
  static inline int toMicroseconds(float value,
                                   int minUs = 1000,
                                   int maxUs = 2000) {
    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;
    return minUs + (int)(value * (maxUs - minUs));
  }

private:
  float minOut;
  float maxOut;
};

#endif // MIXER_H