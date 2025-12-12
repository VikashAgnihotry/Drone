#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

// -------- CONFIG -------
const int BATTERY_PIN = 35;      // ADC pin
const float R1_OHMS = 30000.0;   // 30kΩ
const float R2_OHMS = 7500.0;    // 7.5kΩ
// Divider ratio = (R1+R2)/R2 = 5
// --------------------------------------------

// ADC settings
const int ADC_BITS = 12;
const int ADC_MAX = (1 << ADC_BITS) - 1; // 4095
const float ADC_VREF = 3.3;              // volts

const int AVG_SAMPLES = 20;

inline void batteryBegin() {
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);  // allows reading up to ~3.6V
}

inline float getBatteryVoltage() {
  long sum = 0;

  for (int i = 0; i < AVG_SAMPLES; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(2);
  }

  float raw = (float)sum / AVG_SAMPLES;
  float vOut = (raw / ADC_MAX) * ADC_VREF;               // voltage at ADC pin
  float batteryVoltage  = vOut * ((R1_OHMS + R2_OHMS) / R2_OHMS); // multiply by divider ratio

  return batteryVoltage+ 0.8;
}

#endif
