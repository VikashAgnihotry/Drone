#ifndef BMP280_MODULE_H
#define BMP280_MODULE_H

#include <Adafruit_BMP280.h>

struct BMPData {
  float temperature;
  float pressure;
  float altitude;
};

static Adafruit_BMP280 bmp;

inline bool bmp280ModuleBegin(uint8_t addr = 0x76) {
  return bmp.begin(addr);
}

inline BMPData getBMP() {
  BMPData d;
  d.temperature = bmp.readTemperature();        // °C
  d.pressure    = bmp.readPressure();           // Pa
  d.altitude    = bmp.readAltitude(1013.25);    // meters
  return d;
}

#endif