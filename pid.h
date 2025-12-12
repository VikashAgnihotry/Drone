#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PID {
public:
    PID(float kp = 0, float ki = 0, float kd = 0) {
        setGains(kp, ki, kd);
        setLimits(-400.0f, 400.0f);
        setDFilter(0.02f);
        lastTime = millis();
    }

    void setGains(float kp, float ki, float kd) {
        Kp = kp; Ki = ki; Kd = kd;
    }

    void setLimits(float minOut, float maxOut) {
        outMin = minOut; outMax = maxOut;
    }

    void setDFilter(float tau) {
        dFilterTau = tau;
    }

    void reset() {
        integrator = 0;
        lastMeasurement = 0;
        dFiltered = 0;
        lastTime = millis();
    }

    float compute(float setpoint, float measurement) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0.00001f) dt = 0.001f;
        lastTime = now;

        float error = setpoint - measurement;

        float P = Kp * error;

        integrator += Ki * error * dt;
        if (integrator > outMax) integrator = outMax;
        if (integrator < outMin) integrator = outMin;

        float dMeas = (measurement - lastMeasurement) / dt;

        float alpha = dFilterTau / (dFilterTau + dt);
        dFiltered = alpha * dFiltered + (1 - alpha) * dMeas;

        float D = -Kd * dFiltered;

        lastMeasurement = measurement;

        float out = P + integrator + D;

        if (out > outMax) out = outMax;
        if (out < outMin) out = outMin;

        return out;
    }

private:
    float Kp, Ki, Kd;
    float outMin, outMax;
    float integrator = 0.0;
    float lastMeasurement = 0.0;
    float dFilterTau = 0.02f;
    float dFiltered = 0.0;
    unsigned long lastTime = 0;
};

// Simple low-pass filter for altitude
inline float altitudeLPF(float current, float prev, float alpha = 0.9f) {
    return prev * alpha + current * (1 - alpha);
}

#endif
