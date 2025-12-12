#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

/*
   Generic PID controller
   - Works for Roll / Pitch / Yaw / Altitude
   - Anti-windup protection
   - Derivative low-pass filtering
   - Safe timing handling
*/

class PID {
public:
    PID(float kp = 0, float ki = 0, float kd = 0) {
        setGains(kp, ki, kd);
        setLimits(-400.0f, 400.0f);
        setDFilter(0.02f);
        setDeadband(0.0f);
        lastTime = millis();
    }

    void setGains(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    void setLimits(float minOut, float maxOut) {
        outMin = minOut;
        outMax = maxOut;
    }

    void setDFilter(float tau) {
        dFilterTau = tau;
    }

    // Optional: ignore tiny errors (useful for altitude hold)
    void setDeadband(float db) {
        deadband = fabs(db);
    }

    void reset() {
        integrator = 0.0f;
        lastMeasurement = 0.0f;
        dFiltered = 0.0f;
        lastTime = millis();
    }

    float compute(float setpoint, float measurement) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;

        // Protect against bad timing
        if (dt < 0.0005f) dt = 0.0005f;
        if (dt > 0.1f)    dt = 0.1f;

        lastTime = now;

        float error = setpoint - measurement;

        // Apply deadband
        if (fabs(error) < deadband) {
            error = 0.0f;
        }

        // ----- P -----
        float P = Kp * error;

        // ----- I (anti-windup aware) -----
        float newIntegrator = integrator + Ki * error * dt;

        // ----- D (on measurement, filtered) -----
        float dMeas = (measurement - lastMeasurement) / dt;

        float alpha = dFilterTau / (dFilterTau + dt);
        dFiltered = alpha * dFiltered + (1.0f - alpha) * dMeas;

        float D = -Kd * dFiltered;

        // ----- Output before clamping -----
        float out = P + newIntegrator + D;

        // ----- Clamp output -----
        float outClamped = out;
        if (outClamped > outMax) outClamped = outMax;
        if (outClamped < outMin) outClamped = outMin;

        // ----- Accept integrator only if not saturating -----
        if (out == outClamped) {
            integrator = newIntegrator;
        }

        lastMeasurement = measurement;
        return outClamped;
    }

private:
    float Kp = 0, Ki = 0, Kd = 0;
    float outMin = -400.0f, outMax = 400.0f;

    float integrator = 0.0f;
    float lastMeasurement = 0.0f;

    float dFilterTau = 0.02f;
    float dFiltered = 0.0f;

    float deadband = 0.0f;

    unsigned long lastTime = 0;
};

// -------- Simple low-pass filter (used for altitude) --------
inline float altitudeLPF(float current, float prev, float alpha = 0.9f) {
    return prev * alpha + current * (1.0f - alpha);
}

#endif
