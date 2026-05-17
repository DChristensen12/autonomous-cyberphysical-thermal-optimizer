// Wraps the physical side of the project: read the DS18B20, drive the heater
// and fan, run a PID step. Deliberately no integral term — the GP is tuning
// Kp and Kd only and an extra I term would just confuse the performance score.

#ifndef THERMAL_PLANT_H
#define THERMAL_PLANT_H

#include <Arduino.h>
#include "acpto_config.h"

class ThermalPlant {
public:
    ThermalPlant();

    // Pin setup, PWM resolution, sensor bus. Call from setup().
    void begin();

    // The DS18B20 takes ~200 ms to do a 10-bit conversion. Calling
    // request_temperature_async() at the top of a tick and read_temperature_c()
    // a tick or two later avoids ever blocking.
    void  request_temperature_async();
    float read_temperature_c();

    // The optimizer pokes new gains in here between trials.
    void set_gains(float kp, float kd);

    // One PID step. dt_s is the time since the last call.
    void pid_tick(float dt_s);

    // Hard shutdown. Heater off, fan full blast.
    void emergency_stop();

    // For logging.
    float last_temperature() const { return last_temp_c_; }
    float last_error()       const { return last_error_c_; }

private:
    // Positive control -> heater. Negative -> fan. Magnitude is duty cycle.
    void drive_actuators(float control_signal);

    float kp_;
    float kd_;
    float setpoint_c_;
    float last_temp_c_;
    float last_error_c_;
    float prev_error_c_;
    bool  first_tick_;   // skip the derivative on the very first step
};

#endif
