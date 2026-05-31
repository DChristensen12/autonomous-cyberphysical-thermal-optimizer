// Wraps the physical side of the project: read the thermistor, drive the
// heater and fan, run a PID step. The PID math itself lives in PIDController
// so the hardware and the laptop simulation run the identical controller.

#ifndef THERMAL_PLANT_H
#define THERMAL_PLANT_H

#include <Arduino.h>
#include "acpto_config.h"
#include "PIDController.h"

class ThermalPlant {
public:
    ThermalPlant();

    // Pin setup, PWM resolution, ADC resolution. Call from setup().
    void begin();

    // Read the divider and return the block temperature in Celsius. Averages
    // a handful of ADC samples internally to settle the noise.
    float read_temperature_c();

    // The optimizer pokes new gains in here between trials.
    void set_gains(float kp, float kd);

    // One PID step. dt_s is the time since the last call.
    void pid_tick(float dt_s);

    // Hard shutdown. Heater off, fan full blast.
    void emergency_stop();

    // For logging.
    float last_temperature() const { return last_temp_c_; }
    float last_error()       const { return pid_.last_error(); }

private:
    // Positive control runs the heater, negative runs the fan. Magnitude is
    // duty cycle.
    void drive_actuators(float control_signal);

    PIDController pid_;
    float last_temp_c_;
};

#endif
