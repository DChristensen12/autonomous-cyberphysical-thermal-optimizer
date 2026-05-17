#include "ThermalPlant.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// Kept these at file scope rather than as members so the OneWire and
// DallasTemperature includes don't leak into the header. The rest of the
// project doesn't need to know which sensor library I'm using.
namespace {
    OneWire           one_wire(pins::DS18B20_DATA);
    DallasTemperature sensors(&one_wire);
    bool              conversion_pending = false;
    uint32_t          conversion_request_us = 0;
}

ThermalPlant::ThermalPlant()
    : kp_(0.0f),
      kd_(0.0f),
      setpoint_c_(control::SETPOINT_C),
      last_temp_c_(25.0f),   // reasonable room-temp default until first read
      last_error_c_(0.0f),
      prev_error_c_(0.0f),
      first_tick_(true)
{}

void ThermalPlant::begin() {
    pinMode(pins::HEATER_PWM, OUTPUT);
    pinMode(pins::FAN_PWM,    OUTPUT);
    pinMode(pins::STATUS_LED, OUTPUT);

    analogWriteResolution(control::PWM_RESOLUTION_BITS);

    sensors.begin();
    sensors.setWaitForConversion(false);  // async — we'll check back later
    sensors.setResolution(10);             // 10-bit ~= 187 ms conversions

    drive_actuators(0.0f);
}

void ThermalPlant::request_temperature_async() {
    if (!conversion_pending) {
        sensors.requestTemperatures();
        conversion_request_us = micros();
        conversion_pending = true;
    }
}

float ThermalPlant::read_temperature_c() {
    if (conversion_pending) {
        const uint32_t elapsed = micros() - conversion_request_us;
        if (elapsed > 200000) {  // 200 ms is generous for a 10-bit read
            const float t = sensors.getTempCByIndex(0);
            // The DallasTemperature library returns -127 on failure.
            // Bracket the sane range and ignore garbage.
            if (t > -100.0f && t < 200.0f) last_temp_c_ = t;
            conversion_pending = false;
        }
    }
    return last_temp_c_;
}

void ThermalPlant::set_gains(float kp, float kd) {
    kp_ = kp;
    kd_ = kd;
    first_tick_ = true;  // wipe derivative history so the gains don't kick
}

void ThermalPlant::pid_tick(float dt_s) {
    request_temperature_async();
    const float t = read_temperature_c();

    if (t > control::SAFETY_MAX_C) {
        emergency_stop();
        return;
    }

    const float error = setpoint_c_ - t;
    last_error_c_ = error;

    float derivative = 0.0f;
    if (!first_tick_ && dt_s > 1e-6f) {
        derivative = (error - prev_error_c_) / dt_s;
    }
    first_tick_ = false;
    prev_error_c_ = error;

    drive_actuators(kp_ * error + kd_ * derivative);
}

void ThermalPlant::drive_actuators(float control_signal) {
    // Clamp into [-1, 1] and split into heater/fan duty.
    if (control_signal >  1.0f) control_signal =  1.0f;
    if (control_signal < -1.0f) control_signal = -1.0f;

    int heater_pwm = 0;
    int fan_pwm    = 0;
    if (control_signal > 0.0f) heater_pwm = (int)( control_signal * control::PWM_MAX);
    else                       fan_pwm    = (int)(-control_signal * control::PWM_MAX);

    analogWrite(pins::HEATER_PWM, heater_pwm);
    analogWrite(pins::FAN_PWM,    fan_pwm);
}

void ThermalPlant::emergency_stop() {
    analogWrite(pins::HEATER_PWM, 0);
    analogWrite(pins::FAN_PWM,    control::PWM_MAX);
}
