#include "ThermalPlant.h"
#include <math.h>

ThermalPlant::ThermalPlant()
    : pid_(control::SETPOINT_C),
      last_temp_c_(25.0f)   // sane room temp default until the first real read
{}

void ThermalPlant::begin() {
    pinMode(pins::HEATER_PWM, OUTPUT);
    pinMode(pins::FAN_PWM,    OUTPUT);
    pinMode(pins::STATUS_LED, OUTPUT);

    analogWriteResolution(control::PWM_RESOLUTION_BITS);
    analogReadResolution(thermistor::ADC_BITS);

    drive_actuators(0.0f);
}

// Read the divider, average a handful of samples to settle the noise, and
// convert to Celsius with the Beta equation. The ADC noise on a bare divider
// is real, so the oversampling earns its keep here.
float ThermalPlant::read_temperature_c() {
    uint32_t acc = 0;
    for (int i = 0; i < thermistor::ADC_OVERSAMPLE; ++i) {
        acc += analogRead(pins::THERMISTOR_ADC);
    }
    const float counts = (float)acc / (float)thermistor::ADC_OVERSAMPLE;

    // Guard the ends so we never divide by zero if a wire falls out and the
    // reading pins to a rail.
    if (counts <= 1.0f || counts >= (float)thermistor::ADC_MAX - 1.0f) {
        return last_temp_c_;  // keep the last good value rather than spike
    }

    // Tap voltage as a fraction of full scale. With the thermistor on top and
    // the fixed resistor on the bottom, the tap fraction is the bottom
    // resistor over the total, so we can back out the thermistor resistance.
    const float frac = counts / (float)thermistor::ADC_MAX;

    // frac = Rfixed / (Rtherm + Rfixed)  ->  Rtherm = Rfixed * (1/frac - 1)
    const float r_therm = thermistor::SERIES_RESISTOR * (1.0f / frac - 1.0f);

    // Beta equation, solved for temperature in Kelvin:
    //   1/T = 1/T0 + (1/Beta) * ln(R/R0)
    const float t0_k = thermistor::NOMINAL_TEMP_C + 273.15f;
    float inv_t = 1.0f / t0_k
                + (1.0f / thermistor::BETA)
                  * logf(r_therm / thermistor::NOMINAL_RES);
    const float temp_c = (1.0f / inv_t) - 273.15f;

    last_temp_c_ = temp_c;
    return temp_c;
}

void ThermalPlant::set_gains(float kp, float kd) {
    pid_.set_gains(kp, kd);
}

void ThermalPlant::pid_tick(float dt_s) {
    const float t = read_temperature_c();

    if (t > control::SAFETY_MAX_C) {
        emergency_stop();
        return;
    }

    drive_actuators(pid_.compute(t, dt_s));
}

// Positive control runs the heater, negative runs the fan. The fan side is
// clamped to FAN_MAX_DUTY because it is a 5V fan sitting on the 9V rail.
void ThermalPlant::drive_actuators(float control_signal) {
    if (control_signal >  1.0f) control_signal =  1.0f;
    if (control_signal < -1.0f) control_signal = -1.0f;

    int heater_pwm = 0;
    int fan_pwm    = 0;

    if (control_signal > 0.0f) {
        heater_pwm = (int)(control_signal * control::PWM_MAX);
    } else {
        float fan_duty = -control_signal;
        if (fan_duty > control::FAN_MAX_DUTY) fan_duty = control::FAN_MAX_DUTY;
        fan_pwm = (int)(fan_duty * control::PWM_MAX);
    }

    analogWrite(pins::HEATER_PWM, heater_pwm);
    analogWrite(pins::FAN_PWM,    fan_pwm);
}

void ThermalPlant::emergency_stop() {
    analogWrite(pins::HEATER_PWM, 0);
    analogWrite(pins::FAN_PWM,    control::PWM_MAX);
}
