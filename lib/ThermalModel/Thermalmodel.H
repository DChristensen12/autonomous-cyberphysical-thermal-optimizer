// A physics model of the aluminum block, the heater, the fan, and the sensor.
// This stands in for the real hardware so the whole optimizer can run on a
// laptop. Nothing in here is Arduino-specific.
//
// The model is a lumped-capacitance thermal system, which is the standard
// first cut for something like this:
//
//   block heats up from the resistor, loses heat to the air, and the rate of
//   loss goes up when the fan is running.
//
//     C * dT_block/dt = P_heater - G(fan) * (T_block - T_ambient)
//
//   the sensor doesn't read the block temperature instantly. A waterproof
//   DS18B20 probe has real thermal mass, so it lags behind:
//
//     tau_sensor * dT_sensor/dt = T_block - T_sensor
//
// The sensor lag is the reason Kd matters at all. Without it the derivative
// term would have nothing useful to do. The lag plus measurement noise is
// also what creates a genuine sweet spot in gain space: too much Kp rings
// against the lag, too much Kd amplifies the noise.
//
// Default parameters are ballpark numbers for a 40x40x20mm block with a 10W
// resistor. They are guesses. Once the real rig exists I'll fit these against
// a step-response capture and replace them.

#ifndef THERMAL_MODEL_H
#define THERMAL_MODEL_H

#include <math.h>
#include <stdint.h>

class ThermalModel {
public:
    struct Params {
        float thermal_mass_j_per_k = 78.0f;   // C. roughly 86g of aluminum.
        float g_natural_w_per_k    = 0.20f;   // loss to still air, fan off
        float g_fan_max_w_per_k    = 0.80f;   // extra loss at full fan
        float heater_max_w         = 10.0f;   // the ceramic resistor at full duty
        float ambient_c            = 25.0f;
        float tau_sensor_s         = 6.0f;    // probe lag. bigger = sluggish sensor
        float sensor_noise_c       = 0.02f;   // gaussian stddev on each read
        float sensor_quant_c       = 0.0625f; // DS18B20 12-bit step. set 0 to disable
    };

    ThermalModel()
        : p_(),
          t_block_(p_.ambient_c),
          t_sensor_(p_.ambient_c),
          rng_state_(0x12345678u)
    {}

    explicit ThermalModel(const Params& p)
        : p_(p),
          t_block_(p_.ambient_c),
          t_sensor_(p_.ambient_c),
          rng_state_(0x12345678u)
    {}

    // Park the block and sensor at a temperature and let everything settle
    // there. Use this at the top of a trial.
    void reset(float initial_c) {
        t_block_  = initial_c;
        t_sensor_ = initial_c;
    }

    void reset_to_ambient() { reset(p_.ambient_c); }

    // Advance the physics by dt seconds. control_signal follows the same
    // convention as the PID output and the real driver: positive runs the
    // heater, negative runs the fan, magnitude is duty in [0, 1].
    // Forward Euler is plenty here since dt (10 ms) is tiny next to the
    // hundreds-of-seconds block time constant.
    void step(float control_signal, float dt_s) {
        if (control_signal >  1.0f) control_signal =  1.0f;
        if (control_signal < -1.0f) control_signal = -1.0f;

        float heater_duty = 0.0f;
        float fan_duty    = 0.0f;
        if (control_signal > 0.0f) heater_duty =  control_signal;
        else                       fan_duty    = -control_signal;

        const float p_in = heater_duty * p_.heater_max_w;
        const float g    = p_.g_natural_w_per_k + fan_duty * p_.g_fan_max_w_per_k;
        const float q_out = g * (t_block_ - p_.ambient_c);

        const float dT_block = (p_in - q_out) / p_.thermal_mass_j_per_k;
        t_block_ += dT_block * dt_s;

        const float dT_sensor = (t_block_ - t_sensor_) / p_.tau_sensor_s;
        t_sensor_ += dT_sensor * dt_s;
    }

    // True block temperature. The controller never gets to see this directly,
    // it's here for logging and for computing the honest tracking error.
    float true_temperature() const { return t_block_; }

    // What the sensor reports: the lagged sensor temperature plus noise and
    // quantization. This is what the controller actually feeds on.
    float measured_temperature() {
        float m = t_sensor_;
        if (p_.sensor_noise_c > 0.0f) {
            m += p_.sensor_noise_c * gaussian();
        }
        if (p_.sensor_quant_c > 0.0f) {
            m = roundf(m / p_.sensor_quant_c) * p_.sensor_quant_c;
        }
        return m;
    }

    const Params& params() const { return p_; }

    // Let a test pin the noise stream so results are repeatable.
    void seed_noise(uint32_t s) { rng_state_ = s ? s : 1u; }

private:
    // Small xorshift PRNG so the model has no dependency on <random> and
    // behaves identically on every platform.
    float uniform01() {
        rng_state_ ^= rng_state_ << 13;
        rng_state_ ^= rng_state_ >> 17;
        rng_state_ ^= rng_state_ << 5;
        return (rng_state_ & 0xFFFFFFu) / float(0x1000000u);
    }

    // Box-Muller, cached second value.
    float gaussian() {
        if (have_spare_) {
            have_spare_ = false;
            return spare_;
        }
        float u1 = uniform01();
        if (u1 < 1e-7f) u1 = 1e-7f;
        const float u2 = uniform01();
        const float mag = sqrtf(-2.0f * logf(u1));
        const float two_pi = 6.2831853f;
        spare_ = mag * sinf(two_pi * u2);
        have_spare_ = true;
        return mag * cosf(two_pi * u2);
    }

    Params   p_;
    float    t_block_;
    float    t_sensor_;
    uint32_t rng_state_;
    bool     have_spare_ = false;
    float    spare_      = 0.0f;
};

#endif
