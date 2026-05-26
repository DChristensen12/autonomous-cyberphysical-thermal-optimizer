// Project-wide config. All the magic numbers live here so I'm not hunting
// through five files when I need to bump a pin.

#ifndef ACPTO_CONFIG_H
#define ACPTO_CONFIG_H

#include <stdint.h>

namespace pins {
    // Teensy 4.0 PWM pins: 0-9, 14, 15, 18, 19, 22-25, 28, 29, 33, 36, 37.
    // Steering clear of 0/1 (USB serial) and 13 (onboard LED).
    constexpr int HEATER_PWM   = 2;
    constexpr int FAN_PWM      = 3;
    constexpr int DS18B20_DATA = 4;   // don't forget the 4.7k pull-up to 3.3V
    constexpr int STATUS_LED   = 13;
}

namespace control {
    // 100 Hz PID. The aluminum block has dynamics measured in seconds so this
    // is more than enough. If the DS18B20 ever blocks for too long I'll have
    // to revisit.
    constexpr uint32_t PID_PERIOD_US = 10000;

    constexpr float SETPOINT_C = 45.0f;

    // Teensy default PWM is 8-bit. 12-bit gives the controller more room to
    // breathe near the setpoint.
    constexpr int PWM_RESOLUTION_BITS = 12;
    constexpr int PWM_MAX = (1 << PWM_RESOLUTION_BITS) - 1;

    // If we ever see this temperature something is very wrong. Kill the heater.
    constexpr float SAFETY_MAX_C = 75.0f;
}

namespace gp {
    // Kernel matrix is N x N so this caps memory at ~32*32*4 = 4 KB plus
    // change. Compute is the bigger worry, Cholesky is N^3. 32 felt like
    // a reasonable upper bound before I'd want to start thinking about
    // sparse approximations.
    constexpr int MAX_SAMPLES = 32;

    // RBF kernel hyperparameters. One length scale per gain axis, each set to
    // roughly a quarter of its axis range so nearby gains stay correlated but
    // the far corners don't. I expect to retune these once I see the real
    // performance landscape.
    constexpr float SIGMA_F         = 1.0f;
    constexpr float LENGTH_SCALE_KP = 4.0f;
    constexpr float LENGTH_SCALE_KD = 1.0f;
    constexpr float NOISE_VAR       = 1e-3f;  // also doubles as Cholesky jitter

    // UCB exploration weight. Crank this up if the optimizer gets stuck
    // exploiting a local optimum.
    constexpr float UCB_KAPPA = 2.0f;

    // Acquisition is a brute-force grid scan. 21x21 = 441 evals per step,
    // which is nothing on the M7.
    constexpr int ACQ_GRID_N = 21;
}

namespace gains {
    // The box the GP is allowed to search in. These bounds are a guess, 
    // I'll widen or narrow them after the first few trials.
    constexpr float KP_MIN = 0.5f;
    constexpr float KP_MAX = 20.0f;
    constexpr float KD_MIN = 0.0f;
    constexpr float KD_MAX = 5.0f;
}

#endif
