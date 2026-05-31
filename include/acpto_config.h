// Project-wide config. All the magic numbers live here so I'm not hunting
// through five files when I need to bump a pin.
#ifndef ACPTO_CONFIG_H
#define ACPTO_CONFIG_H
#include <stdint.h>
namespace pins {
    // Teensy 4.0 PWM pins: 0-9, 14, 15, 18, 19, 22-25, 28, 29, 33, 36, 37.
    // Steering clear of 0/1 (USB serial) and 13 (onboard LED).
constexpr int HEATER_PWM     = 2;
constexpr int FAN_PWM        = 3;
// A0 is an Arduino framework name, so it only exists on the Teensy build.
    // The laptop sim compiles with plain g++ and has no Arduino headers, so we
    // fall back to a plain number there. The sim never reads a real pin anyway,
    // it drives off the physics model, so the value is just a placeholder for it.
#ifdef ARDUINO
    constexpr int THERMISTOR_ADC = A0;   // divider midpoint, Teensy pin 14
#else
    constexpr int THERMISTOR_ADC = 14;   // placeholder for the native sim
#endif
constexpr int STATUS_LED     = 13;
}
namespace control {
    // 100 Hz PID. The aluminum block has dynamics measured in seconds so this
    // is more than enough.
constexpr uint32_t PID_PERIOD_US = 10000;
constexpr float SETPOINT_C = 45.0f;
    // Teensy default PWM is 8-bit. 12-bit gives the controller more room to
    // breathe near the setpoint.
constexpr int PWM_RESOLUTION_BITS = 12;
constexpr int PWM_MAX = (1 << PWM_RESOLUTION_BITS) - 1;
    // If we ever see this temperature something is very wrong. Kill the heater.
constexpr float SAFETY_MAX_C = 75.0f;
    // The fan is a 5V part but it hangs off the 9V motor rail through the
    // L293D. Never let its duty go past this or the fan cooks. 5 over 9 is
    // about 0.55, so 0.5 keeps a little headroom.
constexpr float FAN_MAX_DUTY = 0.5f;
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
// NTC thermistor and its divider. Our wiring is 3.3V into the thermistor on
// top, down to the A0 tap, then the fixed 10k from the tap down to ground.
// So when the block heats up the thermistor resistance falls, which pulls
// MORE of the 3.3V across the bottom resistor, so the tap voltage rises with
// temperature. The math below assumes exactly that arrangement, so if you
// ever flip the divider you have to revisit it.
//
// The Beta model is the simple two number fit for an NTC. Not as precise as
// full Steinhart Hart but more than good enough to hold a setpoint, and it
// needs just the one beta constant off the datasheet.
namespace thermistor {
constexpr float SERIES_RESISTOR = 10000.0f;  // the fixed 10k, on the bottom
constexpr float NOMINAL_RES     = 10000.0f;  // thermistor resistance at 25 C
constexpr float NOMINAL_TEMP_C  = 25.0f;     // temperature that rating is at
constexpr float BETA            = 3950.0f;   // beta, typical for these 10k beads
constexpr int   ADC_BITS        = 12;        // we read at 12 bit resolution
constexpr int   ADC_MAX         = (1 << ADC_BITS) - 1;
constexpr int   ADC_OVERSAMPLE  = 16;        // average this many reads per sample
}
#endif
