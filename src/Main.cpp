// Top-level orchestrator.
//
// Two things are happening at once:
//   - A 100 Hz PID loop running in an IntervalTimer ISR. It just minds the
//     thermal mass, oblivious to whatever the optimizer is up to.
//   - This file's loop(): pick the next (Kp, Kd) to try, push it into the
//     plant, let it run for a few seconds, score the result, feed it back
//     into the GP. Repeat.

#include <Arduino.h>
#include <IntervalTimer.h>
#include "acpto_config.h"
#include "Matrix2D.h"
#include "GaussianProcess2D.h"
#include "Acquisition.h"
#include "ThermalPlant.h"

static ThermalPlant                        g_plant;
static GaussianProcess2D<gp::MAX_SAMPLES>  g_gp(gp::SIGMA_F,
                                                gp::LENGTH_SCALE_KP,
                                                gp::LENGTH_SCALE_KD,
                                                gp::NOISE_VAR);
static IntervalTimer g_pid_timer;

// Trial scoring lives in the ISR side; main loop reads it under a brief
// timer stop to avoid tearing.
static volatile float    g_iae_accum   = 0.0f;
static volatile uint32_t g_iae_samples = 0;
static volatile uint32_t g_tick_count  = 0;

void pid_isr() {
    const float dt_s = control::PID_PERIOD_US * 1e-6f;
    g_plant.pid_tick(dt_s);
    g_iae_accum += fabsf(g_plant.last_error());
    g_iae_samples++;
    g_tick_count++;
}

// Acquisition lives in Acquisition.h now so the laptop sim and this firmware
// run the identical search. This wrapper just feeds it the gain box.
// Heads up: sim/sim_main.cpp scores a step-response with an overshoot
// penalty, while the run_trial below scores plain mean-abs-error at a fixed
// setpoint. Unify those before trusting hardware numbers against the sim.
AcqResult find_next_query() {
    return argmax_ucb<gp::MAX_SAMPLES>(
        g_gp,
        gains::KP_MIN, gains::KP_MAX,
        gains::KD_MIN, gains::KD_MAX,
        gp::ACQ_GRID_N, gp::UCB_KAPPA);
}

// Hold these gains for `window_ms`, then report mean absolute tracking error.
// Negated because the GP is set up to maximize.
float run_trial(float kp, float kd, uint32_t window_ms) {
    g_plant.set_gains(kp, kd);

    // Drop the timer for the duration of the reset so the volatile accumulators
    // don't get mid-update tears.
    g_pid_timer.end();
    g_iae_accum   = 0.0f;
    g_iae_samples = 0;
    g_pid_timer.begin(pid_isr, control::PID_PERIOD_US);

    delay(window_ms);

    g_pid_timer.end();
    const float    iae = g_iae_accum;
    const uint32_t n   = g_iae_samples;
    g_pid_timer.begin(pid_isr, control::PID_PERIOD_US);

    if (n == 0) return -1e6f;  // something went very wrong
    return -(iae / (float)n);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}  // give the USB host a moment

    Serial.println(F("# ACPTO booting"));
    g_plant.begin();

    // Seed the GP with a few corners + center so the posterior has something
    // to interpolate from. Without this, every prediction is just the prior
    // and UCB picks an arbitrary grid corner.
    struct Seed { float kp; float kd; };
    const Seed seeds[] = {
        { gains::KP_MIN, gains::KD_MIN },
        { gains::KP_MAX, gains::KD_MIN },
        { gains::KP_MIN, gains::KD_MAX },
        { gains::KP_MAX, gains::KD_MAX },
        { (gains::KP_MIN + gains::KP_MAX) * 0.5f,
          (gains::KD_MIN + gains::KD_MAX) * 0.5f },
    };

    g_pid_timer.begin(pid_isr, control::PID_PERIOD_US);

    Serial.println(F("# seeding GP"));
    Serial.println(F("# iter,kp,kd,score"));

    for (size_t i = 0; i < sizeof(seeds) / sizeof(seeds[0]); ++i) {
        const float score = run_trial(seeds[i].kp, seeds[i].kd, 5000);
        g_gp.add_sample(seeds[i].kp, seeds[i].kd, score);
        Serial.print(F("seed,"));
        Serial.print(seeds[i].kp, 3); Serial.print(',');
        Serial.print(seeds[i].kd, 3); Serial.print(',');
        Serial.println(score, 5);
    }

    if (!g_gp.update_posterior()) {
        Serial.println(F("# posterior update FAILED, try a bigger noise_var"));
    }
}

void loop() {
    static int iter = 0;
    const AcqResult next  = find_next_query();
    const float     score = run_trial(next.kp, next.kd, 5000);

    if (!g_gp.add_sample(next.kp, next.kd, score)) {
        Serial.println(F("# GP at capacity, stopping"));
        while (true) delay(1000);
    }
    if (!g_gp.update_posterior()) {
        Serial.println(F("# posterior update FAILED, try a bigger noise_var"));
    }

    Serial.print(iter++);          Serial.print(',');
    Serial.print(next.kp, 3);      Serial.print(',');
    Serial.print(next.kd, 3);      Serial.print(',');
    Serial.println(score, 5);

    digitalWriteFast(pins::STATUS_LED, !digitalReadFast(pins::STATUS_LED));
}
