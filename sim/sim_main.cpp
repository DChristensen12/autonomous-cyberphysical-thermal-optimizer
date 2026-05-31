// Runs the entire ACPTO optimizer on a laptop, no Teensy required. It wires
// the real GP, the real acquisition function, and the real PID controller to
// the simulated thermal block, then turns the optimizer loose to find good
// (Kp, Kd) gains.
//
// Because the GP, acquisition, and PID code here are the same files that get
// compiled for the Teensy, a result that looks good in this sim should hold
// up on hardware as long as the thermal model is roughly right.
//
// Build and run:
//   cd sim && make && ./acpto_sim
//
// Output is CSV on stdout: iter,kp,kd,score plus a summary at the end.
// Pipe it to a file and plot it if you want to watch the convergence.

#include <cstdio>
#include <cmath>

#include "acpto_config.h"
#include "GaussianProcess2D.h"
#include "Acquisition.h"
#include "PIDController.h"
#include "ThermalModel.h"

namespace {

// Trial protocol. We hold the block at a base temperature, let the controller
// settle into its holding power, then kick the setpoint up by a small step
// (a stand-in for a thermal shock) and measure how cleanly it recovers.
constexpr float TRIAL_BASE_C   = 45.0f;
constexpr float TRIAL_STEP_C   = 0.5f;
constexpr float TRIAL_WARMUP_S = 40.0f;
constexpr float TRIAL_WINDOW_S = 60.0f;
constexpr float SIM_DT_S       = control::PID_PERIOD_US * 1e-6f;  // match the 100 Hz loop

// How harshly to punish overshoot past the target. The project objective is
// tracking error plus settling behavior, and overshoot is the part of that
// where the derivative gain actually earns its keep. Without this term the
// score just rewards cranking Kp to the ceiling and zeroing Kd.
constexpr float OVERSHOOT_WEIGHT = 4.0f;

// Run one trial and return a score. Higher is better, so we negate the cost.
// Cost is mean absolute tracking error plus a penalty on peak overshoot.
// Error is measured against the TRUE block temperature, not the noisy sensor,
// because we care how well the block was actually regulated.
float run_trial(float kp, float kd) {
    ThermalModel  model;
    PIDController pid(TRIAL_BASE_C);

    // Pin the noise stream per trial so two evaluations of the same gains
    // give the same score. Bayesian opt assumes a deterministic objective
    // (or at least handles noise through the kernel), and a jittery score
    // would just confuse the GP here.
    model.seed_noise(0xACE1u);
    model.reset(TRIAL_BASE_C);
    pid.set_gains(kp, kd);
    pid.reset();

    // Warm-up: let the controller find its holding output at the base
    // temperature before we disturb it. Nothing is scored during this phase.
    int warmup_steps = (int)(TRIAL_WARMUP_S / SIM_DT_S);
    for (int i = 0; i < warmup_steps; ++i) {
        const float meas = model.measured_temperature();
        const float u    = pid.compute(meas, SIM_DT_S);
        model.step(u, SIM_DT_S);
    }

    // Apply the step and score the recovery.
    pid.set_setpoint(TRIAL_BASE_C + TRIAL_STEP_C);
    const float target = TRIAL_BASE_C + TRIAL_STEP_C;

    int window_steps = (int)(TRIAL_WINDOW_S / SIM_DT_S);
    float abs_error_sum = 0.0f;
    float peak_overshoot = 0.0f;
    for (int i = 0; i < window_steps; ++i) {
        const float meas = model.measured_temperature();
        const float u    = pid.compute(meas, SIM_DT_S);
        model.step(u, SIM_DT_S);

        const float true_t = model.true_temperature();
        abs_error_sum += fabsf(target - true_t);

        const float over = true_t - target;
        if (over > peak_overshoot) peak_overshoot = over;
    }

    const float mean_abs_error = abs_error_sum / (float)window_steps;
    const float cost = mean_abs_error + OVERSHOOT_WEIGHT * peak_overshoot;
    return -cost;
}

} // namespace

int main() {
    GaussianProcess2D<gp::MAX_SAMPLES> gp(gp::SIGMA_F,
                                          gp::LENGTH_SCALE_KP, gp::LENGTH_SCALE_KD,
                                          gp::NOISE_VAR);

    printf("# ACPTO simulation\n");
    printf("# trial: settle at %.1fC, step +%.1fC, score = -mean|err| over %.0fs\n",
           TRIAL_BASE_C, TRIAL_STEP_C, TRIAL_WINDOW_S);
    printf("phase,iter,kp,kd,score\n");

    // Seed the GP with the corners and center of the gain box so it has
    // something to interpolate from before UCB takes over.
    struct Seed { float kp; float kd; };
    const Seed seeds[] = {
        { gains::KP_MIN, gains::KD_MIN },
        { gains::KP_MAX, gains::KD_MIN },
        { gains::KP_MIN, gains::KD_MAX },
        { gains::KP_MAX, gains::KD_MAX },
        { 0.5f * (gains::KP_MIN + gains::KP_MAX),
          0.5f * (gains::KD_MIN + gains::KD_MAX) },
    };

    for (size_t i = 0; i < sizeof(seeds) / sizeof(seeds[0]); ++i) {
        const float score = run_trial(seeds[i].kp, seeds[i].kd);
        gp.add_sample(seeds[i].kp, seeds[i].kd, score);
        printf("seed,%zu,%.4f,%.4f,%.6f\n", i, seeds[i].kp, seeds[i].kd, score);
    }
    if (!gp.update_posterior()) {
        printf("# posterior update failed after seeding, bump noise_var\n");
        return 1;
    }

    // Active learning loop. Keep going until the GP is full.
    float best_score = -1e30f;
    float best_kp = 0.0f, best_kd = 0.0f;

    int iter = 0;
    while (gp.num_samples() < gp::MAX_SAMPLES) {
        const AcqResult next = argmax_ucb<gp::MAX_SAMPLES>(
            gp,
            gains::KP_MIN, gains::KP_MAX,
            gains::KD_MIN, gains::KD_MAX,
            gp::ACQ_GRID_N, gp::UCB_KAPPA);

        const float score = run_trial(next.kp, next.kd);
        gp.add_sample(next.kp, next.kd, score);
        if (!gp.update_posterior()) {
            printf("# posterior update failed at iter %d, bump noise_var\n", iter);
            break;
        }

        if (score > best_score) {
            best_score = score;
            best_kp = next.kp;
            best_kd = next.kd;
        }

        printf("opt,%d,%.4f,%.4f,%.6f\n", iter, next.kp, next.kd, score);
        ++iter;
    }

    printf("# best observed: kp=%.4f kd=%.4f score=%.6f (mean abs error %.4f C)\n",
           best_kp, best_kd, best_score, -best_score);

    // The best *observed* sample is noisy. The gains you'd actually deploy are
    // the ones that maximize the GP's posterior mean, since that's the model's
    // smoothed best guess with the exploration bonus stripped out. Scan the
    // grid one more time on mu alone.
    float rec_kp = gains::KP_MIN, rec_kd = gains::KD_MIN, rec_mu = -1e30f;
    const float dkp = (gains::KP_MAX - gains::KP_MIN) / (gp::ACQ_GRID_N - 1);
    const float dkd = (gains::KD_MAX - gains::KD_MIN) / (gp::ACQ_GRID_N - 1);
    for (int i = 0; i < gp::ACQ_GRID_N; ++i) {
        const float kp = gains::KP_MIN + i * dkp;
        for (int j = 0; j < gp::ACQ_GRID_N; ++j) {
            const float kd = gains::KD_MIN + j * dkd;
            float mu, sigma;
            gp.predict(kp, kd, mu, sigma);
            if (mu > rec_mu) { rec_mu = mu; rec_kp = kp; rec_kd = kd; }
        }
    }
    printf("# recommended (argmax posterior mean): kp=%.4f kd=%.4f\n", rec_kp, rec_kd);
    return 0;
}
