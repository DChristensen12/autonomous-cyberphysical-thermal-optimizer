// The acquisition step: given the current GP, find the most promising
// (Kp, Kd) to try next. Pulled out into its own header so the simulation
// and the Teensy use the identical search.
//
// It's a brute-force scan of a grid. Could be a gradient ascent on the
// posterior instead, but a few hundred UCB evaluations is nothing on the
// M7 and the grid can't get stuck in a local max, so I'm keeping it simple
// until there's a reason not to.

#ifndef ACQUISITION_H
#define ACQUISITION_H

#include "GaussianProcess2D.h"

struct AcqResult {
    float kp;
    float kd;
    float ucb;
};

// Scan grid_n x grid_n points across the gain box and return the one with
// the highest UCB score.
template <int MAX_N>
AcqResult argmax_ucb(const GaussianProcess2D<MAX_N>& gp,
                     float kp_min, float kp_max,
                     float kd_min, float kd_max,
                     int grid_n, float kappa) {
    AcqResult best = { kp_min, kd_min, -1e30f };
    const float dkp = (kp_max - kp_min) / (grid_n - 1);
    const float dkd = (kd_max - kd_min) / (grid_n - 1);

    for (int i = 0; i < grid_n; ++i) {
        const float kp = kp_min + i * dkp;
        for (int j = 0; j < grid_n; ++j) {
            const float kd = kd_min + j * dkd;
            const float u  = gp.ucb(kp, kd, kappa);
            if (u > best.ucb) {
                best.kp  = kp;
                best.kd  = kd;
                best.ucb = u;
            }
        }
    }
    return best;
}

#endif
