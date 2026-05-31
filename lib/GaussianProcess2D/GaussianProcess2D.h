// 2D Gaussian Process with an RBF kernel and a UCB helper. Started from the
// API in PeterBakarac/BayesianOptimization (1D, scalar input) and pushed it
// up to 2D so it can chew on (Kp, Kd) jointly.
//
// Two notable departures from the original:
//   1. I switched the linear solve from Gauss-Jordan to Cholesky. The kernel
//      matrix is symmetric positive-definite by construction, so Cholesky is
//      both faster and better conditioned. It's also a much nicer target for
//      hand-written assembly later.
//   2. The class owns all its storage, no dynamic allocation anywhere.
//      MAX_N is a template parameter.

#ifndef GAUSSIAN_PROCESS_2D_H
#define GAUSSIAN_PROCESS_2D_H

#include <math.h>
#include "Matrix2D.h"

template <int MAX_N>
class GaussianProcess2D {
public:
    // Two length scales, one per input axis. Kp and Kd live on very different
    // scales (Kp ranges over ~20, Kd over ~5), so a single shared length scale
    // would make the kernel treat almost every sample as uncorrelated with its
    // neighbors. Sizing each length scale to its own axis fixes that.
    GaussianProcess2D(float sigma_f, float length_scale_kp, float length_scale_kd,
                      float noise_var)
        : sigma_f_sq_(sigma_f * sigma_f),
          inv_two_l_kp_sq_(1.0f / (2.0f * length_scale_kp * length_scale_kp)),
          inv_two_l_kd_sq_(1.0f / (2.0f * length_scale_kd * length_scale_kd)),
          noise_var_(noise_var),
          n_(0),
          posterior_stale_(true)
    {}

    // Drop in one (Kp, Kd, score) observation. Returns false if we're full.
    bool add_sample(float kp, float kd, float y) {
        if (n_ >= MAX_N) return false;
        X_(n_, 0) = kp;
        X_(n_, 1) = kd;
        y_[n_] = y;
        ++n_;
        posterior_stale_ = true;
        return true;
    }

    int num_samples() const { return n_; }

    // Refactor the kernel matrix and resolve for alpha. Call this after any
    // add_sample() and before any predict(). If it returns false the matrix
    // wasn't SPD, usually the fix is more noise jitter.
    bool update_posterior() {
        L_.resize(n_, n_);
        for (int i = 0; i < n_; ++i) {
            for (int j = 0; j <= i; ++j) {
                const float k = rbf_kernel(X_(i, 0), X_(i, 1),
                                           X_(j, 0), X_(j, 1));
                L_(i, j) = k;
                L_(j, i) = k;  // mirror it so the matrix is well-defined,
                               // even though Cholesky only touches the lower half
            }
            L_(i, i) += noise_var_;
        }

        if (!linalg::cholesky_in_place<MAX_N>(L_)) return false;

        float y_scratch[MAX_N];
        linalg::cholesky_solve<MAX_N>(L_, y_, alpha_, y_scratch);
        posterior_stale_ = false;
        return true;
    }

    // Posterior mean and stddev at a query point.
    void predict(float kp_query, float kd_query, float& mu, float& sigma) const {
        if (n_ == 0 || posterior_stale_) {
            // No data yet, or someone forgot to call update_posterior().
            // Fall back to the prior.
            mu = 0.0f;
            sigma = sqrtf(sigma_f_sq_);
            return;
        }

        // k_*, kernel vector between the query and every training point.
        float k_star[MAX_N] = {0.0f};
        for (int i = 0; i < n_; ++i) {
            k_star[i] = rbf_kernel(kp_query, kd_query, X_(i, 0), X_(i, 1));
        }

        // mu = k_*^T alpha
        float mu_acc = 0.0f;
        for (int i = 0; i < n_; ++i) mu_acc += k_star[i] * alpha_[i];
        mu = mu_acc;

        // var = k(x*, x*) - v^T v, with v = L^{-1} k_*
        float v[MAX_N];
        linalg::forward_substitute<MAX_N>(L_, k_star, v);
        float var = sigma_f_sq_;  // k(x*, x*) for RBF when x* == x*
        for (int i = 0; i < n_; ++i) var -= v[i] * v[i];
        if (var < 0.0f) var = 0.0f;  // FP roundoff can dip it slightly negative
        sigma = sqrtf(var);
    }

    // UCB acquisition. The outer loop maximizes this over a grid.
    inline float ucb(float kp, float kd, float kappa) const {
        float mu, sigma;
        predict(kp, kd, mu, sigma);
        return mu + kappa * sigma;
    }

private:
    inline float rbf_kernel(float kp1, float kd1, float kp2, float kd2) const {
        const float d_kp = kp1 - kp2;
        const float d_kd = kd1 - kd2;
        const float exponent = d_kp * d_kp * inv_two_l_kp_sq_
                             + d_kd * d_kd * inv_two_l_kd_sq_;
        return sigma_f_sq_ * expf(-exponent);
    }

    const float sigma_f_sq_;
    const float inv_two_l_kp_sq_;
    const float inv_two_l_kd_sq_;
    const float noise_var_;

    Matrix2D<MAX_N, 2>    X_;       // training inputs, one row per sample
    float                  y_[MAX_N];
    int                    n_;

    Matrix2D<MAX_N, MAX_N> L_;       // Cholesky factor of (K + noise * I)
    float                  alpha_[MAX_N];
    bool                   posterior_stale_;
};

#endif
