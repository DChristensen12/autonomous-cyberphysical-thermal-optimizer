// Standalone unit tests. No test framework needed, just g++.
//   cd test && make && ./test_units
//
// Covers the pure computational pieces: the matrix code and Cholesky, the GP,
// the PID controller, the thermal model, and the acquisition search. The big
// end-to-end check is the simulation in sim/, this is the fast per-unit net.

#include <cstdio>
#include <cmath>

#include "Matrix2D.h"
#include "GaussianProcess2D.h"
#include "Acquisition.h"
#include "PIDController.h"
#include "ThermalModel.h"

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond) do {                                        \
    ++g_checks;                                                 \
    if (!(cond)) { ++g_fails; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define CHECK_NEAR(a, b, tol) do {                              \
    ++g_checks;                                                 \
    if (fabsf((a) - (b)) > (tol)) { ++g_fails;                  \
        printf("FAIL %s:%d  |%g - %g| > %g\n", __FILE__, __LINE__, (double)(a), (double)(b), (double)(tol)); } \
} while (0)


void test_matrix_identity() {
    Matrix2D<4, 4> M(3, 3);
    M.identity();
    CHECK_NEAR(M(0, 0), 1.0f, 1e-6f);
    CHECK_NEAR(M(1, 1), 1.0f, 1e-6f);
    CHECK_NEAR(M(2, 2), 1.0f, 1e-6f);
    CHECK_NEAR(M(0, 1), 0.0f, 1e-6f);
}

void test_cholesky_known() {
    // A = [[4, 2], [2, 3]]  factors to  L = [[2, 0], [1, sqrt(2)]]
    Matrix2D<2, 2> A(2, 2);
    A(0, 0) = 4.0f; A(0, 1) = 2.0f;
    A(1, 0) = 2.0f; A(1, 1) = 3.0f;
    CHECK(linalg::cholesky_in_place<2>(A));
    CHECK_NEAR(A(0, 0), 2.0f, 1e-5f);
    CHECK_NEAR(A(1, 0), 1.0f, 1e-5f);
    CHECK_NEAR(A(1, 1), sqrtf(2.0f), 1e-5f);
}

void test_cholesky_solve_residual() {
    // Solve A x = b and confirm A x really is b.
    Matrix2D<3, 3> A(3, 3);
    float a[3][3] = {{4, 1, 1}, {1, 3, 0}, {1, 0, 2}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) A(i, j) = a[i][j];

    float b[3] = {6.0f, 5.0f, 4.0f};

    Matrix2D<3, 3> L = A;          // keep A intact for the residual check
    CHECK(linalg::cholesky_in_place<3>(L));

    float x[3], scratch[3];
    linalg::cholesky_solve<3>(L, b, x, scratch);

    for (int i = 0; i < 3; ++i) {
        float row = 0.0f;
        for (int j = 0; j < 3; ++j) row += a[i][j] * x[j];
        CHECK_NEAR(row, b[i], 1e-4f);
    }
}

void test_gp_reproduces_training() {
    GaussianProcess2D<8> gp(1.0f, 1.0f, 1.0f, 1e-6f);
    gp.add_sample(1.0f, 2.0f,  0.7f);
    gp.add_sample(3.0f, 4.0f, -0.4f);
    gp.add_sample(2.0f, 1.0f,  0.2f);
    CHECK(gp.update_posterior());

    float mu, sigma;
    gp.predict(1.0f, 2.0f, mu, sigma);
    CHECK_NEAR(mu, 0.7f, 2e-3f);
    CHECK(sigma < 1e-2f);              // near-zero uncertainty at a sample
}

void test_gp_uncertainty_grows_away() {
    GaussianProcess2D<8> gp(1.0f, 1.0f, 1.0f, 1e-6f);
    gp.add_sample(5.0f, 5.0f, 1.0f);
    CHECK(gp.update_posterior());

    float mu_near, s_near, mu_far, s_far;
    gp.predict(5.0f, 5.0f, mu_near, s_near);
    gp.predict(50.0f, 50.0f, mu_far, s_far);
    CHECK(s_far > s_near);             // less sure the farther we get
    CHECK(s_near < 0.05f);
}

void test_pid_signs() {
    PIDController pid(50.0f);
    pid.set_gains(2.0f, 0.5f);

    // First step: measurement below setpoint -> positive (heat) output, and
    // first tick has no derivative contribution.
    float u0 = pid.compute(40.0f, 0.01f);   // error = +10
    CHECK_NEAR(u0, 20.0f, 1e-4f);           // 2.0 * 10, derivative skipped

    // Measurement above setpoint -> negative (cool) output.
    PIDController pid2(50.0f);
    pid2.set_gains(2.0f, 0.0f);
    float u1 = pid2.compute(55.0f, 0.01f);  // error = -5
    CHECK_NEAR(u1, -10.0f, 1e-4f);
}

void test_pid_reset_kills_kick() {
    PIDController pid(50.0f);
    pid.set_gains(1.0f, 1.0f);
    pid.compute(40.0f, 0.01f);   // establishes prev_error
    pid.reset();
    // After reset the next step should behave like a first tick: no derivative
    // term, so output is purely proportional.
    float u = pid.compute(45.0f, 0.01f);   // error = +5
    CHECK_NEAR(u, 5.0f, 1e-4f);
}

void test_model_steady_state() {
    // Full heater, fan off. Block should climb toward ambient + Pmax/Gnat.
    ThermalModel::Params p;
    p.sensor_noise_c = 0.0f;
    p.sensor_quant_c = 0.0f;
    ThermalModel m(p);
    m.reset_to_ambient();

    const float expected_ceiling = p.ambient_c + p.heater_max_w / p.g_natural_w_per_k;
    for (int i = 0; i < 2000000; ++i) m.step(1.0f, 0.01f);  // long soak
    CHECK_NEAR(m.true_temperature(), expected_ceiling, 0.5f);
}

void test_model_sensor_lag() {
    // While heating from cold, the lagged sensor should read below the true
    // block temperature.
    ThermalModel::Params p;
    p.sensor_noise_c = 0.0f;
    p.sensor_quant_c = 0.0f;
    ThermalModel m(p);
    m.reset_to_ambient();
    for (int i = 0; i < 500; ++i) m.step(1.0f, 0.01f);   // 5 s of heating
    CHECK(m.measured_temperature() < m.true_temperature());
}

void test_acquisition_exploits() {
    // With kappa = 0 the acquisition is pure exploitation, so it should land
    // on the gain with the best modeled score.
    GaussianProcess2D<8> gp(1.0f, 1.0f, 1.0f, 1e-4f);
    gp.add_sample(5.0f, 2.5f, 1.0f);     // clear winner
    gp.add_sample(0.0f, 0.0f, -1.0f);
    gp.add_sample(10.0f, 5.0f, -1.0f);
    CHECK(gp.update_posterior());

    AcqResult r = argmax_ucb<8>(gp, 0.0f, 10.0f, 0.0f, 5.0f, 21, 0.0f);
    CHECK_NEAR(r.kp, 5.0f, 1.0f);
    CHECK_NEAR(r.kd, 2.5f, 0.6f);
}

int main() {
    test_matrix_identity();
    test_cholesky_known();
    test_cholesky_solve_residual();
    test_gp_reproduces_training();
    test_gp_uncertainty_grows_away();
    test_pid_signs();
    test_pid_reset_kills_kick();
    test_model_steady_state();
    test_model_sensor_lag();
    test_acquisition_exploits();

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}