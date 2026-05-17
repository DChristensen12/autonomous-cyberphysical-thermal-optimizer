// Laptop-side tests for the math. Way faster than reflashing the Teensy
// every time I tweak the kernel.
//   pio test -e native_test

#include <unity.h>
#include <stdio.h>
#include <math.h>
#include "../lib/Matrix2D/Matrix2D.h"
#include "../lib/GaussianProcess2D/GaussianProcess2D.h"

void test_matrix_identity() {
    Matrix2D<4, 4> M(3, 3);
    M.identity();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, M(0, 0));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, M(1, 1));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, M(2, 2));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, M(0, 1));
}

void test_cholesky_on_known_matrix() {
    // A = [[4, 2], [2, 3]]  =>  L = [[2, 0], [1, sqrt(2)]]
    Matrix2D<2, 2> A(2, 2);
    A(0, 0) = 4.0f; A(0, 1) = 2.0f;
    A(1, 0) = 2.0f; A(1, 1) = 3.0f;

    TEST_ASSERT_TRUE(linalg::cholesky_in_place<2>(A));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, A(0, 0));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, A(1, 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, sqrtf(2.0f), A(1, 1));
}

void test_gp_predicts_training_point_exactly() {
    // With effectively no noise, the GP should return its training targets
    // when queried at the training locations. Sanity check.
    GaussianProcess2D<8> gp(1.0f, 0.5f, 1e-6f);
    gp.add_sample(1.0f, 2.0f,  0.7f);
    gp.add_sample(3.0f, 4.0f, -0.4f);
    gp.add_sample(2.0f, 1.0f,  0.2f);
    TEST_ASSERT_TRUE(gp.update_posterior());

    float mu, sigma;
    gp.predict(1.0f, 2.0f, mu, sigma);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.7f, mu);
    TEST_ASSERT_TRUE(sigma < 1e-2f);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_matrix_identity);
    RUN_TEST(test_cholesky_on_known_matrix);
    RUN_TEST(test_gp_predicts_training_point_exactly);
    return UNITY_END();
}
