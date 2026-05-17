// Small fixed-size matrix for embedded use. The whole storage lives inside
// the object as a flat float array — no heap, no std::vector, no surprises
// when the MCU heap fragments at 3 AM.
//
// MAX_ROWS/MAX_COLS are the *capacity*. Logical size (rows_, cols_) can be
// smaller and grow at runtime, which is what the GP needs since the kernel
// matrix gains a row and column with every new sample.
//
// Style here borrows from pronenewbits/Arduino_Constrained_MPC_Library but
// I made it a template so I'm not paying for one fixed MAX_DIM project-wide.

#ifndef MATRIX2D_H
#define MATRIX2D_H

#include <stdint.h>
#include <math.h>

template <int MAX_ROWS, int MAX_COLS>
class Matrix2D {
public:
    Matrix2D() : rows_(0), cols_(0) {
        for (int i = 0; i < MAX_ROWS * MAX_COLS; ++i) data_[i] = 0.0f;
    }

    Matrix2D(int rows, int cols) : rows_(rows), cols_(cols) {
        for (int i = 0; i < MAX_ROWS * MAX_COLS; ++i) data_[i] = 0.0f;
    }

    inline int rows() const { return rows_; }
    inline int cols() const { return cols_; }
    inline int max_rows() const { return MAX_ROWS; }
    inline int max_cols() const { return MAX_COLS; }

    // No bounds checking — caller's problem. The hot paths can't afford it.
    inline float& operator()(int r, int c)       { return data_[r * MAX_COLS + c]; }
    inline float  operator()(int r, int c) const { return data_[r * MAX_COLS + c]; }

    // Raw pointer for the asm routines that want to skip the wrapper.
    inline float*       data()       { return data_; }
    inline const float* data() const { return data_; }

    void resize(int rows, int cols) { rows_ = rows; cols_ = cols; }

    void zero() {
        for (int i = 0; i < MAX_ROWS * MAX_COLS; ++i) data_[i] = 0.0f;
    }

    void identity() {
        zero();
        const int n = (rows_ < cols_) ? rows_ : cols_;
        for (int i = 0; i < n; ++i) (*this)(i, i) = 1.0f;
    }

    void add_to_diagonal(float v) {
        const int n = (rows_ < cols_) ? rows_ : cols_;
        for (int i = 0; i < n; ++i) (*this)(i, i) += v;
    }

    // y = A * x. Caller sizes the buffers; we trust them.
    void mul_vec(const float* x, float* y) const {
        for (int r = 0; r < rows_; ++r) {
            float acc = 0.0f;
            for (int c = 0; c < cols_; ++c) acc += (*this)(r, c) * x[c];
            y[r] = acc;
        }
    }

    // Pretty-printer for the laptop tests. Way too heavy for the MCU.
    #ifdef ACPTO_NATIVE_TEST
    void debug_print() const {
        for (int r = 0; r < rows_; ++r) {
            for (int c = 0; c < cols_; ++c) printf("% .4f ", (*this)(r, c));
            printf("\n");
        }
    }
    #endif

private:
    int rows_;
    int cols_;
    float data_[MAX_ROWS * MAX_COLS];
};


namespace linalg {

// In-place Cholesky: A = L L^T, lower triangle of A gets overwritten with L.
// Returns false if A isn't SPD — almost always means the kernel matrix went
// ill-conditioned, fix it by bumping noise_var.
//
// The innermost line (`s -= A(i, k) * A(j, k);`) is the whole point of the
// assembly work — it dominates runtime once N gets past ~10.
template <int N>
bool cholesky_in_place(Matrix2D<N, N>& A) {
    const int n = A.rows();
    for (int j = 0; j < n; ++j) {
        // diagonal
        float sum = A(j, j);
        for (int k = 0; k < j; ++k) {
            const float ljk = A(j, k);
            sum -= ljk * ljk;
        }
        if (sum <= 0.0f) return false;
        const float ljj = sqrtf(sum);
        A(j, j) = ljj;
        const float inv_ljj = 1.0f / ljj;

        // below-diagonal column — the hot loop
        for (int i = j + 1; i < n; ++i) {
            float s = A(i, j);
            for (int k = 0; k < j; ++k) {
                s -= A(i, k) * A(j, k);
            }
            A(i, j) = s * inv_ljj;
        }
    }
    return true;
}

// Solve L y = b. L is the lower triangle of L_full (output of cholesky_in_place).
template <int N>
void forward_substitute(const Matrix2D<N, N>& L_full, const float* b, float* y) {
    const int n = L_full.rows();
    for (int i = 0; i < n; ++i) {
        float s = b[i];
        for (int k = 0; k < i; ++k) s -= L_full(i, k) * y[k];
        y[i] = s / L_full(i, i);
    }
}

// Solve L^T x = y. Reads the same lower triangle, transposed implicitly.
template <int N>
void back_substitute(const Matrix2D<N, N>& L_full, const float* y, float* x) {
    const int n = L_full.rows();
    for (int i = n - 1; i >= 0; --i) {
        float s = y[i];
        for (int k = i + 1; k < n; ++k) s -= L_full(k, i) * x[k];
        x[i] = s / L_full(i, i);
    }
}

// Convenience: full A x = b solve given the factor. Pass a scratch buffer
// of size N from the stack.
template <int N>
void cholesky_solve(const Matrix2D<N, N>& L_full,
                    const float* b, float* x, float* y_scratch) {
    forward_substitute<N>(L_full, b, y_scratch);
    back_substitute<N>(L_full, y_scratch, x);
}

} // namespace linalg

#endif
