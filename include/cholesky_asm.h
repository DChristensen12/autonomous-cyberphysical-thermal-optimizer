// Bridge to the hand-written assembly in asm/cholesky_inner_asm.S.
// Build with -DACPTO_USE_ASM to actually swap it in — otherwise the pure
// C++ version runs and this header isn't included. Keeps the comparison
// honest when I want to benchmark.

#ifndef CHOLESKY_ASM_H
#define CHOLESKY_ASM_H

#ifdef __cplusplus
extern "C" {
#endif

// Dot product of two float arrays. Just the inner kernel — the surrounding
// Cholesky bookkeeping stays in C++ where it's readable.
float cholesky_dot_asm(const float* row_i, const float* row_j, int len);

#ifdef __cplusplus
}
#endif

#endif
