// Hand-written Thumb-2 inner kernel for the Cholesky dot product. Target is
// the Cortex-M7 on the Teensy 4.0.
//
// What this replaces, on the C++ side:
//
//     for (int k = 0; k < j; ++k) {
//         s -= A(i, k) * A(j, k);
//     }
//
// I lifted just the dot product out into asm — the surrounding bookkeeping
// (the s -=, the division by inv_ljj, the outer i and j loops) stays in
// C++ where it's easy to read and not on the hot path.
//
// Signature (AAPCS):
//     extern "C" float cholesky_dot_asm(const float* row_i,
//                                       const float* row_j,
//                                       int len);
//     returns sum_{k=0}^{len-1} row_i[k] * row_j[k]
//
//     r0  -> row_i
//     r1  -> row_j
//     r2  -> len
//     s0  -> return value
//
// Plan (not yet implemented — see TODO below):
//   - unroll by 4
//   - VLDMIA to grab four floats per row in one instruction
//   - VFMA.F32 to accumulate without intermediate rounding
//   - scalar tail for the leftover < 4 elements
//
// TODO: right now this is the scalar reference version. It's correct but
// not faster than what GCC emits for the C loop. I'll swap in the unrolled
// VFMA version once I've profiled the .lst output and confirmed the C
// version is actually the bottleneck I think it is.

    .syntax unified
    .thumb
    .text
    .align 2

    .global cholesky_dot_asm
    .type   cholesky_dot_asm, %function

cholesky_dot_asm:
    vmov.f32  s0, #0.0           // accumulator

    cmp       r2, #0             // bail on zero/negative length
    ble       .Ldone

.Lscalar_loop:
    vldmia.32 r0!, {s1}          // s1 = *row_i++
    vldmia.32 r1!, {s2}          // s2 = *row_j++
    vfma.f32  s0, s1, s2         // s0 += s1 * s2  (fused, no intermediate round)
    subs      r2, r2, #1
    bne       .Lscalar_loop

.Ldone:
    bx        lr

    .size cholesky_dot_asm, .-cholesky_dot_asm
