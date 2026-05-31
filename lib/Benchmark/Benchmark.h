#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <Arduino.h>

// Hand this a chunk of work and it tells you how long that work takes. You
// pass the work in as a callable, a lambda is easiest, and it runs that work
// over and over, then prints the timing breakdown over Serial.
//
// The reason it runs many times instead of once: a single call on a 600 MHz
// chip can finish faster than the timer can even measure, so one reading is
// mostly noise. Averaging over a few thousand runs gives a number you can
// actually trust. The min and max matter just as much as the average here,
// because a control loop cares about its worst case far more than its typical
// case. One slow outlier that blows past your loop period is a missed
// deadline, and the mean would hide it.
//
// Example:
//   benchmark("posterior update", 2000, [&]() { gp.update_posterior(); });

template <typename Fn>
void benchmark(const char* label, uint32_t iterations, Fn work) {
    if (iterations == 0) iterations = 1;

    // Run it once and throw the result away. The first call pays for things
    // like cache warming that we do not want charged to the real measurement.
    work();

    uint32_t total_us = 0;
    uint32_t min_us = 0xFFFFFFFF;
    uint32_t max_us = 0;

    for (uint32_t i = 0; i < iterations; ++i) {
        const uint32_t start = micros();
        work();
        const uint32_t elapsed = micros() - start;

        total_us += elapsed;
        if (elapsed < min_us) min_us = elapsed;
        if (elapsed > max_us) max_us = elapsed;
    }

    // Keep the mean as a float. If a call averages half a microsecond, an
    // integer mean would just round to zero and tell you nothing.
    const float mean_us = (float)total_us / (float)iterations;

    Serial.println();
    Serial.print("benchmark: ");
    Serial.println(label);
    Serial.print("  iterations:   ");
    Serial.println(iterations);
    Serial.print("  total time:   ");
    Serial.print(total_us);
    Serial.println(" us");
    Serial.print("  mean / call:  ");
    Serial.print(mean_us, 3);
    Serial.println(" us");
    Serial.print("  fastest call: ");
    Serial.print(min_us);
    Serial.println(" us");
    Serial.print("  slowest call: ");
    Serial.print(max_us);
    Serial.println(" us");

    // Rough sense of how many of these you could do per second if the chip
    // did nothing else. Good for checking against your loop rate.
    if (mean_us > 0.0f) {
        const float per_sec = 1000000.0f / mean_us;
        Serial.print("  throughput:   ");
        Serial.print(per_sec, 0);
        Serial.println(" calls/sec");
    }
}

#endif

// To actually use it, drop calls like these into setup() in src/main.cpp, after the GP has been seeded with a few samples (so the timed work is realistic). The volatile trick stops the compiler from deleting a call whose result you do not use, which would otherwise give you a fake near-zero time:
// benchmark("gp posterior update", 2000, [&]() {
//    g_gp.update_posterior();
//});

//benchmark("ucb single eval", 20000, [&]() {
//    volatile float u = g_gp.ucb(8.0f, 2.0f, gp::UCB_KAPPA);
//    (void)u;
//});
