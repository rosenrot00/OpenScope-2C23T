#include "../src/fft.c"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

static void test_sqrt(void) {
    const float values[] = {0.0f, 1.0e-44f, 1.0e-30f, 0.001f, 1.0f,
                            1000000.0f, 1.0e20f, FLT_MAX};
    for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        float expected = sqrtf(values[i]);
        assert(fabsf(local_sqrt(values[i]) - expected) <= expected * 0.000001f);
    }
    assert(local_sqrt(-1.0f) == 0.0f);
    assert(isinf(local_sqrt(INFINITY)));
    assert(isnan(local_sqrt(NAN)));
}

static void test_fft_amplitude(void) {
    const float amplitudes[] = {0.001f, 1.0f, 500.0f, 1000.0f, 1000000.0f};
    float input[FFT_SIZE];
    float magnitude[FFT_SIZE / 2];
    for (uint8_t window = 0; window < 4u; ++window) {
        float reference = 0.0f;
        for (unsigned a = 0; a < sizeof(amplitudes) / sizeof(amplitudes[0]); ++a) {
            for (unsigned i = 0; i < FFT_SIZE; ++i) {
                input[i] = amplitudes[a] * sinf(6.28318530718f * 8.0f * i / FFT_SIZE + 0.3f);
            }
            compute_fft(input, FFT_SIZE, magnitude, window);
            float normalized = magnitude[8] / amplitudes[a];
            if (!a) reference = normalized;
            assert(fabsf(normalized - reference) < reference * 0.00001f);
            float bin_magnitude;
            float phase;
            compute_fft_bin(input, FFT_SIZE, window, 8, &bin_magnitude, &phase);
            assert(fabsf(bin_magnitude - magnitude[8]) < magnitude[8] * 0.00001f);
            assert(isfinite(phase));
        }
    }
    for (unsigned i = 0; i < FFT_SIZE; ++i) {
        float phase = 6.28318530718f * 8.0f * i / FFT_SIZE;
        input[i] = 1000.0f * sinf(phase) + 500.0f * sinf(2.0f * phase);
    }
    compute_fft(input, FFT_SIZE, magnitude, 3);
    assert(fabsf(magnitude[16] / magnitude[8] - 0.5f) < 0.02f);
}

static void test_short_record(void) {
    // Fast timebases only provide 30/60/120 samples followed by zero padding.
    const uint16_t counts[] = {30, 60, 120};
    float input[FFT_SIZE];
    float magnitude[FFT_SIZE / 2];
    for (uint8_t window = 0; window < 4u; ++window) {
        for (unsigned i = 0; i < FFT_SIZE; ++i) {
            input[i] = 1000.0f * sinf(6.28318530718f * 32.0f * i / FFT_SIZE + 0.3f);
        }
        compute_fft(input, FFT_SIZE, magnitude, window);
        float reference = magnitude[32];
        for (unsigned c = 0; c < sizeof(counts) / sizeof(counts[0]); ++c) {
            for (unsigned i = counts[c]; i < FFT_SIZE; ++i) input[i] = 0.0f;
            compute_fft(input, counts[c], magnitude, window);
            assert(fabsf(magnitude[32] - reference) < reference * 0.02f);
            for (unsigned i = 1; i < FFT_SIZE / 2; ++i) assert(magnitude[i] <= magnitude[32]);
            float bin_magnitude;
            float phase;
            compute_fft_bin(input, counts[c], window, 32, &bin_magnitude, &phase);
            assert(fabsf(bin_magnitude - magnitude[32]) < magnitude[32] * 0.00001f);
            for (unsigned i = 0; i < FFT_SIZE; ++i) {
                input[i] = 1000.0f * sinf(6.28318530718f * 32.0f * i / FFT_SIZE + 0.3f);
            }
        }
    }
}

int main(void) {
    test_sqrt();
    test_fft_amplitude();
    test_short_record();
    puts("FFT amplitude regression tests passed");
    return 0;
}
