#ifndef FFT_H
#define FFT_H

#include <stdint.h>

#define FFT_SIZE 256
#define FFT_BITS 8


// count: real samples at the start of input_real; the rest is zero padding.
void compute_fft(const float *input_real, uint16_t count, float *output_magnitude,
                 uint8_t window_type);
void compute_fft_bin(const float *input_real, uint16_t count, uint8_t window_type, uint16_t bin,
                     float *out_magnitude, float *out_phase_rad);

#endif // FFT_H
