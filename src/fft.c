#include "fft.h"

typedef struct {
    float real;
    float imag;
} complex_t;

static complex_t fft_work[FFT_SIZE];

// taylor series trig approx
static float local_sin(float x) {
    // Normalize angle to (-PI, PI]
    while (x > 3.14159265f)  x -= 6.2831853f;
    while (x <= -3.14159265f) x += 6.2831853f;
    
    // Taylor series expansion for sin(x) = x - x^3/3! + x^5/5! - x^7/7!
    float x2 = x * x;
    float term = x;
    float sum = x;
    
    term = (term * x2) / 6.0f;    // x^3 / 3!
    sum -= term;
    term = (term * x2) / 20.0f;   // x^5 / 5!
    sum += term;
    term = (term * x2) / 42.0f;   // x^7 / 7!
    sum -= term;
    
    return sum;
}

static float local_cos(float x) {
    // cos(x) = sin(x + PI/2)
    return local_sin(x + 1.57079632f);
}

static float local_atan2(float y, float x) {
    if (x == 0.0f) {
        return y > 0.0f ? 1.57079632f : -1.57079632f;
    }

    float ax = x < 0.0f ? -x : x;
    float ay = y < 0.0f ? -y : y;
    float a = (ax < ay) ? ax / ay : ay / ax;
    float s = a * a;
    float r = (((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a);
    if (ay > ax) {
        r = 1.57079632f - r;
    }
    if (x < 0.0f) {
        r = 3.14159265f - r;
    }
    if (y < 0.0f) {
        r = -r;
    }
    return r;
}

static float local_sqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    union { float value; uint32_t bits; } seed = { .value = x };
    float scale = 1.0f;

    if (seed.bits >= 0x7F800000u) return x;
    if (seed.bits < 0x00800000u) {
        x *= 16777216.0f;
        seed.value = x;
        scale = 0.000244140625f;
    }
    // Estimate from the exponent so four iterations also converge for large FFT bins.
    seed.bits = (seed.bits >> 1) + 0x1FC00000u;
    float guess = seed.value;
    for (int i = 0; i < 4; i++) {
        guess = 0.5f * (guess + x / guess);
    }
    return guess * scale;
}

// bit-reversal
static uint8_t fft_bit_reverse(uint8_t index) {
    uint8_t rev = 0;
    for (uint8_t i = 0; i < FFT_BITS; i++) {
        if (index & (1 << i)) {
            rev |= (1u << (FFT_BITS - 1u - i));
        }
    }
    return rev;
}

// Sum of the symmetric window over count points.
static float fft_window_gain(uint8_t window_type, uint16_t count) {
    if (window_type == 0) {
        return 0.5f * (float)(count - 1u);
    }
    if (window_type == 1) {
        return 0.54f * (float)count - 0.46f;
    }
    if (window_type == 2) {
        return 0.42f * (float)count - 0.42f;
    }
    return (float)count;
}

static void fft_compute_complex(const float *input_real, uint16_t count, uint8_t window_type,
                                complex_t *data) {
    float scale;

    if (count < 2u || count > FFT_SIZE) {
        count = FFT_SIZE;
    }
    // Window only the real samples and keep amplitudes comparable to a full record.
    scale = fft_window_gain(window_type, FFT_SIZE) / fft_window_gain(window_type, count);

    for (int i = 0; i < FFT_SIZE; i++) {
        uint8_t rev_idx = fft_bit_reverse((uint8_t)i);
        float w = 0.0f;

        if (i < count) {
            float angle = (2.0f * 3.14159265f * (float)i) / (float)(count - 1u);

            w = 1.0f;
            if (window_type == 0) {
                w = 0.5f * (1.0f - local_cos(angle));
            } else if (window_type == 1) {
                w = 0.54f - 0.46f * local_cos(angle);
            } else if (window_type == 2) {
                w = 0.42f - 0.5f * local_cos(angle) + 0.08f * local_cos(2.0f * angle);
            }
            w *= scale;
        }

        data[rev_idx].real = input_real[i] * w;
        data[rev_idx].imag = 0.0f;
    }

    for (int size = 2; size <= FFT_SIZE; size <<= 1) {
        int half_size = size >> 1;
        float tab_step = -2.0f * 3.14159265f / (float)size;

        for (int i = 0; i < FFT_SIZE; i += size) {
            for (int j = 0; j < half_size; j++) {
                float angle = (float)j * tab_step;
                float twiddle_real = local_cos(angle);
                float twiddle_imag = local_sin(angle);

                int t_idx = i + j + half_size;
                int u_idx = i + j;

                float t_real = data[t_idx].real * twiddle_real - data[t_idx].imag * twiddle_imag;
                float t_imag = data[t_idx].real * twiddle_imag + data[t_idx].imag * twiddle_real;

                data[t_idx].real = data[u_idx].real - t_real;
                data[t_idx].imag = data[u_idx].imag - t_imag;
                data[u_idx].real += t_real;
                data[u_idx].imag += t_imag;
            }
        }
    }
}

void compute_fft(const float *input_real, uint16_t count, float *output_magnitude,
                 uint8_t window_type) {
    fft_compute_complex(input_real, count, window_type, fft_work);

    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float r = fft_work[i].real;
        float im = fft_work[i].imag;
        output_magnitude[i] = local_sqrt(r * r + im * im);
    }
}

void compute_fft_bin(const float *input_real, uint16_t count, uint8_t window_type, uint16_t bin,
                     float *out_magnitude, float *out_phase_rad) {
    uint16_t idx;

    if (out_magnitude) {
        *out_magnitude = 0.0f;
    }
    if (out_phase_rad) {
        *out_phase_rad = 0.0f;
    }
    if (!out_magnitude && !out_phase_rad) {
        return;
    }

    fft_compute_complex(input_real, count, window_type, fft_work);

    idx = bin;
    if (idx >= (uint16_t)(FFT_SIZE / 2)) {
        idx = (uint16_t)((FFT_SIZE / 2) - 1u);
    }
    if (idx < 1u) {
        idx = 1u;
    }

    {
        float r = fft_work[idx].real;
        float im = fft_work[idx].imag;
        if (out_magnitude) {
            *out_magnitude = local_sqrt(r * r + im * im);
        }
        if (out_phase_rad) {
            *out_phase_rad = local_atan2(im, r);
        }
    }
}

// Manually satisfy the bare-metal __aeabi_memclr4 symbol requirement
__attribute__((used)) 
void __aeabi_memclr4(void *dest, unsigned int bytes) {
    uint32_t *d = (uint32_t *)dest;
    unsigned int words = bytes >> 2; // Divide by 4 to get word count
    
    while (words--) {
        *d++ = 0;
    }
}
