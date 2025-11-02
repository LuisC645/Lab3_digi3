/**
 * @file utils_codec.c
 * @brief Implementaciones de códecs de duty: alfabeto (legacy) y ASCII-7 (3+4).
 */
#include "utils_codec.h"

/* ---------------- Helpers comunes ---------------- */
static inline float clamp01(float x){
    if (x < 0.f) return 0.f;
    if (x > 1.f) return 1.f;
    return x;
}

/* ==========================================================================
 *                       MODO "ALFABETO" (legacy)
 * ========================================================================== */
float codec_index_to_duty(int idx){
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);
    const int   N    = CODEC_NSYMS;

    if (idx < 0) idx = 0;
    if (idx >= N) idx = N-1;

    const float center = ((float)idx + 0.5f) / (float)N; /* (i+0.5)/N */
    return clamp01(Dmin + center * span);
}

int codec_duty_to_index(float duty_meas){
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);
    const int   N    = CODEC_NSYMS;

    float d = duty_meas;
    if (d < Dmin) d = Dmin;
    if (d > Dmax) d = Dmax;

    float rel = (d - Dmin) / span;      /* [0..1] */
    int   i   = (int)(rel * (float)N);  /* piso */
    float center = ((float)i + 0.5f) / (float)N;
    float delta  = rel - center;
    if (delta >=  0.5f/(float)N) i++;
    if (delta <= -0.5f/(float)N) i--;
    if (i < 0) i = 0; if (i >= N) i = N-1;
    return i;
}

/* ==========================================================================
 *                 MODO ASCII 7-bit: MSB3 (8 bins) + LSB4 (16 bins)
 * ========================================================================== */
float codec8_tribit_to_duty(uint8_t v){
    if (v > 7) v = 7;
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);
    /* centro del bin: (v + 0.5)/8 */
    const float center = ((float)v + 0.5f) / 8.0f;
    return clamp01(Dmin + center * span);
}

uint8_t codec8_duty_to_tribit(float duty){
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);

    float d = duty;
    if (d < Dmin) d = Dmin;
    if (d > Dmax) d = Dmax;

    float rel = (d - Dmin) / span;     /* [0..1] */
    int   i   = (int)(rel * 8.0f);     /* piso 0..7 */

    /* redondeo al centro del bin más cercano */
    float center = ((float)i + 0.5f) / 8.0f;
    float delta  = rel - center;
    if (delta >=  0.5f/8.0f) i++;
    if (delta <= -0.5f/8.0f) i--;

    if (i < 0) i = 0;
    if (i > 7) i = 7;
    return (uint8_t)i;
}

float codec16_nibble_to_duty(uint8_t n){
    if (n > 15) n = 15;
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);
    /* centro del bin: (n + 0.5)/16 */
    const float center = ((float)n + 0.5f) / 16.0f;
    return clamp01(Dmin + center * span);
}

uint8_t codec16_duty_to_nibble(float duty){
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);

    float d = duty;
    if (d < Dmin) d = Dmin;
    if (d > Dmax) d = Dmax;

    float rel = (d - Dmin) / span;     /* [0..1] */
    int   i   = (int)(rel * 16.0f);    /* piso 0..15 */

    /* redondeo al centro del bin más cercano */
    float center = ((float)i + 0.5f) / 16.0f;
    float delta  = rel - center;
    if (delta >=  0.5f/16.0f) i++;
    if (delta <= -0.5f/16.0f) i--;

    if (i < 0)  i = 0;
    if (i > 15) i = 15;
    return (uint8_t)i;
}
