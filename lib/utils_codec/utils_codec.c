/**
 * @file utils_codec.c
 * @brief Implementación del codec ASCII 7-bit con PWM duty-cycle.
 */

#include "utils_codec.h"

/* ===== Helper ===== */
static inline float clamp01(float x) {
    if (x < 0.f) return 0.f;
    if (x > 1.f) return 1.f;
    return x;
}

/* ==========================================================================
 *                          MSB3 (8 bins: 0-7)
 * ========================================================================== */

float codec_msb3_to_duty(uint8_t v) {
    if (v > 7) v = 7;
    
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);
    
    // Centro del bin: (v + 0.5) / 8
    const float center = ((float)v + 0.5f) / 8.0f;
    return clamp01(Dmin + center * span);
}

uint8_t codec_duty_to_msb3(float duty) {
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);
    
    float d = duty;
    if (d < Dmin) d = Dmin;
    if (d > Dmax) d = Dmax;
    
    float rel = (d - Dmin) / span;     // [0..1]
    int i = (int)(rel * 8.0f);         // piso 0..7
    
    // Redondeo al centro del bin más cercano
    float center = ((float)i + 0.5f) / 8.0f;
    float delta = rel - center;
    
    if (delta >= 0.5f / 8.0f) i++;
    if (delta <= -0.5f / 8.0f) i--;
    
    if (i < 0) i = 0;
    if (i > 7) i = 7;
    
    return (uint8_t)i;
}

/* ==========================================================================
 *                          LSB4 (16 bins: 0-15)
 * ========================================================================== */

float codec_lsb4_to_duty(uint8_t n) {
    if (n > 15) n = 15;
    
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);
    
    // Centro del bin: (n + 0.5) / 16
    const float center = ((float)n + 0.5f) / 16.0f;
    return clamp01(Dmin + center * span);
}

uint8_t codec_duty_to_lsb4(float duty) {
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);
    
    float d = duty;
    if (d < Dmin) d = Dmin;
    if (d > Dmax) d = Dmax;
    
    float rel = (d - Dmin) / span;     // [0..1]
    int i = (int)(rel * 16.0f);        // piso 0..15
    
    // Redondeo al centro del bin más cercano
    float center = ((float)i + 0.5f) / 16.0f;
    float delta = rel - center;
    
    if (delta >= 0.5f / 16.0f) i++;
    if (delta <= -0.5f / 16.0f) i--;
    
    if (i < 0) i = 0;
    if (i > 15) i = 15;
    
    return (uint8_t)i;
}
