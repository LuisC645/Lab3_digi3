#include "utils_codec.h"

/* Helpers */
static inline float clamp01(float x){ if(x<0.f) return 0.f; if(x>1.f) return 1.f; return x; }

float codec_index_to_duty(int idx){
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);
    const int   N    = CODEC_NSYMS;
    if (idx < 0) idx = 0; if (idx >= N) idx = N-1;
    /* centro del bin i: (i+0.5)/N */
    float center = ((float)idx + 0.5f) / (float)N;
    float duty   = Dmin + center * span;
    return clamp01(duty);
}

int codec_duty_to_index(float duty_meas){
    const float Dmin = UAPWMC_DUTY_MIN_DATA;
    const float Dmax = UAPWMC_DUTY_MAX_DATA;
    const float span = (Dmax - Dmin);
    const int   N    = CODEC_NSYMS;

    /* clamp a la banda; si está fuera, lo traemos al borde más cercano */
    if (duty_meas < Dmin) duty_meas = Dmin;
    if (duty_meas > Dmax) duty_meas = Dmax;

    /* posición relativa [0..1] y bin nearest */
    float rel  = (duty_meas - Dmin) / span;
    int   idx  = (int)(rel * N);          /* piso */
    /* corrige al bin más cercano comparando medios */
    float center = ((float)idx + 0.5f) / (float)N;
    float rcenter = rel - center;
    if (rcenter >= 0.5f / (float)N) idx++;
    if (rcenter <= -0.5f / (float)N) idx--;
    if (idx < 0) idx = 0; if (idx >= N) idx = N-1;
    return idx;
}
