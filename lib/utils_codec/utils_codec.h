/**
 * @file utils_codec.h
 * @brief Códecs de duty-cycle para RP2040 (C puro):
 *  - "Alfabeto" (opcional/legacy)
 *  - ASCII 7-bit robusto con 2 símbolos por carácter: MSB3 (8 bins) + LSB4 (16 bins)
 *
 * Banda de datos (coherente con utils_tx/utils_rx):
 *   [UAPWMC_DUTY_MIN_DATA .. UAPWMC_DUTY_MAX_DATA]
 */

#ifndef UTILS_CODEC_H_
#define UTILS_CODEC_H_

#include <stdint.h>
#include <stddef.h>

/* ---------------- Banda segura de datos ---------------- */
#ifndef UAPWMC_DUTY_MIN_DATA
#define UAPWMC_DUTY_MIN_DATA  0.15f   /* 15% */
#endif
#ifndef UAPWMC_DUTY_MAX_DATA
#define UAPWMC_DUTY_MAX_DATA  0.75f   /* 75% */
#endif

/* ==========================================================================
 *                     MODO "ALFABETO" (legacy / opcional)
 * ========================================================================== */
#ifndef CODEC_ALPHABET
#define CODEC_ALPHABET " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,!?-"
#endif
#define CODEC_NSYMS    ((int)(sizeof(CODEC_ALPHABET)-1))

/* idx [0..NSYMS-1] <-> duty */
float codec_index_to_duty(int idx);
int   codec_duty_to_index(float duty_meas);

/* char <-> duty (fallback a espacio si no existe) */
static inline float codec_char_to_duty(char ch){
    for (int i=0;i<CODEC_NSYMS;i++)
        if (CODEC_ALPHABET[i]==ch) return codec_index_to_duty(i);
    return codec_index_to_duty(0);
}
static inline char codec_duty_to_char(float duty_meas){
    int idx = codec_duty_to_index(duty_meas);
    if (idx < 0) idx = 0; if (idx >= CODEC_NSYMS) idx = CODEC_NSYMS-1;
    return CODEC_ALPHABET[idx];
}
/* checksum (solo si usas el modo alfabeto) */
static inline int codec_checksum_idx_from_text(const char* s, size_t n){
    int x=0;
    for(size_t i=0;i<n;i++){
        int f=-1;
        for (int k=0;k<CODEC_NSYMS;k++) if (CODEC_ALPHABET[k]==s[i]){ f=k; break; }
        if (f<0) f=0;
        x ^= f;
    }
    if (x >= CODEC_NSYMS) x %= CODEC_NSYMS;
    return x;
}

/* ==========================================================================
 *               MODO ASCII 7-bit (dos símbolos por carácter)
 *               MSB3 (8 bins: bits 6..4) + LSB4 (16 bins: bits 3..0)
 * ========================================================================== */
/* 3 bits (0..7) -> duty (centro del bin) e inversa */
float   codec8_tribit_to_duty(uint8_t v);     /* v en [0..7] */
uint8_t codec8_duty_to_tribit(float duty);

/* 4 bits (0..15) -> duty (centro del bin) e inversa */
float   codec16_nibble_to_duty(uint8_t n);    /* n en [0..15] */
uint8_t codec16_duty_to_nibble(float duty);

/* XOR sobre 7 bits (checksum de payload ASCII-7) */
static inline uint8_t codec7_checksum_xor(const uint8_t* p, size_t n){
    uint8_t c=0; for(size_t i=0;i<n;++i) c ^= (p[i] & 0x7F); return (uint8_t)(c & 0x7F);
}

#endif /* UTILS_CODEC_H_ */
