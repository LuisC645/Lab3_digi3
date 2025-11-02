/**
 * @file utils_codec.h
 * @brief Mapeo carácter <-> duty usando un alfabeto fijo y bandas seguras.
 *
 * Cada carácter del alfabeto se asigna a un duty uniforme en [D_MIN..D_MAX].
 * RX decodifica por "binning" (nearest) y aplica clamp dentro de la banda.
 */
#ifndef UTILS_CODEC_H_
#define UTILS_CODEC_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Banda segura para datos: debe ser consistente con TX/RX */
#ifndef UAPWMC_DUTY_MIN_DATA
#define UAPWMC_DUTY_MIN_DATA   0.15f  /* 15% */
#endif
#ifndef UAPWMC_DUTY_MAX_DATA
#define UAPWMC_DUTY_MAX_DATA   0.75f  /* 75% */
#endif

/* Alfabeto permitido (puedes ampliarlo si necesitas más símbolos) */
#define CODEC_ALPHABET  " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,!?-"
#define CODEC_NSYMS     ((int)(sizeof(CODEC_ALPHABET)-1))  /* sin el \0 */

/* Mapea índice [0..NSYMS-1] a duty centrado en su bin */
float codec_index_to_duty(int idx);

/* Dado un duty medido, devuelve índice [0..NSYMS-1] (nearest bin) */
int   codec_duty_to_index(float duty_meas);

/* Carácter <-> duty (con fallback a ' ' si no existe) */
static inline float codec_char_to_duty(char ch){
    /* busca idx del char (lineal; N pequeño) */
    for (int i=0;i<CODEC_NSYMS;i++) if (CODEC_ALPHABET[i]==ch) return codec_index_to_duty(i);
    /* si no está, mapea a espacio */
    return codec_index_to_duty(0);
}
static inline char codec_duty_to_char(float duty_meas){
    int idx = codec_duty_to_index(duty_meas);
    if (idx < 0) idx = 0; if (idx >= CODEC_NSYMS) idx = CODEC_NSYMS-1;
    return CODEC_ALPHABET[idx];
}

/* Checksum: XOR de índices del alfabeto (resultado mod NSYMS, ya que XOR cabe) */
static inline int codec_checksum_idx_from_text(const char* s, size_t n){
    int x = 0;
    for (size_t i=0;i<n;i++){
        /* convierte char a índice y acumula por XOR */
        char ch = s[i];
        int found = -1;
        for (int k=0;k<CODEC_NSYMS;k++) if (CODEC_ALPHABET[k]==ch){ found=k; break; }
        if (found<0) found = 0; /* espacio si no está */
        x ^= found;
    }
    /* El XOR ya está en [0..(2^ceil(log2(NSYMS))-1)], lo bin-eamos a índice por clamp */
    if (x >= CODEC_NSYMS) x %= CODEC_NSYMS;
    return x;
}

#ifdef __cplusplus
}
#endif
#endif /* UTILS_CODEC_H_ */
