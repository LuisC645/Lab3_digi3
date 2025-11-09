/**
 * @file utils_codec.h
 * @brief Codec ASCII 7-bit para RP2040 usando PWM duty-cycle.
 *        Cada carácter se transmite como 2 símbolos:
 *        - MSB3 (bits 6-4): 8 bins
 *        - LSB4 (bits 3-0): 16 bins
 */

#ifndef UTILS_CODEC_H_
#define UTILS_CODEC_H_

#include <stdint.h>
#include <stddef.h>

/* ===== Banda de datos (duty cycle) ===== */
#ifndef UAPWMC_DUTY_MIN_DATA
#define UAPWMC_DUTY_MIN_DATA  0.15f   /* 15% */
#endif
#ifndef UAPWMC_DUTY_MAX_DATA
#define UAPWMC_DUTY_MAX_DATA  0.75f   /* 75% */
#endif

/* ==========================================================================
 *                    ASCII 7-bit (2 símbolos por carácter)
 * ========================================================================== */

/**
 * @brief Convierte 3 bits (MSB) a duty cycle.
 * @param v Valor 0-7 (3 bits altos del ASCII: bits 6-4)
 * @return Duty cycle [DUTY_MIN_DATA .. DUTY_MAX_DATA]
 */
float codec_msb3_to_duty(uint8_t v);

/**
 * @brief Convierte duty cycle a 3 bits (MSB).
 * @param duty Duty cycle medido
 * @return Valor 0-7 (3 bits altos)
 */
uint8_t codec_duty_to_msb3(float duty);

/**
 * @brief Convierte 4 bits (LSB) a duty cycle.
 * @param n Valor 0-15 (4 bits bajos del ASCII: bits 3-0)
 * @return Duty cycle [DUTY_MIN_DATA .. DUTY_MAX_DATA]
 */
float codec_lsb4_to_duty(uint8_t n);

/**
 * @brief Convierte duty cycle a 4 bits (LSB).
 * @param duty Duty cycle medido
 * @return Valor 0-15 (4 bits bajos)
 */
uint8_t codec_duty_to_lsb4(float duty);

/**
 * @brief Codifica un carácter ASCII 7-bit completo (devuelve ambos símbolos).
 * @param ch Carácter ASCII (solo se usan 7 bits)
 * @param duty_msb Salida: duty cycle para MSB3
 * @param duty_lsb Salida: duty cycle para LSB4
 */
static inline void codec_char_to_duties(char ch, float* duty_msb, float* duty_lsb) {
    uint8_t byte = (uint8_t)ch & 0x7F;  // Asegurar 7 bits
    uint8_t msb3 = (byte >> 4) & 0x07;  // bits 6-4
    uint8_t lsb4 = byte & 0x0F;         // bits 3-0
    
    *duty_msb = codec_msb3_to_duty(msb3);
    *duty_lsb = codec_lsb4_to_duty(lsb4);
}

/**
 * @brief Decodifica un carácter ASCII desde dos duty cycles.
 * @param duty_msb Duty cycle del símbolo MSB3
 * @param duty_lsb Duty cycle del símbolo LSB4
 * @return Carácter ASCII reconstruido
 */
static inline char codec_duties_to_char(float duty_msb, float duty_lsb) {
    uint8_t msb3 = codec_duty_to_msb3(duty_msb);
    uint8_t lsb4 = codec_duty_to_lsb4(duty_lsb);
    uint8_t byte = ((msb3 & 0x07) << 4) | (lsb4 & 0x0F);
    return (char)byte;
}

/**
 * @brief Calcula checksum XOR de 7 bits sobre un buffer.
 * @param data Puntero al buffer de datos
 * @param len Longitud del buffer
 * @return Checksum de 7 bits (0-127)
 */
static inline uint8_t codec_checksum_xor7(const uint8_t* data, size_t len) {
    uint8_t chk = 0;
    for (size_t i = 0; i < len; i++) {
        chk ^= (data[i] & 0x7F);
    }
    return (uint8_t)(chk & 0x7F);
}

#endif /* UTILS_CODEC_H_ */
