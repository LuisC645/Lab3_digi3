/**
 * @file codif.h
 * @brief Codec ASCII 7-bit (MSB3+LSB4) usando duty PWM.
 */
#ifndef CODIF_H_
#define CODIF_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

/** Duty de control (fuera de datos). */
#define CODIF_DUTY_IDLE   0.05f
#define CODIF_DUTY_STOP   0.88f
#define CODIF_DUTY_START  0.97f
#define CODIF_TOLERANCIA  0.03f

/** Bandas de datos separadas. */
#define CODIF_DUTY_MIN_MSB3 0.15f
#define CODIF_DUTY_MAX_MSB3 0.45f
#define CODIF_DUTY_MIN_LSB4 0.55f
#define CODIF_DUTY_MAX_LSB4 0.85f
#define CODIF_BINS_MSB3 8
#define CODIF_BINS_LSB4 16

/** Compara duty con tolerancia. */
static inline bool codif_duty_match(float a, float b){ return fabsf(a-b) < CODIF_TOLERANCIA; }

/** MSB3: 0..7 → duty. */
float   codif_msb3_to_duty(uint8_t msb3);
/** LSB4: 0..15 → duty. */
float   codif_lsb4_to_duty(uint8_t lsb4);
/** duty → MSB3 (0..7). */
uint8_t codif_duty_to_msb3(float duty);
/** duty → LSB4 (0..15). */
uint8_t codif_duty_to_lsb4(float duty);

/** Byte 7-bit → (duty_msb, duty_lsb). */
void    codif_encode_byte7(uint8_t byte7, float* duty_msb, float* duty_lsb);
/** (duty_msb, duty_lsb) → byte 7-bit. */
uint8_t codif_decode_byte7(float duty_msb, float duty_lsb);

/** XOR 7-bit de un buffer. */
uint8_t codif_checksum7(const uint8_t* data, size_t n);

/** ASCII imprimible (32..126). */
static inline bool codif_is_printable(char ch){ return ch>=32 && ch<=126; }

#endif /* CODIF_H_ */
