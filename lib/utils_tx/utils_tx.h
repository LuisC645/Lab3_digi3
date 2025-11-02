/**
 * @file utils_tx.h
 * @brief TX por PWM (polling) + protocolo simple con checksum XOR.
 */
#ifndef UTILS_TX_H_
#define UTILS_TX_H_

#include <stdint.h>
#include <stddef.h>

/* ---------------- Config del protocolo (duty) ---------------- */
#define UAPWMC_DUTY_IDLE        0.10f   /* 10%  */
#define UAPWMC_DUTY_START       0.20f   /* 20%  */
#define UAPWMC_DUTY_STOP        0.80f   /* 80%  */

#define UAPWMC_DUTY_MIN_DATA    0.15f   /* 15%  límite inferior datos */
#define UAPWMC_DUTY_MAX_DATA    0.75f   /* 75%  límite superior datos */
#define UAPWMC_DUTY_SPAN_DATA   (UAPWMC_DUTY_MAX_DATA - UAPWMC_DUTY_MIN_DATA)

/* ---------------- PWM (TX) básico ---------------- */
void     tx_pwm_init(uint32_t gpio, uint32_t f_pwm_hz, uint32_t top);
void     tx_pwm_set_freq(uint32_t f_pwm_hz);
void     tx_pwm_set_duty01(float duty01);
void     tx_pwm_wait_cycles(uint8_t cycles);
uint32_t tx_pwm_get_gpio(void);

/* ---------------- Utilidades de datos ---------------- */
static inline float  tx_duty_from_byte(uint8_t b){
    return UAPWMC_DUTY_MIN_DATA + (UAPWMC_DUTY_SPAN_DATA * ((float)b / 255.0f));
}
static inline uint8_t tx_checksum_xor(const uint8_t* p, size_t n){
    uint8_t c = 0;
    for(size_t i=0;i<n;++i) c ^= p[i];
    return c;
}

/* ---------------- Envío de trama ----------------
   Formato: IDLE(2c) -> START(Sc) -> LEN(1B) -> DATA(NB) -> CHK(1B) -> STOP(Sc) -> IDLE(1c)
   Donde 'c' = ciclos PWM a sostener, 'Sc' = symbol_cycles. LEN ∈ [0..255].
*/
void tx_send_frame(const uint8_t* data, uint8_t len,
                   uint8_t symbol_cycles, uint8_t data_cycles);

#endif /* UTILS_TX_H_ */
