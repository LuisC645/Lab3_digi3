/**
 * @file utils_rx.h
 * @brief RX por polling: medir duty/periodo y decodificar trama (checksum XOR).
 */
#ifndef UTILS_RX_H_
#define UTILS_RX_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Reusa constantes de TX para coherencia de protocolo (copiadas aquí) */
#define UAPWMC_DUTY_IDLE        0.10f
#define UAPWMC_DUTY_START       0.20f
#define UAPWMC_DUTY_STOP        0.80f
#define UAPWMC_DUTY_MIN_DATA    0.15f
#define UAPWMC_DUTY_MAX_DATA    0.75f
#define UAPWMC_DUTY_SPAN_DATA   (UAPWMC_DUTY_MAX_DATA - UAPWMC_DUTY_MIN_DATA)

/* Tolerancias para reconocer símbolos de control */
#define UAPWMC_EPS_CTRL         0.03f   /* ±3%  alrededor de START/STOP */
#define UAPWMC_MIN_VALID_DUTY   0.02f   /* para descartar basura */

/* GPIO input */
void rx_in_init(uint32_t gpio, int pull_up, int pull_down);

/* Mediciones básicas (útiles si quieres debug) */
bool  rx_measure_one_cycle(uint32_t pin, uint32_t expected_period_us,
                           uint32_t* period_us, uint32_t* high_us);
float rx_measure_duty_avg(uint32_t pin, uint32_t expected_period_us, int cycles);

/* Decodificación byte<->duty igual que TX */
static inline uint8_t rx_byte_from_duty(float duty){
    if (duty < UAPWMC_DUTY_MIN_DATA) duty = UAPWMC_DUTY_MIN_DATA;
    if (duty > UAPWMC_DUTY_MAX_DATA) duty = UAPWMC_DUTY_MAX_DATA;
    float norm = (duty - UAPWMC_DUTY_MIN_DATA) / UAPWMC_DUTY_SPAN_DATA;
    int v = (int)(norm * 255.0f + 0.5f);
    if (v<0) v=0; if (v>255) v=255;
    return (uint8_t)v;
}
static inline uint8_t rx_checksum_xor(const uint8_t* p, size_t n){
    uint8_t c=0; for(size_t i=0;i<n;++i) c^=p[i]; return c;
}

/**
 * @brief Recibe una trama completa con validación (bloqueante simple).
 *
 * @param pin               GPIO de entrada.
 * @param expected_period_us Periodo esperado (ej. 1000 us para 1 kHz).
 * @param maxlen            Capacidad del buffer 'out'.
 * @param out               Buffer de salida del payload.
 * @param out_len           Longitud efectiva recibida.
 * @param timeout_ms        Timeout máximo para encontrar y recibir (ms).
 * @return true si OK (checksum válido y STOP detectado).
 *
 * Formato esperado: IDLE -> START -> LEN -> DATA... -> CHK -> STOP
 */
bool rx_recv_frame(uint32_t pin, uint32_t expected_period_us,
                   uint8_t* out, uint8_t maxlen, uint8_t* out_len,
                   uint32_t timeout_ms);
#endif /* UTILS_RX_H_ */
