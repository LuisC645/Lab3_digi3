/**
 * @file utils_tx.h
 * @brief Transmisor UAPWMC por PWM (polling de ciclos).
 */
#ifndef UTILS_TX_H_
#define UTILS_TX_H_

#include <stdint.h>
#include <stddef.h>

/* Duty de control: toma los de tu codec (codif.h) */
#ifndef CODIF_DUTY_START
  #define CODIF_DUTY_START 0.97f
#endif
#ifndef CODIF_DUTY_STOP
  #define CODIF_DUTY_STOP  0.88f
#endif
#ifndef CODIF_DUTY_IDLE
  #define CODIF_DUTY_IDLE  0.05f
#endif

#define UAPWMC_DUTY_START  CODIF_DUTY_START
#define UAPWMC_DUTY_STOP   CODIF_DUTY_STOP
#define UAPWMC_DUTY_IDLE   CODIF_DUTY_IDLE

/* Rango para datos “continuos” (byte→duty). Puedes sobreescribirlos. */
#ifndef UAPWMC_DATA_DMIN
  #define UAPWMC_DATA_DMIN 0.15f
#endif
#ifndef UAPWMC_DATA_DMAX
  #define UAPWMC_DATA_DMAX 0.75f
#endif

/** Inicializa PWM TX en @p gpio_pin a @p freq_hz. */
void uapwmc_tx_init(uint32_t gpio_pin, uint32_t freq_hz);

/** Fija duty [0..1]. */
void uapwmc_tx_set_duty(float duty01);

/** Mantiene @p duty durante @p cycles ciclos del PWM (bloqueante). */
void tx_hold(float duty, uint8_t cycles);

/** Envía trama: [IDLE][START][LEN][DATA...][CHK][STOP][IDLE]. */
void uapwmc_tx_send_frame(const uint8_t* data, uint8_t length, uint32_t symbol_cycles);

/** Byte (0..255) → duty en [UAPWMC_DATA_DMIN..UAPWMC_DATA_DMAX]. */
static inline float uapwmc_tx_byte_to_duty(uint8_t byte){
    const float dmin = UAPWMC_DATA_DMIN, dmax = UAPWMC_DATA_DMAX;
    const float span = (dmax - dmin);
    return dmin + ( (float)byte / 255.0f ) * span;
}

/** XOR-8 de un bloque de datos. */
static inline uint8_t uapwmc_tx_checksum(const uint8_t* data, size_t length){
    uint8_t c = 0;
    for (size_t i=0;i<length;++i) c ^= data[i];
    return c;
}

#endif /* UTILS_TX_H_ */
