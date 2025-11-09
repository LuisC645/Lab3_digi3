/**
 * @file utils_rx.c
 * @brief Implementación de RX por polling para medir duty PWM.
 */
#include "utils_rx.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

/* --- Helpers internos --------------------------------------------------- */

/* Espera a que el pin alcance 'level' (0/1) antes de timeout.
 * Devuelve time_us_32() del instante o 0 si hubo timeout. */
static inline uint32_t wait_level_until(uint32_t pin, int level,
                                        uint32_t t_start_us, uint32_t timeout_ms) {
    const uint32_t timeout_us = timeout_ms * 1000u;
    for (;;) {
        if ((gpio_get(pin) != 0) == (level != 0)) {
            return time_us_32();
        }
        if ((uint32_t)(time_us_32() - t_start_us) > timeout_us) {
            return 0u; /* timeout */
        }
        tight_loop_contents();
    }
}

/* --- API ---------------------------------------------------------------- */

void rx_polling_init(uint32_t gpio_pin) {
    gpio_init(gpio_pin);
    gpio_set_dir(gpio_pin, GPIO_IN);
    gpio_disable_pulls(gpio_pin);   /* línea conducida externamente (PWM TX) */
    /* Si quieres forzar un estado en reposo, usa pull_down/up aquí. */
}

float rx_get_duty_polling(uint32_t gpio_pin, uint32_t timeout_ms) {
    const uint32_t t0 = time_us_32();

    /* 1) Sincroniza a borde de subida: garantiza inicio de periodo. */
    /* Si empezamos en alto, espera bajar y luego subir para no cortar el pulso. */
    if (gpio_get(gpio_pin)) {
        if (!wait_level_until(gpio_pin, 0, t0, timeout_ms)) return -1.0f;
    }
    uint32_t t_rise = wait_level_until(gpio_pin, 1, t0, timeout_ms);
    if (!t_rise) return -1.0f;

    /* 2) Espera la bajada (fin de Thigh). */
    uint32_t t_fall = wait_level_until(gpio_pin, 0, t0, timeout_ms);
    if (!t_fall) return -1.0f;

    /* 3) Espera la siguiente subida (fin de periodo). */
    uint32_t t_next_rise = wait_level_until(gpio_pin, 1, t0, timeout_ms);
    if (!t_next_rise) return -1.0f;

    /* 4) Calcula tiempos (cuida underflow con aritmética uint32_t). */
    uint32_t t_high   = t_fall       - t_rise;
    uint32_t t_period = t_next_rise  - t_rise;
    if (t_period == 0u || t_high > t_period) return -1.0f;

    float duty = (float)t_high / (float)t_period;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    return duty;
}
