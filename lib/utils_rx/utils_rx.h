/**
 * @file utils_rx.h
 * @brief RX por polling para medir duty PWM (0.0..1.0).
 */
#ifndef UTILS_RX_H_
#define UTILS_RX_H_

#include <stdint.h>

/**
 * @brief Configura el GPIO como entrada para RX (sin pulls).
 * @param gpio_pin Número de GPIO.
 */
void rx_polling_init(uint32_t gpio_pin);

/**
 * @brief Mide duty = Thigh / T mediante polling.
 * @param gpio_pin   GPIO de entrada.
 * @param timeout_ms Tiempo máximo total (ms).
 * @return duty (0.0..1.0) o -1.0f si timeout/error.
 */
float rx_get_duty_polling(uint32_t gpio_pin, uint32_t timeout_ms);

#endif /* UTILS_RX_H_ */
