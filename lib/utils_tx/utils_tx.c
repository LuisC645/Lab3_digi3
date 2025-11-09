/**
 * @file utils_tx.c
 * @brief Implementación del transmisor UAPWMC por PWM (polling de ciclos).
 */
#include "utils_tx.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

/* Wrap fijo para buena resolución (clkdiv se ajusta en init). */
#ifndef UAPWMC_PWM_WRAP
#define UAPWMC_PWM_WRAP 9999u
#endif

/* Estado (un solo canal TX) */
static uint8_t  s_slice = 0;
static uint8_t  s_chan  = 0;
static uint32_t s_wrap  = UAPWMC_PWM_WRAP;

/* ---------- Helpers ---------- */

static inline float clamp01(float x){
    return (x < 0.f) ? 0.f : (x > 1.f ? 1.f : x);
}

/* Espera @cycles wraps del contador de PWM, detectando “now < prev”. */
static void wait_wrap_cycles(uint8_t cycles){
    for (uint8_t k=0;k<cycles;++k){
        uint16_t prev = (uint16_t)pwm_hw->slice[s_slice].ctr;
        for (;;) {
            uint16_t now = (uint16_t)pwm_hw->slice[s_slice].ctr;
            if (now < prev) break; /* wrap detectado */
            prev = now;
            tight_loop_contents();
        }
    }
}

/* ---------- API ---------- */

void uapwmc_tx_init(uint32_t gpio_pin, uint32_t freq_hz){
    /* Función PWM en pin, mejora de forma de onda por HW. */
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    gpio_set_drive_strength(gpio_pin, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_slew_rate(gpio_pin, GPIO_SLEW_RATE_FAST);

    s_slice = pwm_gpio_to_slice_num(gpio_pin);
    s_chan  = pwm_gpio_to_channel(gpio_pin);
    s_wrap  = UAPWMC_PWM_WRAP;

    /* Configura wrap y clkdiv para alcanzar freq_hz ~= clk_sys/(clkdiv*(wrap+1)) */
    pwm_set_wrap(s_slice, s_wrap);
    const uint32_t f_sys = clock_get_hz(clk_sys);
    float clkdiv = (float)f_sys / ( (float)freq_hz * (float)(s_wrap + 1u) );
    if (clkdiv < 1.0f)    clkdiv = 1.0f;   /* límites seguros del HW */
    if (clkdiv > 255.0f)  clkdiv = 255.0f;

    pwm_set_clkdiv(s_slice, clkdiv);
    pwm_set_chan_level(s_slice, s_chan, 0);
    pwm_set_enabled(s_slice, true);

    /* Línea en IDLE al arrancar */
    uapwmc_tx_set_duty(UAPWMC_DUTY_IDLE);
}

void uapwmc_tx_set_duty(float duty01){
    duty01 = clamp01(duty01);
    /* Nivel = duty*(wrap+1), saturado a wrap */
    uint32_t level = (uint32_t)((float)(s_wrap + 1u) * duty01 + 0.5f);
    if (level > s_wrap) level = s_wrap;
    pwm_set_chan_level(s_slice, s_chan, level);
}

void tx_hold(float duty, uint8_t cycles){
    uapwmc_tx_set_duty(duty);
    wait_wrap_cycles(cycles);
}

void uapwmc_tx_send_frame(const uint8_t* data, uint8_t length, uint32_t symbol_cycles){
    if (!data || !length) {
        /* Secuencia mínima para mantener protocolo visible en RX */
        tx_hold(UAPWMC_DUTY_IDLE, 2);
        tx_hold(UAPWMC_DUTY_START, symbol_cycles);
        tx_hold(UAPWMC_DUTY_STOP,  symbol_cycles);
        tx_hold(UAPWMC_DUTY_IDLE,  1);
        return;
    }

    /* Guard + START */
    tx_hold(UAPWMC_DUTY_IDLE,  2);
    tx_hold(UAPWMC_DUTY_START, symbol_cycles);

    /* LEN */
    tx_hold(uapwmc_tx_byte_to_duty(length), symbol_cycles);

    /* DATA */
    for (uint8_t i=0;i<length;++i){
        tx_hold(uapwmc_tx_byte_to_duty(data[i]), symbol_cycles);
    }

    /* CHK */
    const uint8_t chk = uapwmc_tx_checksum(data, length);
    tx_hold(uapwmc_tx_byte_to_duty(chk), symbol_cycles);

    /* STOP + pequeño IDLE */
    tx_hold(UAPWMC_DUTY_STOP, symbol_cycles);
    tx_hold(UAPWMC_DUTY_IDLE, 1);
}
