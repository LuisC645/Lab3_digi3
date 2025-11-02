/**
 * @file utils_tx.c
 * @brief Implementación TX por PWM (polling) + protocolo con checksum XOR.
 */
#include "utils_tx.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

/* Estado PWM */
static uint8_t  s_slice=0, s_channel=0;
static uint32_t s_gpio=0, s_top=999;

static inline float clamp01(float x){
    if (x<0.f) return 0.f; if (x>1.f) return 1.f; return x;
}

void tx_pwm_init(uint32_t gpio, uint32_t f_pwm_hz, uint32_t top){
    s_gpio=gpio; s_top=top;
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    s_slice   = (uint8_t)pwm_gpio_to_slice_num(gpio);
    s_channel = (uint8_t)pwm_gpio_to_channel(gpio);
    pwm_set_wrap(s_slice, s_top);
    uint32_t f_sys = clock_get_hz(clk_sys);
    float clkdiv = (float)f_sys / ((float)f_pwm_hz * (s_top + 1));
    pwm_set_clkdiv(s_slice, clkdiv);
    pwm_set_chan_level(s_slice, s_channel, 0);
    pwm_set_enabled(s_slice, true);
}

void tx_pwm_set_freq(uint32_t f_pwm_hz){
    uint32_t f_sys = clock_get_hz(clk_sys);
    float clkdiv = (float)f_sys / ((float)f_pwm_hz * (s_top + 1));
    pwm_set_clkdiv(s_slice, clkdiv);
}

void tx_pwm_set_duty01(float duty01){
    duty01 = clamp01(duty01);
    uint32_t level = (uint32_t)((s_top + 1) * duty01 + 0.5f);
    if (level > s_top) level = s_top;
    pwm_set_chan_level(s_slice, s_channel, level);
}

void tx_pwm_wait_cycles(uint8_t cycles){
    for(uint8_t i=0;i<cycles;++i){
        uint16_t prev = (uint16_t)pwm_hw->slice[s_slice].ctr;
        for(;;){
            uint16_t now = (uint16_t)pwm_hw->slice[s_slice].ctr;
            if(now < prev) break; prev = now;
        }
    }
}

uint32_t tx_pwm_get_gpio(void){ return s_gpio; }

static inline void tx_hold(float duty, uint8_t cycles){
    tx_pwm_set_duty01(duty);
    tx_pwm_wait_cycles(cycles);
}

void tx_send_frame(const uint8_t* data, uint8_t len,
                   uint8_t symbol_cycles, uint8_t data_cycles)
{
    /* Preamble IDLE */
    tx_hold(UAPWMC_DUTY_IDLE, 2);

    /* START */
    tx_hold(UAPWMC_DUTY_START, symbol_cycles);

    /* LEN */
    tx_hold(tx_duty_from_byte(len), data_cycles);

    /* DATA */
    for(uint8_t i=0;i<len;++i){
        tx_hold(tx_duty_from_byte(data[i]), data_cycles);
    }

    /* CHK = XOR(payload) */
    uint8_t chk = tx_checksum_xor(data, len);
    tx_hold(tx_duty_from_byte(chk), data_cycles);

    /* STOP + IDLE */
    tx_hold(UAPWMC_DUTY_STOP, symbol_cycles);
    tx_hold(UAPWMC_DUTY_IDLE, 1);
}
