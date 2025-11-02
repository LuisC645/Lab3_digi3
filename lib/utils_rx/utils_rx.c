/**
 * @file utils_rx.c
 * @brief RX por polling: detección de START/STOP y decodificación con checksum.
 */
#include "utils_rx.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

/* Helpers */
static inline uint32_t ts_us(void){ return time_us_32(); }
static inline bool in_band(float x, float center, float eps){
    return (x >= (center - eps)) && (x <= (center + eps));
}

void rx_in_init(uint32_t gpio, int pull_up, int pull_down){
    gpio_init(gpio);
    gpio_set_dir(gpio, false);
    gpio_disable_pulls(gpio);
    if (pull_up)   gpio_pull_up(gpio);
    if (pull_down) gpio_pull_down(gpio);
}

/* --- Medición de un ciclo / duty promedio (ya la tenías antes) --- */
static uint32_t wait_edge_poll(uint32_t pin, int level_actual, uint32_t timeout_us){
    const uint32_t t0 = ts_us();
    int last = level_actual;
    while ((ts_us() - t0) < timeout_us) {
        int v = gpio_get(pin);
        if (v != last) {
            int vv = gpio_get(pin);
            if (vv == v) return ts_us();
        }
        last = v;
    }
    return 0;
}

bool rx_measure_one_cycle(uint32_t pin, uint32_t expected_period_us,
                          uint32_t* period_us, uint32_t* high_us){
    const uint32_t TOUT = expected_period_us * 5u;
    uint32_t t_rise1=0, t_fall=0, t_rise2=0;
    int lvl = gpio_get(pin);
    if (lvl == 1) { if (!wait_edge_poll(pin, 1, TOUT)) return false; lvl = 0; }
    t_rise1 = wait_edge_poll(pin, lvl, TOUT); if(!t_rise1) return false;
    t_fall  = wait_edge_poll(pin, 1,   TOUT); if(!t_fall ) return false;
    t_rise2 = wait_edge_poll(pin, 0,   TOUT); if(!t_rise2) return false;
    uint32_t T  = t_rise2 - t_rise1;
    uint32_t TH = t_fall  - t_rise1;
    if (T==0 || TH>T) return false;
    *period_us = T; *high_us = TH; return true;
}

float rx_measure_duty_avg(uint32_t pin, uint32_t expected_period_us, int cycles){
    if (cycles < 1) cycles = 1;
    uint64_t sumT=0, sumH=0; int ok=0;
    for(int i=0;i<cycles;++i){
        uint32_t T,H;
        if(!rx_measure_one_cycle(pin, expected_period_us, &T, &H)) continue;
        sumT += T; sumH += H; ok++;
    }
    if (ok==0 || sumT==0) return -1.f;
    float duty = (float)sumH / (float)sumT;
    if (duty<0.f) duty=0.f; if (duty>1.f) duty=1.f;
    return duty;
}

/* --- Recepción de trama completa --- */
static bool wait_for_start(uint32_t pin, uint32_t expected_period_us, uint32_t timeout_ms){
    const uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - t0) < timeout_ms){
        float d = rx_measure_duty_avg(pin, expected_period_us, 1);
        if (d < 0.f) continue;
        if (in_band(d, UAPWMC_DUTY_START, UAPWMC_EPS_CTRL)) return true;
    }
    return false;
}

static bool expect_stop(uint32_t pin, uint32_t expected_period_us, uint32_t timeout_ms){
    const uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - t0) < timeout_ms){
        float d = rx_measure_duty_avg(pin, expected_period_us, 1);
        if (d < 0.f) continue;
        if (in_band(d, UAPWMC_DUTY_STOP, UAPWMC_EPS_CTRL)) return true;
    }
    return false;
}

bool rx_recv_frame(uint32_t pin, uint32_t expected_period_us,
                   uint8_t* out, uint8_t maxlen, uint8_t* out_len,
                   uint32_t timeout_ms)
{
    *out_len = 0;
    /* 1) Espera START */
    if (!wait_for_start(pin, expected_period_us, timeout_ms)) return false;

    /* 2) LEN (promedia 2 ciclos para robustez) */
    float d_len = rx_measure_duty_avg(pin, expected_period_us, 2);
    if (d_len < 0.f) return false;
    uint8_t len = rx_byte_from_duty(d_len);
    if (len > maxlen) return false;

    /* 3) DATA */
    for(uint8_t i=0;i<len;++i){
        float dd = rx_measure_duty_avg(pin, expected_period_us, 2);
        if (dd < UAPWMC_MIN_VALID_DUTY) return false;
        out[i] = rx_byte_from_duty(dd);
    }

    /* 4) CHK */
    float dchk = rx_measure_duty_avg(pin, expected_period_us, 2);
    if (dchk < 0.f) return false;
    uint8_t chk = rx_byte_from_duty(dchk);

    /* 5) STOP */
    if (!expect_stop(pin, expected_period_us, timeout_ms)) return false;

    /* 6) Verificación checksum */
    uint8_t calc = rx_checksum_xor(out, len);
    if (calc != chk) return false;

    *out_len = len;
    return true;
}
