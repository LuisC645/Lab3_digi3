// main.c — TX/RX por duty con ASCII 7-bit (MSB3 + LSB4), C puro.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "lib/utils_tx/utils_tx.h"
#include "lib/utils_rx/utils_rx.h"
#include "lib/utils_codec/utils_codec.h"   // codec8_* y codec16_*

#define TX_PIN         2
#define RX_PIN         15
#define PWM_TOP        999
#define F_PWM_INIT     1000    // 1 kHz
#define SYMBOL_CYCLES  2
#define DATA_CYCLES    2
#define LINE_MAX       256

static inline void tx_hold(float duty, uint8_t cycles){
    tx_pwm_set_duty01(duty);
    tx_pwm_wait_cycles(cycles);
}

static int read_line_usb(char* buf, int maxlen, uint32_t tout_ms){
    int n=0; uint32_t t0=to_ms_since_boot(get_absolute_time());
    while(n<maxlen-1){
        if ((to_ms_since_boot(get_absolute_time())-t0) > tout_ms) break;
        int ch = getchar_timeout_us(1000);
        if (ch==PICO_ERROR_TIMEOUT) continue;
        if (ch=='\r' || ch=='\n') break;
        buf[n++] = (char)ch;
    }
    buf[n]=0; return n;
}

/* XOR 7-bit para checksum */
static inline uint8_t chk7(const uint8_t* p, size_t n){
    uint8_t c=0; for(size_t i=0;i<n;++i) c ^= (p[i] & 0x7F); return (uint8_t)(c & 0x7F);
}

int main(void){
    stdio_init_all();
    sleep_ms(300);

    tx_pwm_init(TX_PIN, F_PWM_INIT, PWM_TOP);
    rx_in_init(RX_PIN, 0, 0);
    const uint32_t Texp_us = 1000000u / F_PWM_INIT;

    printf("\nASCII-7 (3+4) TX GP%u -> RX GP%u @ %u Hz  TOP=%u  (puentea GP%u->GP%u)\n",
           TX_PIN, RX_PIN, F_PWM_INIT, PWM_TOP, TX_PIN, RX_PIN);
    printf("Protocolo: IDLE -> START -> (MSB3, LSB4)*N -> (MSB3, LSB4 de CHK7) -> STOP -> IDLE\n");
    printf("Escribe texto y Enter para enviar/recibir.\n");

    char     tx_text[LINE_MAX];
    uint8_t  rx_bytes[LINE_MAX];

    while (1){
        printf("\n> "); fflush(stdout);
        int n = read_line_usb(tx_text, LINE_MAX, 60000);
        if (n <= 0){ printf("(vacío/timeout)\n"); continue; }
        if (n > 255) n = 255;

        /* Preambulo */
        tx_hold(UAPWMC_DUTY_IDLE, 2);
        tx_hold(UAPWMC_DUTY_START, SYMBOL_CYCLES);

        /* DATA: por cada char 7-bit -> MSB3 + LSB4 */
        for (int i=0; i<n; ++i){
            uint8_t b7 = ((uint8_t)tx_text[i]) & 0x7F;   // 7 bits
            uint8_t msb3 = (b7 >> 4) & 0x07;             // bits 6..4
            uint8_t lsb4 =  b7       & 0x0F;             // bits 3..0

            // MSB3 (8 bins)
            tx_pwm_set_duty01(codec8_tribit_to_duty(msb3));
            float d_msb = rx_measure_duty_avg(RX_PIN, Texp_us, DATA_CYCLES);
            uint8_t r_msb3 = codec8_duty_to_tribit(d_msb);

            // LSB4 (16 bins)
            tx_pwm_set_duty01(codec16_nibble_to_duty(lsb4));
            float d_lsb = rx_measure_duty_avg(RX_PIN, Texp_us, DATA_CYCLES);
            uint8_t r_lsb4 = codec16_duty_to_nibble(d_lsb);

            rx_bytes[i] = (uint8_t)((r_msb3 << 4) | (r_lsb4 & 0x0F));
        }

        /* CHK7 (XOR 7 bits), también en 3+4 */
        uint8_t c7 = chk7((const uint8_t*)tx_text, (size_t)n);
        uint8_t c7_msb3 = (c7 >> 4) & 0x07;
        uint8_t c7_lsb4 =  c7       & 0x0F;

        // MSB3 de CHK7
        tx_pwm_set_duty01(codec8_tribit_to_duty(c7_msb3));
        float d_cmsb = rx_measure_duty_avg(RX_PIN, Texp_us, DATA_CYCLES);
        uint8_t r_cmsb3 = codec8_duty_to_tribit(d_cmsb);

        // LSB4 de CHK7
        tx_pwm_set_duty01(codec16_nibble_to_duty(c7_lsb4));
        float d_clsb = rx_measure_duty_avg(RX_PIN, Texp_us, DATA_CYCLES);
        uint8_t r_clsb4 = codec16_duty_to_nibble(d_clsb);

        uint8_t c7_rx = (uint8_t)((r_cmsb3 << 4) | (r_clsb4 & 0x0F));

        /* STOP + IDLE */
        tx_hold(UAPWMC_DUTY_STOP, SYMBOL_CYCLES);
        tx_hold(UAPWMC_DUTY_IDLE, 1);

        /* Verificación y salida */
        char rx_text[LINE_MAX];
        int mism = 0;
        for (int i=0;i<n;++i){
            rx_bytes[i] &= 0x7F;
            char rxch = (char)rx_bytes[i];
            char txch = (char)(((uint8_t)tx_text[i]) & 0x7F);
            rx_text[i] = rxch;
            if (rxch != txch) mism++;
        }
        rx_text[n] = 0;

        uint8_t c7_calc = chk7(rx_bytes, (size_t)n);
        int bytes_ok = (mism == 0);
        int chk_ok   = (c7_rx == c7_calc);
        int pass     = bytes_ok && chk_ok;

        printf("TX(%d): \"%.*s\"\n", n, n, tx_text);
        printf("RX(%d): \"%s\"\n", n, rx_text);
        printf("CHK7: tx=%u  rx=%u  calc(rx)=%u  => %s\n",
               (unsigned)c7, (unsigned)c7_rx, (unsigned)c7_calc,
               chk_ok ? "OK" : "FAIL");
        printf("RESULT: BYTES=%s  CHK=%s  => %s\n",
               bytes_ok ? "OK" : "FAIL",
               chk_ok   ? "OK" : "FAIL",
               pass     ? "PASS" : "FAIL");

        sleep_ms(150);
    }
    // return 0;
}
