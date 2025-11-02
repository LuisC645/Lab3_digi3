#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "lib/utils_tx/utils_tx.h"
#include "lib/utils_rx/utils_rx.h"
#include "lib/utils_codec/utils_codec.h"

#define TX_PIN         2
#define RX_PIN         15
#define PWM_TOP        999
#define F_PWM_INIT     1000     // 1 kHz
#define SYMBOL_CYCLES  2
#define DATA_CYCLES    2
#define LINE_MAX       128

static inline void tx_hold(float duty, uint8_t cycles){
    tx_pwm_set_duty01(duty);
    tx_pwm_wait_cycles(cycles);
}

static int read_line_usb(char* buf, int maxlen, uint32_t tout_ms){
    int n=0; uint32_t t0=to_ms_since_boot(get_absolute_time());
    while(n<maxlen-1){
        if ((to_ms_since_boot(get_absolute_time())-t0)>tout_ms) break;
        int ch=getchar_timeout_us(1000);
        if (ch==PICO_ERROR_TIMEOUT) continue;
        if (ch=='\r'||ch=='\n') break;
        buf[n++]=(char)ch;
    }
    buf[n]=0; return n;
}

int main(void){
    stdio_init_all();
    sleep_ms(300);

    tx_pwm_init(TX_PIN, F_PWM_INIT, PWM_TOP);
    rx_in_init(RX_PIN, 0, 0);
    const uint32_t Texp_us = 1000000u / F_PWM_INIT;

    printf("\nDuty-Codec TX GP%u -> RX GP%u @ %u Hz (puentea GP%u->GP%u)\n",
           TX_PIN, RX_PIN, F_PWM_INIT, TX_PIN, RX_PIN);
    printf("Alfabeto(%d): \"%s\"\n", CODEC_NSYMS, CODEC_ALPHABET);
    printf("Escribe texto y Enter. Se enviará por duty y se verificará checksum (por duty de índice).\n");

    char     tx_text[LINE_MAX];
    char     rx_text[LINE_MAX];
    while (true){
        printf("\n> "); fflush(stdout);
        int n = read_line_usb(tx_text, LINE_MAX, 60000);
        if (n<=0){ printf("(vacío/timeout)\n"); continue; }
        if (n>255) n=255;

        /* Preámbulo: IDLE y START */
        tx_hold(UAPWMC_DUTY_IDLE, 2);
        tx_hold(UAPWMC_DUTY_START, SYMBOL_CYCLES);

        /* Envío/recepción sincronizada carácter a carácter */
        for (int i=0;i<n;i++){
            float duty_set = codec_char_to_duty(tx_text[i]);
            tx_pwm_set_duty01(duty_set);
            float duty_meas = rx_measure_duty_avg(RX_PIN, Texp_us, DATA_CYCLES);
            rx_text[i] = codec_duty_to_char(duty_meas);
        }

        /* CHK: XOR de índices del alfabeto (enviado como “otro carácter”) */
        int chk_idx_tx = codec_checksum_idx_from_text(tx_text, (size_t)n);
        float duty_chk = codec_index_to_duty(chk_idx_tx);
        tx_pwm_set_duty01(duty_chk);
        float duty_chk_meas = rx_measure_duty_avg(RX_PIN, Texp_us, DATA_CYCLES);
        int chk_idx_rx = codec_duty_to_index(duty_chk_meas);

        /* STOP + pequeño IDLE */
        tx_hold(UAPWMC_DUTY_STOP, SYMBOL_CYCLES);
        tx_hold(UAPWMC_DUTY_IDLE, 1);

        /* Cierra cadena RX y verifica */
        rx_text[n] = 0;

        int mism=0;
        for (int i=0;i<n;i++) if (rx_text[i]!=tx_text[i]) mism++;
        int pass = (mism==0) && (chk_idx_rx==chk_idx_tx);

        printf("TX: \"%.*s\"\n", n, tx_text);
        printf("RX: \"%s\"\n", rx_text);
        printf("CHK idx: tx=%d  rx=%d  => %s\n",
               chk_idx_tx, chk_idx_rx, (chk_idx_tx==chk_idx_rx)?"OK":"FAIL");
        printf("RESULT: BYTES=%s  CHK=%s  => %s\n",
               (mism==0)?"OK":"FAIL",
               (chk_idx_tx==chk_idx_rx)?"OK":"FAIL",
               pass?"PASS":"FAIL");
    }
    // return 0;
}
