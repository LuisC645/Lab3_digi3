/**
 * @file practica4_digi.c
 * @brief Chat bidireccional por PWM (TX + RX en modo Polling)
 * @details
 * Comunicación ASCII 7-bit (MSB3 + LSB4) usando duty PWM.
 * Misma app sirve para dos Picos (Full-Duplex) o loopback.
 * Correcciones:
 * 1) Delays de TX a 5 ms para que RX por polling no pierda símbolos.
 * 2) Reset de la MEF RX (got_msb) en START y STOP.
 * 3) try_receive_frame() retorna true en STOP para imprimir aun con CHK ERROR.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "lib/utils_tx/utils_tx.h"   // uapwmc_tx_init, uapwmc_tx_set_duty
#include "lib/utils_rx/utils_rx.h"   // rx_polling_init, rx_get_duty_polling
#include "lib/utils_codec/utils_codec.h"  // codec 7-bit (MSB3+LSB4)

#define TX_PIN        0
#define RX_PIN        1
#define F_PWM_INIT    1000    // 1 kHz
#define RX_TIMEOUT_MS 120
#define MAX_LEN       64

/* Prototipos */
static void send_frame(const char *msg);
static bool try_receive_frame(char *out, int maxlen);

int main(void) {
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(100);

    /* ----- Mensaje de bienvenida / ayuda corta ----- */
    printf("\n====================================================\n");
    printf(" PWM CHAT (ASCII 7-bit) — Modo Polling @ %u Hz\n", F_PWM_INIT);
    printf(" TX: GP%d   RX: GP%d\n", TX_PIN, RX_PIN);
    printf(" Conecta TX->RX entre Picos (o loopback con puenteado).\n");
    printf(" Escribe un mensaje y presiona ENTER para enviarlo.\n");
    printf("====================================================\n");

    /* Inicialización */
    uapwmc_tx_init(TX_PIN, F_PWM_INIT);

    /* Línea en IDLE antes de transmitir */
    uapwmc_tx_set_duty(CODIF_DUTY_IDLE);
    sleep_ms(10);

    rx_polling_init(RX_PIN);  // modo polling

    char tx_msg[MAX_LEN];
    char rx_msg[MAX_LEN];

    printf("> "); fflush(stdout);

    /* Bucle principal */
    while (true) {
        /* ---------- TX por consola ---------- */
        int c = getchar_timeout_us(0);
        static int idx = 0;
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\n' || c == '\r') {
                tx_msg[idx] = '\0';
                if (idx > 0) {
                    printf("\n[TX] \"%s\"\n", tx_msg);
                    send_frame(tx_msg);
                    printf("> ");
                    idx = 0;
                } else {
                    printf("\n> ");
                }
            } else if (idx < MAX_LEN - 1) {
                tx_msg[idx++] = (char)c;
                putchar(c);
            }
        }

        /* ---------- RX por polling ---------- */
        if (try_receive_frame(rx_msg, MAX_LEN)) {
            printf("\n[RX] \"%s\"\n> ", rx_msg);  // imprime aun si checksum falla
            fflush(stdout);
        }

        tight_loop_contents();
    }
}

/* =======================================================
 * Envío de trama (bloqueante) — ASCII 7-bit MSB3+LSB4
 * ======================================================= */
static void send_frame(const char *msg) {
    int n = (int)strlen(msg);
    if (n <= 0) return;

    /* Checksum XOR-7 (0..127) del payload */
    uint8_t chk7 = codif_checksum7((const uint8_t *)msg, (size_t)n);

    /* Guard IDLE */
    uapwmc_tx_set_duty(CODIF_DUTY_IDLE);
    sleep_ms(5);

    /* START */
    uapwmc_tx_set_duty(CODIF_DUTY_START);
    sleep_ms(5);

    /* DATA: cada char 7-bit = MSB3 + LSB4 */
    for (int i = 0; i < n; i++) {
        uint8_t b7   = (uint8_t)msg[i] & 0x7F;
        uint8_t msb3 = (b7 >> 4) & 0x07;    // bits 6..4
        uint8_t lsb4 =  b7       & 0x0F;    // bits 3..0

        float d_msb = codif_msb3_to_duty(msb3);
        float d_lsb = codif_lsb4_to_duty(lsb4);

        uapwmc_tx_set_duty(d_msb);
        sleep_ms(5);
        uapwmc_tx_set_duty(d_lsb);
        sleep_ms(5);
    }

    /* CHK7 también como MSB3+LSB4 */
    uint8_t chk_msb3 = (chk7 >> 4) & 0x07;
    uint8_t chk_lsb4 =  chk7       & 0x0F;
    uapwmc_tx_set_duty(codif_msb3_to_duty(chk_msb3));
    sleep_ms(5);
    uapwmc_tx_set_duty(codif_lsb4_to_duty(chk_lsb4));
    sleep_ms(5);

    /* STOP + vuelta a IDLE */
    uapwmc_tx_set_duty(CODIF_DUTY_STOP);
    sleep_ms(5);

    uapwmc_tx_set_duty(CODIF_DUTY_IDLE);
}

/* =======================================================
 * Recepción (Polling) — decodifica MSB3+LSB4, retorna true en STOP
 * ======================================================= */
static bool try_receive_frame(char *out, int maxlen) {
    static uint8_t rx_data[MAX_LEN];
    static int  idx        = 0;
    static bool receiving  = false;

    /* Parser de byte (MSB3+LSB4) */
    static bool    got_msb  = false;
    static uint8_t tmp_byte = 0;

    float d = rx_get_duty_polling(RX_PIN, RX_TIMEOUT_MS);
    if (d < 0.0f) return false;

    /* IDLE (opcional) */
    if (!receiving && codif_duty_match(d, CODIF_DUTY_IDLE)) {
        return false;
    }

    /* START */
    if (!receiving && codif_duty_match(d, CODIF_DUTY_START)) {
        receiving = true;
        idx = 0;
        got_msb = false;   // reset parser
        tmp_byte = 0;
        // (no imprimir duty)
        return false;
    }

    /* STOP */
    if (receiving && codif_duty_match(d, CODIF_DUTY_STOP)) {
        receiving = false;
        got_msb = false;   // reset parser

        if (idx < 1) return false;  // solo START/STOP -> inválido

        /* Último byte es CHK7 */
        uint8_t chk_rx   = rx_data[idx - 1];
        uint8_t chk_calc = codif_checksum7(rx_data, (size_t)(idx - 1));

        /* Copia payload a salida */
        int data_len = idx - 1;
        if (data_len > maxlen - 1) data_len = maxlen - 1;
        for (int i = 0; i < data_len; i++) out[i] = (char)rx_data[i];
        out[data_len] = '\0';

        printf("[RX] CHK7: %s\n", (chk_rx == chk_calc) ? "OK" : "ERROR");
        idx = 0;

        /* SIEMPRE retorna true al ver STOP (imprime aunque falle checksum) */
        return true;
    }

    /* Datos: dentro de las bandas MSB3/LSB4 */
    if (receiving) {
        if (d >= CODIF_DUTY_MIN_MSB3 && d <= CODIF_DUTY_MAX_MSB3) {
            /* MSB3 */
            tmp_byte = (uint8_t)(codif_duty_to_msb3(d) << 4);
            got_msb = true;
        } else if (d >= CODIF_DUTY_MIN_LSB4 && d <= CODIF_DUTY_MAX_LSB4) {
            /* LSB4 */
            if (got_msb) {
                tmp_byte |= codif_duty_to_lsb4(d);
                if (idx < MAX_LEN) rx_data[idx++] = tmp_byte;
                got_msb = false;
            }
            /* else: llegó LSB sin MSB -> se ignora */
        }
        /* else: fuera de bandas -> ignora (ruido/IDLE intermedio) */
    }

    return false;
}
