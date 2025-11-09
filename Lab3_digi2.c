// main.c — Half-Duplex Loopback con ASCII 7-bit
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "lib/utils_rx/utils_rx.h"
#include "lib/utils_codec/utils_codec.h"

/* ===== Pines (loopback: conectar GP0→GP6 y GP1→GP7) ===== */
#define TX1_PIN  0
#define RX1_PIN  7
#define TX2_PIN  1
#define RX2_PIN  6

/* ===== Protocolo ===== */
#define PWM_TOP        999
#define F_PWM_HZ       800
#define SYMBOL_CYCLES  3
#define DATA_CYCLES    3
#define LINE_MAX       256
#define MAX_RETRIES    3
#define GUARD_FACTOR   0.75f

/* ===== Duty Cycles Control ===== */
#ifndef UAPWMC_DUTY_IDLE
#define UAPWMC_DUTY_IDLE  0.10f
#endif
#ifndef UAPWMC_DUTY_START
#define UAPWMC_DUTY_START 0.20f
#endif
#ifndef UAPWMC_DUTY_STOP
#define UAPWMC_DUTY_STOP  0.80f
#endif

/* ===== Debug ===== */
#define DEBUG_VERBOSE  0  // 1 para ver cada símbolo
#define DEBUG_ERRORS   1  // Solo errores

/* ===== PWM ===== */
typedef struct {
    uint pin, slice, chan;
    uint32_t top;
} txpwm_t;

static void txpwm_init(txpwm_t* h, uint pin, uint32_t f_pwm_hz, uint32_t top) {
    h->pin   = pin;
    h->slice = pwm_gpio_to_slice_num(pin);
    h->chan  = pwm_gpio_to_channel(pin);
    h->top   = top;

    gpio_set_function(pin, GPIO_FUNC_PWM);
    gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);

    uint32_t clk_sys = 125000000u;
    float clkdiv = (float)clk_sys / ((float)f_pwm_hz * (float)(top + 1));

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, top);
    pwm_config_set_clkdiv(&cfg, clkdiv);
    pwm_init(h->slice, &cfg, true);

    pwm_set_chan_level(h->slice, h->chan, 0);
}

static inline void txpwm_set_duty01(const txpwm_t* h, float duty01) {
    if (duty01 < 0.f) duty01 = 0.f;
    if (duty01 > 1.f) duty01 = 1.f;
    uint32_t level = (uint32_t)((float)h->top * duty01 + 0.5f);
    pwm_set_chan_level(h->slice, h->chan, level);
}

/* ===== Timing ===== */
static inline uint32_t Texp_us(void) {
    return 1000000u / F_PWM_HZ;
}

static inline void wait_periods_approx(uint8_t n) {
    uint32_t T = Texp_us();
    for (uint8_t i = 0; i < n; i++) {
        sleep_us(T);
    }
}

/* ===== TX/RX ASCII 7-bit ===== */
static bool link_send_and_receive_ascii7(const txpwm_t* tx, uint rx_pin,
                                         const char* tx_text, char* rx_text,
                                         int n_chars) {
    const uint32_t T = Texp_us();
    const uint32_t guard_time = (uint32_t)(T * GUARD_FACTOR);
    const uint32_t measure_time = T - guard_time;

    // IDLE
    txpwm_set_duty01(tx, UAPWMC_DUTY_IDLE);
    wait_periods_approx(2);

    // START
    txpwm_set_duty01(tx, UAPWMC_DUTY_START);
    wait_periods_approx(SYMBOL_CYCLES);

    // DATOS: 2 símbolos por carácter
    int byte_errors = 0;
    for (int i = 0; i < n_chars; i++) {
        uint8_t ch_tx = (uint8_t)tx_text[i] & 0x7F;
        float duty_msb, duty_lsb;
        
        // Codificar
        codec_char_to_duties(tx_text[i], &duty_msb, &duty_lsb);

#if DEBUG_VERBOSE
        uint8_t msb3 = (ch_tx >> 4) & 0x07;
        uint8_t lsb4 = ch_tx & 0x0F;
        printf("[TX %d] '%c' (0x%02X) MSB3=%d LSB4=%d → d1=%.3f d2=%.3f\n",
               i, (ch_tx >= 32 && ch_tx < 127) ? ch_tx : '.', 
               ch_tx, msb3, lsb4, duty_msb, duty_lsb);
#endif

        // Enviar MSB3
        txpwm_set_duty01(tx, duty_msb);
        sleep_us(guard_time);
        float duty_msb_meas = rx_measure_duty_avg(rx_pin, T, DATA_CYCLES);
        sleep_us(measure_time);

        // Enviar LSB4
        txpwm_set_duty01(tx, duty_lsb);
        sleep_us(guard_time);
        float duty_lsb_meas = rx_measure_duty_avg(rx_pin, T, DATA_CYCLES);
        sleep_us(measure_time);

        // Decodificar
        char ch_rx = codec_duties_to_char(duty_msb_meas, duty_lsb_meas);
        rx_text[i] = ch_rx;

#if DEBUG_VERBOSE
        printf("[RX %d] d1=%.3f d2=%.3f → '%c' (0x%02X) %s\n",
               i, duty_msb_meas, duty_lsb_meas,
               ((uint8_t)ch_rx >= 32 && (uint8_t)ch_rx < 127) ? ch_rx : '.',
               (uint8_t)ch_rx,
               (ch_rx == tx_text[i]) ? "✓" : "✗");
#endif

        if (ch_rx != tx_text[i]) byte_errors++;
    }

    // CHECKSUM (también 2 símbolos)
    uint8_t chk_tx = codec_checksum_xor7((const uint8_t*)tx_text, (size_t)n_chars);
    float chk_duty_msb, chk_duty_lsb;
    codec_char_to_duties((char)chk_tx, &chk_duty_msb, &chk_duty_lsb);

#if DEBUG_VERBOSE
    printf("[TX CHK] 0x%02X → d1=%.3f d2=%.3f\n", chk_tx, chk_duty_msb, chk_duty_lsb);
#endif

    // Enviar checksum MSB
    txpwm_set_duty01(tx, chk_duty_msb);
    sleep_us(guard_time);
    float chk_msb_meas = rx_measure_duty_avg(rx_pin, T, DATA_CYCLES);
    sleep_us(measure_time);

    // Enviar checksum LSB
    txpwm_set_duty01(tx, chk_duty_lsb);
    sleep_us(guard_time);
    float chk_lsb_meas = rx_measure_duty_avg(rx_pin, T, DATA_CYCLES);
    sleep_us(measure_time);

    // Verificar checksum
    char chk_rx_char = codec_duties_to_char(chk_msb_meas, chk_lsb_meas);
    uint8_t chk_rx = (uint8_t)chk_rx_char & 0x7F;
    uint8_t chk_calc = codec_checksum_xor7((const uint8_t*)rx_text, (size_t)n_chars);

#if DEBUG_VERBOSE
    printf("[RX CHK] d1=%.3f d2=%.3f → 0x%02X (calc=0x%02X) %s\n",
           chk_msb_meas, chk_lsb_meas, chk_rx, chk_calc,
           (chk_rx == chk_calc) ? "✓" : "✗");
#endif

    // STOP
    txpwm_set_duty01(tx, UAPWMC_DUTY_STOP);
    wait_periods_approx(SYMBOL_CYCLES);
    txpwm_set_duty01(tx, UAPWMC_DUTY_IDLE);
    wait_periods_approx(1);

    // Validar
    bool chk_ok = (chk_rx == chk_calc);
    bool data_ok = (byte_errors == 0);

    return (data_ok && chk_ok);
}

/* ===== Transmisión con reintentos ===== */
static bool link_send_with_retry(const txpwm_t* tx, uint rx_pin,
                                 const char* msg, char* rx_text, int len) {
    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        if (retry > 0) {
#if DEBUG_ERRORS
            printf("[RETRY %d/%d]\n", retry + 1, MAX_RETRIES);
#endif
            sleep_ms(10);
        }

        bool ok = link_send_and_receive_ascii7(tx, rx_pin, msg, rx_text, len);

        if (ok) {
            return true;
        }
    }

    return false;
}

/* ===== Consola ===== */
static int read_line_usb(char* buf, int maxlen, uint32_t tout_ms) {
    int n = 0;
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    
    while (n < maxlen - 1) {
        if ((to_ms_since_boot(get_absolute_time()) - t0) > tout_ms) break;
        
        int ch = getchar_timeout_us(1000);
        if (ch == PICO_ERROR_TIMEOUT) continue;
        if (ch == '\r' || ch == '\n') break;
        
        buf[n++] = (char)ch;
    }
    
    buf[n] = 0;
    return n;
}

static void print_help(void) {
    printf("\n╔════════════════════════════════════════════════╗\n");
    printf("║           COMANDOS - Half-Duplex              ║\n");
    printf("╠════════════════════════════════════════════════╣\n");
    printf("║ 1: <texto>     → Envía desde Nodo 1           ║\n");
    printf("║ 2: <texto>     → Envía desde Nodo 2           ║\n");
    printf("║ help           → Este menú                    ║\n");
    printf("╚════════════════════════════════════════════════╝\n");
}

/* ===== MAIN ===== */
int main(void) {
    stdio_init_all();
    sleep_ms(500);

    // Inicializar TX
    txpwm_t tx1, tx2;
    txpwm_init(&tx1, TX1_PIN, F_PWM_HZ, PWM_TOP);
    txpwm_init(&tx2, TX2_PIN, F_PWM_HZ, PWM_TOP);

    // Inicializar RX
    rx_in_init(RX1_PIN, 0, 0);
    rx_in_init(RX2_PIN, 0, 0);
    gpio_pull_down(RX1_PIN);
    gpio_pull_down(RX2_PIN);

    // Estado IDLE
    txpwm_set_duty01(&tx1, UAPWMC_DUTY_IDLE);
    txpwm_set_duty01(&tx2, UAPWMC_DUTY_IDLE);
    sleep_ms(100);

    uint tx_active = 1;

    char line[LINE_MAX], msg[LINE_MAX], rx_text[LINE_MAX];

    while (true) {
        printf("\n[Nodo %u]> ", tx_active);
        fflush(stdout);

        int n = read_line_usb(line, LINE_MAX, 600000);
        if (n <= 0) continue;

        // Comando: help
        if (!strncmp(line, "help", 4)) {
            print_help();
            continue;
        }

        // Atajos "1: ..." / "2: ..."
        if ((line[0] == '1' && line[1] == ':') || 
            (line[0] == '2' && line[1] == ':')) {
            tx_active = (line[0] == '1') ? 1 : 2;
            const char* p = line + 2;
            while (*p == ' ') p++;
            
            if (strlen(p) == 0) {
                printf("(mensaje vacío)\n");
                continue;
            }
            
            strncpy(msg, p, LINE_MAX - 1);
            msg[LINE_MAX - 1] = 0;
        } else {
            printf("Comando no reconocido\n");
            continue;
        }

        int m = (int)strlen(msg);

        // Transmitir
        printf("\n────────────────────────────────────────\n");
        
        bool success;
        if (tx_active == 1) {
            printf("Nodo 1 → Nodo 2\n");
            success = link_send_with_retry(&tx1, RX2_PIN, msg, rx_text, m);
        } else {
            printf("Nodo 2 → Nodo 1\n");
            success = link_send_with_retry(&tx2, RX1_PIN, msg, rx_text, m);
        }

        rx_text[m] = 0;

        printf("\n📤 TX: \"%s\"\n", msg);
        printf("📥 RX: \"%s\"\n", rx_text);

        if (success) {
            printf("\nTRANSMISIÓN EXITOSA\n");
        } else {
            printf("\nTRANSMISIÓN FALLIDA\n");
            
            // Mostrar hex dump para debug
            printf("\nHex dump:\n");
            printf("TX: ");
            for (int i = 0; i < m; i++) printf("%02X ", (uint8_t)msg[i]);
            printf("\nRX: ");
            for (int i = 0; i < m; i++) printf("%02X ", (uint8_t)rx_text[i]);
            printf("\n");
        }
        printf("────────────────────────────────────────\n");
    }

    return 0;
}