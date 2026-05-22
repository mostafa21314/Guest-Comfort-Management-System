/*
 * Smart Home — Firebase REST + IR AC control
 *
 * New in this revision
 * ────────────────────
 * • IR LED wired to GPIO_NUM_19 through a 2N2222 / BC337 NPN transistor
 *   (collector → LED anode via 33 Ω, emitter → GND, base → 100 Ω → GPIO 19).
 *
 * • Uses the ESP-IDF RMT (Remote Control) peripheral to generate a clean
 *   38 kHz carrier — no bit-banging, no timing drift under FreeRTOS.
 *
 * • AC command format written by the dashboard:
 *       "AC_SET_TEMP:24"   (integer 16..30)
 *
 * • IR protocol: NEC-style — most mid-range split ACs (Gree, Midea, Carrier,
 *   Aux, …) use proprietary 48-bit or 64-bit frames that extend NEC timing.
 *   REPLACE ir_ac_build_frame() with your AC's actual bit pattern.
 *   See README below for how to capture your remote's codes.
 *
 * README — capturing your AC remote's IR codes
 * ─────────────────────────────────────────────
 * 1. Wire a TSOP4838 IR receiver to another GPIO (e.g. GPIO 34, 3.3 V, GND).
 * 2. Flash the ESP-IDF "ir_rx" example, point your AC remote at it and press
 *    the target temperature buttons.
 * 3. The example prints the raw RMT symbols (mark/space durations in µs).
 * 4. Copy those durations into ir_ac_build_frame() below, one symbol per
 *    rmt_symbol_word_t entry: { .duration0 = mark_us, .level0 = 1,
 *                               .duration1 = space_us, .level1 = 0 }.
 * 5. Set IR_FRAME_SYMBOLS to the total count of symbols you captured.
 *
 * The placeholder implementation below sends a valid NEC address/command
 * pair where the command byte encodes the temperature (0x10 + temp - 16).
 * This is NOT correct for any specific AC unit but will exercise the
 * transmitter hardware while you capture your unit's real codes.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_http_client.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"
#include "secrets.h"

// ── WiFi credentials (in secrets.h) ──────────────────────────────────────────
#define WIFI_MAX_RETRY  10

// ── Firebase ──────────────────────────────────────────────────────────────────
#define FIREBASE_BASE     "/smarthome/room001"
static char fb_id_token[1200]          = {0};
static TickType_t fb_token_obtained_at = 0;
#define TOKEN_REFRESH_INTERVAL_MS  (55 * 60 * 1000)

#define PATH_TEMP       FIREBASE_BASE "/temperature.json"
#define PATH_HUM        FIREBASE_BASE "/humidity.json"
#define PATH_ROOM       FIREBASE_BASE "/room.json"
#define PATH_COUNT      FIREBASE_BASE "/count.json"
#define PATH_COMMAND    FIREBASE_BASE "/command.json"

// ── Pin definitions ───────────────────────────────────────────────────────────
#define DHT_PIN         GPIO_NUM_26
#define PIR_PIN         GPIO_NUM_25
#define IR_OUTER_PIN    GPIO_NUM_13
#define IR_INNER_PIN    GPIO_NUM_14
#define RELAY_PIN       GPIO_NUM_23
#define ATOMIZER_PIN    GPIO_NUM_21
#define DFPLAYER_TX_PIN GPIO_NUM_17
#define DFPLAYER_UART   UART_NUM_2
#define DFPLAYER_BAUD   9600

// ── IR TX ─────────────────────────────────────────────────────────────────────
#define IR_TX_GPIO      GPIO_NUM_19   // NPN transistor base (see wiring note above)
#define IR_CARRIER_HZ   38000         // standard NEC carrier
#define IR_CARRIER_DUTY 0.33f         // 33 % duty gives clean demodulation

// NEC timing constants (µs)
#define NEC_LEADING_MARK    9000
#define NEC_LEADING_SPACE   4500
#define NEC_BIT_MARK         560
#define NEC_ONE_SPACE       1690
#define NEC_ZERO_SPACE       560
#define NEC_TRAIL_MARK       560

// Total symbols for a 32-bit NEC frame:
//   1 header + 32 bits × 1 symbol each + 1 trailing mark = 34 symbols.
// Adjust IR_FRAME_SYMBOLS if your AC uses more bits.
#define IR_FRAME_SYMBOLS   34

// AC address byte — replace with your AC's NEC address if known.
#define AC_NEC_ADDRESS     0xB2

// ── Timing ────────────────────────────────────────────────────────────────────
#define POLL_PERIOD_MS         10
#define DEBOUNCE_SAMPLES       5
#define SEQUENCE_TIMEOUT_MS    3000
#define COOLDOWN_MS            2000
#define DHT_INTERVAL_MS        5000
#define COMMAND_POLL_MS        3000

static const char *TAG = "SmartHome";

// ── Firebase root CA ──────────────────────────────────────────────────────────
static const char FIREBASE_ROOT_CA[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIDdTCCAl2gAwIBAgILBAAAAAABFUtaw5QwDQYJKoZIhvcNAQEFBQAwVzELMAkG\n"
    "A1UEBhMCQkUxGTAXBgNVBAoTEEdsb2JhbFNpZ24gbnYtc2ExEDAOBgNVBAsTB1Jv\n"
    "b3QgQ0ExGzAZBgNVBAMTEkdsb2JhbFNpZ24gUm9vdCBDQTAeFw05ODA5MDExMjAw\n"
    "MDBaFw0yODAxMjgxMjAwMDBaMFcxCzAJBgNVBAYTAkJFMRkwFwYDVQQKExBHbG9i\n"
    "YWxTaWduIG52LXNhMRAwDgYDVQQLEwdSb290IENBMRswGQYDVQQDExJHbG9iYWxT\n"
    "aWduIFJvb3QgQ0EwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDaDuaZ\n"
    "jc6j40+Kfvvxi4Mla+pIH/EqsLmVEQS98GPR4mdmzxzdzxtIK+6NiY6arymAZavp\n"
    "xy0Sy6scTHAHoT0KMM0VjU/43dSMUBUc71DuxC73/OlS8pF94G3VNTCOXkNz8kHp\n"
    "1Wrjsok6Vjk4bwY8iGlbKk3Fp1S4bInMm/k8yuX9ifUSPJJ4ltbcdG6TRGHRjcdG\n"
    "snUOhugZitVtbNV4FpWi6cgKOOvyJBNPc1STE4U6G7weNLWLBYy5d4ux2x8gkasJ\n"
    "U26Qzns3dLlwR5EiUWMWea6xrkEmCMgZK9FGqkjWZCrXgzT/LCrBbBlDSgeF59N8\n"
    "9iFo7+ryUp9/k5DPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNVHRMBAf8E\n"
    "BTADAQH/MB0GA1UdDgQWBBRge2YaRQ2XyolQL30EzTSo//z9SzANBgkqhkiG9w0B\n"
    "AQUFAAOCAQEA1nPnfE920I2/7LqivjTFKDK1fPxsnCwrvQmeU79rXqoRSLblCKOz\n"
    "yj1hTdNGCbM+w6DjY1Ub8rrvrTnhQ7k4o+YviiY776BQVvnGCv04zcQLcFGUl5gE\n"
    "38NflNUVyRRBnMRddWQVDf9VMOyGj/8N7yy5Y0b2qvzfvGn9LhJIZJrglfCm7ymP\n"
    "AbEVtQwdpf5pLGkkeB6zpxxxYu7KyJesF12KwvhHhm4qxFYxldBniYUr+WymXUad\n"
    "DKqC5JlR3XC321Y9YeRq4VzW9v493kHMB65jUr9TU/Qr6cf9tveCX4XSQRjbgbME\n"
    "HMUfpIBvFSDJ3gyICh3WZlXi/EjJKSZp4A==\n"
    "-----END CERTIFICATE-----\n";


// ─────────────────────────────────────────────────────────────────────────────
// IR TX via RMT peripheral
// ─────────────────────────────────────────────────────────────────────────────

static rmt_channel_handle_t  ir_tx_channel  = NULL;
static rmt_encoder_handle_t  ir_copy_enc    = NULL;
static rmt_transmit_config_t ir_tx_cfg      = { .loop_count = 0 };

static void ir_tx_init(void)
{
    rmt_tx_channel_config_t ch_cfg = {
        .gpio_num            = IR_TX_GPIO,
        .clk_src             = RMT_CLK_SRC_DEFAULT,
        .resolution_hz       = 1000000,   // 1 µs resolution — matches NEC timing
        .mem_block_symbols   = 64,
        .trans_queue_depth   = 4,
        .flags.invert_out    = false,
        .flags.with_dma      = false,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&ch_cfg, &ir_tx_channel));

    rmt_carrier_config_t carrier = {
        .frequency_hz         = IR_CARRIER_HZ,
        .duty_cycle           = IR_CARRIER_DUTY,
        .flags.polarity_active_low = false,
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(ir_tx_channel, &carrier));

    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &ir_copy_enc));

    ESP_ERROR_CHECK(rmt_enable(ir_tx_channel));
    ESP_LOGI("ir_tx", "RMT IR TX ready on GPIO %d", IR_TX_GPIO);
}

/*
 * ir_ac_build_frame() — build the RMT symbol array for a given temperature.
 *
 * Current implementation: 32-bit NEC frame
 *   Byte 0 : AC_NEC_ADDRESS         (device address)
 *   Byte 1 : ~AC_NEC_ADDRESS        (inverted address)
 *   Byte 2 : command (0x10 + temp - 16)
 *   Byte 3 : ~command               (inverted command)
 *
 * *** REPLACE THE BODY OF THIS FUNCTION with your AC's real frame. ***
 * Capture it with a TSOP4838 receiver + the ESP-IDF ir_rx example as
 * described in the README comment at the top of this file.
 */
static void ir_ac_build_frame(int temp_c, rmt_symbol_word_t *syms, size_t *out_count)
{
    // Clamp temperature to safe range
    if (temp_c < 16) temp_c = 16;
    if (temp_c > 30) temp_c = 30;

    uint8_t addr    = AC_NEC_ADDRESS;
    uint8_t addr_inv = ~addr;
    uint8_t cmd     = 0x10 + (uint8_t)(temp_c - 16);
    uint8_t cmd_inv = ~cmd;

    // Pack 32-bit NEC word: LSB first per byte, address first
    uint32_t nec_word = ((uint32_t)addr)
                      | ((uint32_t)addr_inv << 8)
                      | ((uint32_t)cmd      << 16)
                      | ((uint32_t)cmd_inv  << 24);

    size_t idx = 0;

    // Leading burst (9 ms mark + 4.5 ms space)
    syms[idx].duration0 = NEC_LEADING_MARK;
    syms[idx].level0    = 1;
    syms[idx].duration1 = NEC_LEADING_SPACE;
    syms[idx].level1    = 0;
    idx++;

    // 32 data bits, LSB first
    for (int bit = 0; bit < 32; bit++) {
        syms[idx].duration0 = NEC_BIT_MARK;
        syms[idx].level0    = 1;
        if ((nec_word >> bit) & 1U) {
            syms[idx].duration1 = NEC_ONE_SPACE;
        } else {
            syms[idx].duration1 = NEC_ZERO_SPACE;
        }
        syms[idx].level1 = 0;
        idx++;
    }

    // Trailing mark (burst to end the last bit's space)
    syms[idx].duration0 = NEC_TRAIL_MARK;
    syms[idx].level0    = 1;
    syms[idx].duration1 = 0;   // RMT stops here
    syms[idx].level1    = 0;
    idx++;

    *out_count = idx;
}

static void ir_ac_send(int temp_c)
{
    static rmt_symbol_word_t frame[IR_FRAME_SYMBOLS + 4];
    size_t count = 0;
    ir_ac_build_frame(temp_c, frame, &count);

    esp_err_t err = rmt_transmit(ir_tx_channel, ir_copy_enc,
                                 frame, count * sizeof(rmt_symbol_word_t),
                                 &ir_tx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE("ir_tx", "rmt_transmit failed: %s", esp_err_to_name(err));
        return;
    }
    // Wait for the transmission to finish before returning
    rmt_tx_wait_all_done(ir_tx_channel, pdMS_TO_TICKS(500));
    ESP_LOGI("ir_tx", "AC IR frame sent: %d°C (%zu symbols)", temp_c, count);
}


// ─────────────────────────────────────────────────────────────────────────────
// Relay / atomizer helpers
// ─────────────────────────────────────────────────────────────────────────────

static void relay_set(bool on)
{
    gpio_set_level(RELAY_PIN, on ? 0 : 1);
    ESP_LOGI("relay", "Lamp %s", on ? "ON" : "OFF");
}

static void atomizer_press(void)
{
    gpio_set_level(ATOMIZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(ATOMIZER_PIN, 0);
    ESP_LOGI("atomizer", "button pressed");
}


// ─────────────────────────────────────────────────────────────────────────────
// WiFi
// ─────────────────────────────────────────────────────────────────────────────

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
static int wifi_retry_count = 0;

static volatile bool room_occupied = false;
static int           people_count  = 0;

typedef enum {
    DETECT_IDLE,
    DETECT_OUTER_FIRST,
    DETECT_INNER_FIRST,
    DETECT_AWAIT_PIR,
} detect_state_t;

static detect_state_t detect_state      = DETECT_IDLE;
static TickType_t     state_entered_at  = 0;
static TickType_t     last_detection_at = 0;
static bool outer_broken = false, inner_broken = false;
static int  outer_hi = 0, outer_lo = 0;
static int  inner_hi = 0, inner_lo = 0;


// ─────────────────────────────────────────────────────────────────────────────
// DHT22 bit-bang driver
// ─────────────────────────────────────────────────────────────────────────────

#define _DHT_BIT(p)   (1U << ((p) & 31U))
#define DHT_READ(p)   (((REG_READ(GPIO_IN_REG))  >> (p)) & 1U)
#define DHT_HIGH(p)   REG_WRITE(GPIO_OUT_W1TS_REG,  _DHT_BIT(p))
#define DHT_LOW(p)    REG_WRITE(GPIO_OUT_W1TC_REG,  _DHT_BIT(p))
#define DHT_OUTPUT(p) REG_WRITE(GPIO_ENABLE_W1TS_REG, _DHT_BIT(p))
#define DHT_INPUT(p)  REG_WRITE(GPIO_ENABLE_W1TC_REG, _DHT_BIT(p))

typedef struct { float temperature; float humidity; bool valid; } dht_data_t;

static dht_data_t dht_read(int pin)
{
    dht_data_t result = {0.0f, 0.0f, false};
    uint8_t data[5] = {0};

    DHT_OUTPUT(pin); DHT_LOW(pin);
    vTaskDelay(pdMS_TO_TICKS(20));

    portDISABLE_INTERRUPTS();
    DHT_HIGH(pin); esp_rom_delay_us(40); DHT_INPUT(pin);

    int t = 0;
    while (DHT_READ(pin) == 1) { esp_rom_delay_us(1); if (++t > 200) goto done; }
    t = 0;
    while (DHT_READ(pin) == 0) { esp_rom_delay_us(1); if (++t > 200) goto done; }
    t = 0;
    while (DHT_READ(pin) == 1) { esp_rom_delay_us(1); if (++t > 200) goto done; }

    for (int i = 0; i < 40; i++) {
        t = 0;
        while (DHT_READ(pin) == 0) { esp_rom_delay_us(1); if (++t > 100) goto done; }
        esp_rom_delay_us(35);
        if (DHT_READ(pin) == 1) data[i / 8] |= (1U << (7 - (i % 8)));
        t = 0;
        while (DHT_READ(pin) == 1) { esp_rom_delay_us(1); if (++t > 150) goto done; }
    }

    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) goto done;
    result.humidity    = ((data[0] << 8) | data[1]) * 0.1f;
    result.temperature = (((data[2] & 0x7F) << 8) | data[3]) * 0.1f;
    if (data[2] & 0x80) result.temperature = -result.temperature;
    result.valid = true;

done:
    portENABLE_INTERRUPTS();
    DHT_OUTPUT(pin); DHT_HIGH(pin);
    return result;
}


// ─────────────────────────────────────────────────────────────────────────────
// IR beam receivers (obstacle detection)
// ─────────────────────────────────────────────────────────────────────────────

static void ir_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << IR_OUTER_PIN) | (1ULL << IR_INNER_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

static bool debounce_beam(int gpio, int *hi, int *lo, bool *broken)
{
    int level = gpio_get_level(gpio);
    if (level) { (*hi)++; *lo = 0; }
    else        { (*lo)++; *hi = 0; }
    bool prev = *broken;
    if (!*broken && *hi >= DEBOUNCE_SAMPLES) *broken = true;
    if ( *broken && *lo >= DEBOUNCE_SAMPLES) *broken = false;
    return (!prev && *broken);
}


// ─────────────────────────────────────────────────────────────────────────────
// Firebase HTTP helpers
// ─────────────────────────────────────────────────────────────────────────────

static char http_response_buf[2048];
static int  http_response_len = 0;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        int copy = evt->data_len;
        if (http_response_len + copy >= (int)sizeof(http_response_buf) - 1)
            copy = sizeof(http_response_buf) - 1 - http_response_len;
        if (copy > 0) {
            memcpy(http_response_buf + http_response_len, evt->data, copy);
            http_response_len += copy;
            http_response_buf[http_response_len] = '\0';
        }
    }
    return ESP_OK;
}

static bool firebase_signin_anonymous(void)
{
    char url[128];
    snprintf(url, sizeof(url),
        "https://identitytoolkit.googleapis.com/v1/accounts:signUp?key=%s",
        FIREBASE_API_KEY);

    const char *body = "{\"returnSecureToken\":true}";
    http_response_len = 0; http_response_buf[0] = '\0';

    esp_http_client_config_t cfg = {
        .url           = url, .method = HTTP_METHOD_POST,
        .cert_pem      = FIREBASE_ROOT_CA, .event_handler = http_event_handler,
        .timeout_ms    = 10000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept-Encoding", "identity");
    esp_http_client_set_post_field(client, body, strlen(body));
    esp_err_t err = esp_http_client_perform(client);
    int status    = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "Sign-in failed: %s (HTTP %d)", esp_err_to_name(err), status);
        return false;
    }
    char *p = strstr(http_response_buf, "\"idToken\":");
    if (!p) {
        return false;
    }
    p += strlen("\"idToken\":");
    while (*p == ' ') {
        p++;
    }
    if (*p != '"') {
        return false;
    }
    p++; /* skip opening quote */
    char *end = strchr(p, '"');
    if (!end) {
        return false;
    }
    int len = end - p;
    if (len >= (int)sizeof(fb_id_token)) {
        return false;
    }
    strncpy(fb_id_token, p, len);
    fb_id_token[len] = '\0';
    fb_token_obtained_at = xTaskGetTickCount();
    ESP_LOGI(TAG, "Firebase: anonymous sign-in OK (%d bytes).", len);
    return true;
}

static void firebase_put(const char *path, const char *json)
{
    if ((xTaskGetTickCount() - fb_token_obtained_at) >= pdMS_TO_TICKS(TOKEN_REFRESH_INTERVAL_MS))
        firebase_signin_anonymous();

    static char url[1400];
    snprintf(url, sizeof(url), "https://%s%s?auth=%s", FIREBASE_HOST, path, fb_id_token);

    esp_http_client_config_t cfg = {
        .url = url, .method = HTTP_METHOD_PUT, .cert_pem = FIREBASE_ROOT_CA,
        .event_handler = http_event_handler, .timeout_ms = 8000, .buffer_size_tx = 1200,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json, strlen(json));
    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) ESP_LOGW(TAG, "firebase_put(%s) failed: %s", path, esp_err_to_name(err));
    esp_http_client_cleanup(client);
}

static bool firebase_get(const char *path, char *out_buf, int out_size)
{
    if ((xTaskGetTickCount() - fb_token_obtained_at) >= pdMS_TO_TICKS(TOKEN_REFRESH_INTERVAL_MS))
        firebase_signin_anonymous();

    static char url[1400];
    snprintf(url, sizeof(url), "https://%s%s?auth=%s", FIREBASE_HOST, path, fb_id_token);

    http_response_len = 0; http_response_buf[0] = '\0';

    esp_http_client_config_t cfg = {
        .url = url, .method = HTTP_METHOD_GET, .cert_pem = FIREBASE_ROOT_CA,
        .event_handler = http_event_handler, .timeout_ms = 8000, .buffer_size_tx = 1200,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_err_t err = esp_http_client_perform(client);
    int status    = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status != 200) {
        ESP_LOGW(TAG, "firebase_get(%s) failed (status %d)", path, status);
        return false;
    }
    strncpy(out_buf, http_response_buf, out_size - 1);
    out_buf[out_size - 1] = '\0';
    return true;
}

static void firebase_delete(const char *path)
{
    static char url[1400];
    snprintf(url, sizeof(url), "https://%s%s?auth=%s", FIREBASE_HOST, path, fb_id_token);
    esp_http_client_config_t cfg = {
        .url = url, .method = HTTP_METHOD_DELETE,
        .cert_pem = FIREBASE_ROOT_CA, .timeout_ms = 8000, .buffer_size_tx = 1200,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}


// ─────────────────────────────────────────────────────────────────────────────
// Firebase publish helpers
// ─────────────────────────────────────────────────────────────────────────────

static void pub_temperature(float t) { char b[16]; snprintf(b,sizeof(b),"%.1f",t);  firebase_put(PATH_TEMP,  b); }
static void pub_humidity(float h)    { char b[16]; snprintf(b,sizeof(b),"%.1f",h);  firebase_put(PATH_HUM,   b); }
static void pub_count(int c)         { char b[12]; snprintf(b,sizeof(b),"%d",c);    firebase_put(PATH_COUNT, b); }
static void pub_room(const char *s)  { char b[32]; snprintf(b,sizeof(b),"\"%s\"",s); firebase_put(PATH_ROOM, b); }


// ─────────────────────────────────────────────────────────────────────────────
// Command polling — called from main loop every COMMAND_POLL_MS
// ─────────────────────────────────────────────────────────────────────────────

static void poll_command(void)
{
    char raw[128];
    if (!firebase_get(PATH_COMMAND, raw, sizeof(raw))) return;
    if (strcmp(raw, "null") == 0 || strlen(raw) < 3) return;

    // Strip surrounding JSON quotes
    char cmd[64] = {0};
    int len = strlen(raw);
    if (raw[0] == '"' && raw[len - 1] == '"') {
        strncpy(cmd, raw + 1, len - 2);
        cmd[len - 2] = '\0';
    } else {
        strncpy(cmd, raw, sizeof(cmd) - 1);
    }

    ESP_LOGI(TAG, "Command received: %s", cmd);

    if (strcmp(cmd, "LIGHTS_ON") == 0) {
        relay_set(true);

    } else if (strcmp(cmd, "LIGHTS_OFF") == 0) {
        relay_set(false);

    } else if (strcmp(cmd, "STATUS") == 0) {
        pub_room(room_occupied ? "OCCUPIED" : "EMPTY");
        pub_count(people_count);

    } else if (strncmp(cmd, "AC_SET_TEMP:", 12) == 0) {
        // Parse temperature integer after the colon
        int temp = atoi(cmd + 12);
        if (temp >= 16 && temp <= 30) {
            ESP_LOGI(TAG, "AC temperature command: %d°C", temp);
            ir_ac_send(temp);
        } else {
            ESP_LOGW(TAG, "AC_SET_TEMP out of range: %d", temp);
        }
    }

    firebase_delete(PATH_COMMAND);
}


// ─────────────────────────────────────────────────────────────────────────────
// Directional detection (IR beams + PIR)
// ─────────────────────────────────────────────────────────────────────────────

static void dfplayer_play(uint16_t track);
static void dfplayer_stop(void);

static void detection_poll(void)
{
    TickType_t now = xTaskGetTickCount();
    if ((now - last_detection_at) < pdMS_TO_TICKS(COOLDOWN_MS)) return;

    bool outer_just_broke = debounce_beam(IR_OUTER_PIN, &outer_hi, &outer_lo, &outer_broken);
    bool inner_just_broke = debounce_beam(IR_INNER_PIN, &inner_hi, &inner_lo, &inner_broken);

    switch (detect_state) {

        case DETECT_IDLE:
            if (outer_just_broke) {
                detect_state = DETECT_OUTER_FIRST; state_entered_at = now;
                ESP_LOGI(TAG, "Outer beam broke — watching for inner");
            } else if (inner_just_broke) {
                detect_state = DETECT_INNER_FIRST; state_entered_at = now;
                ESP_LOGI(TAG, "Inner beam broke — watching for outer");
            }
            break;

        case DETECT_OUTER_FIRST:
            if ((now - state_entered_at) >= pdMS_TO_TICKS(SEQUENCE_TIMEOUT_MS)) {
                detect_state = DETECT_IDLE;
            } else if (inner_just_broke) {
                detect_state = DETECT_AWAIT_PIR; state_entered_at = now;
                ESP_LOGI(TAG, "outer→inner — awaiting PIR confirmation");
            }
            break;

        case DETECT_INNER_FIRST:
            if ((now - state_entered_at) >= pdMS_TO_TICKS(SEQUENCE_TIMEOUT_MS)) {
                detect_state = DETECT_IDLE;
            } else if (outer_just_broke) {
                people_count = (people_count > 0) ? people_count - 1 : 0;
                pub_count(people_count);
                if (people_count == 0 && room_occupied) {
                    room_occupied = false;
                    pub_room("EMPTY");
                    relay_set(false);
                    atomizer_press();
                    dfplayer_stop();
                }
                ESP_LOGI(TAG, "<<< EXIT (people: %d)", people_count);
                last_detection_at = now;
                detect_state = DETECT_IDLE;
            }
            break;

        case DETECT_AWAIT_PIR:
            if ((now - state_entered_at) >= pdMS_TO_TICKS(SEQUENCE_TIMEOUT_MS)) {
                ESP_LOGI(TAG, "PIR timeout — entrance not confirmed");
                detect_state = DETECT_IDLE;
            } else if (gpio_get_level(PIR_PIN) == 1) {
                people_count++;
                pub_count(people_count);
                if (!room_occupied) {
                    room_occupied = true;
                    pub_room("OCCUPIED");
                    relay_set(true);
                    atomizer_press();
                    dfplayer_play(1);
                }
                ESP_LOGI(TAG, ">>> ENTRANCE confirmed (people: %d)", people_count);
                last_detection_at = now;
                detect_state = DETECT_IDLE;
            }
            break;
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// WiFi
// ─────────────────────────────────────────────────────────────────────────────

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *event_data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect(); wifi_retry_count++;
            ESP_LOGW(TAG, "WiFi retry %d/%d", wifi_retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi connected. IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t h_any, h_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        wifi_event_handler, NULL, &h_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        wifi_event_handler, NULL, &h_ip));

    wifi_config_t wifi_cfg = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    xEventGroupWaitBits(wifi_event_group,
                        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                        pdFALSE, pdFALSE, portMAX_DELAY);
}


// ─────────────────────────────────────────────────────────────────────────────
// DFPlayer Mini — one-way UART control
// ─────────────────────────────────────────────────────────────────────────────

static void dfplayer_send_cmd(uint8_t cmd, uint16_t param)
{
    uint8_t f[10];
    f[0]=0x7E; f[1]=0xFF; f[2]=0x06; f[3]=cmd; f[4]=0x00;
    f[5]=(param>>8)&0xFF; f[6]=param&0xFF;
    uint16_t sum=0; for(int i=1;i<=6;i++) sum+=f[i];
    uint16_t chk=0-sum; f[7]=(chk>>8)&0xFF; f[8]=chk&0xFF; f[9]=0xEF;
    uart_write_bytes(DFPLAYER_UART, (const char *)f, sizeof(f));
}

static void dfplayer_init(void)
{
    uart_config_t cfg = {
        .baud_rate=DFPLAYER_BAUD, .data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE, .stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, .source_clk=UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(DFPLAYER_UART, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(DFPLAYER_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(DFPLAYER_UART, DFPLAYER_TX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    vTaskDelay(pdMS_TO_TICKS(2000));
    dfplayer_send_cmd(0x06, 20);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI("dfplayer", "init done (UART%d, TX=GPIO%d)", DFPLAYER_UART, DFPLAYER_TX_PIN);
}

static void dfplayer_play(uint16_t track) { dfplayer_send_cmd(0x03, track); ESP_LOGI("dfplayer","play %u",track); }
static void dfplayer_stop(void)           { dfplayer_send_cmd(0x16, 0);     ESP_LOGI("dfplayer","stop"); }


// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────

void app_main(void)
{
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Relay
    gpio_config_t relay_cfg = {
        .pin_bit_mask=(1ULL<<RELAY_PIN), .mode=GPIO_MODE_OUTPUT,
        .pull_up_en=GPIO_PULLUP_DISABLE, .pull_down_en=GPIO_PULLDOWN_DISABLE,
        .intr_type=GPIO_INTR_DISABLE,
    };
    gpio_config(&relay_cfg);
    relay_set(false);

    // Atomizer
    gpio_config_t atomizer_cfg = {
        .pin_bit_mask=(1ULL<<ATOMIZER_PIN), .mode=GPIO_MODE_OUTPUT,
        .pull_up_en=GPIO_PULLUP_DISABLE, .pull_down_en=GPIO_PULLDOWN_ENABLE,
        .intr_type=GPIO_INTR_DISABLE,
    };
    gpio_config(&atomizer_cfg);
    gpio_set_level(ATOMIZER_PIN, 0);

    // PIR
    gpio_config_t pir_cfg = {
        .pin_bit_mask=(1ULL<<PIR_PIN), .mode=GPIO_MODE_INPUT,
        .pull_up_en=GPIO_PULLUP_DISABLE, .pull_down_en=GPIO_PULLDOWN_DISABLE,
        .intr_type=GPIO_INTR_DISABLE,
    };
    gpio_config(&pir_cfg);

    // DHT22
    gpio_set_pull_mode(DHT_PIN, GPIO_PULLUP_ONLY);
    DHT_OUTPUT(DHT_PIN); DHT_HIGH(DHT_PIN);

    // IR obstacle receivers
    ir_init();
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "=== Smart Home — Firebase REST + IR AC ===");
    wifi_init();
    esp_wifi_set_ps(WIFI_PS_NONE);
    firebase_signin_anonymous();

    // IR AC transmitter (must be after WiFi / NVS init)
    ir_tx_init();

    dfplayer_init();
    ESP_LOGI(TAG, "System ready.");

    TickType_t last_dht_tick     = xTaskGetTickCount();
    TickType_t last_command_tick = xTaskGetTickCount();

    while (1) {
        TickType_t now = xTaskGetTickCount();

        detection_poll();

        if ((now - last_dht_tick) >= pdMS_TO_TICKS(DHT_INTERVAL_MS)) {
            last_dht_tick = now;
            dht_data_t dht = dht_read(DHT_PIN);
            if (dht.valid) {
                pub_temperature(dht.temperature);
                pub_humidity(dht.humidity);
                ESP_LOGI(TAG, "Temp: %.1f°C | Hum: %.1f%% | Room: %s | Count: %d",
                         dht.temperature, dht.humidity,
                         room_occupied ? "OCCUPIED" : "EMPTY", people_count);
            } else {
                ESP_LOGW(TAG, "DHT22 read failed (GPIO %d)", DHT_PIN);
            }
        }

        if ((now - last_command_tick) >= pdMS_TO_TICKS(COMMAND_POLL_MS)) {
            last_command_tick = now;
            poll_command();
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}