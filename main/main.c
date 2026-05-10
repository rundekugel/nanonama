/* nanoChat – ESP32-C3
 * - SSD1306 128x64 OLED display (I2C)
 * - MQTT over TLS with user authentication
 * - Morse code input via button → decoded char → published via MQTT
 * - Incoming MQTT messages displayed on OLED (top 7 rows)
 * - Bottom row: morse input in progress
 * - Password-protected web config UI
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wps.h"
#include "mqtt_simple.h"

#include "ssd1306.h"
#include "version.h"

static const char *TAG = "nanoChat";

/* ---- Embedded HTML files ------------------------------------------------- */
extern const unsigned char index_html_start[]  asm("_binary_index_html_start");
extern const unsigned char index_html_end[]    asm("_binary_index_html_end");
extern const unsigned char wifi_html_start[]   asm("_binary_wifi_html_start");
extern const unsigned char wifi_html_end[]     asm("_binary_wifi_html_end");
extern const unsigned char config_html_start[] asm("_binary_config_html_start");
extern const unsigned char config_html_end[]   asm("_binary_config_html_end");
extern const unsigned char help_html_start[]   asm("_binary_help_html_start");
extern const unsigned char help_html_end[]     asm("_binary_help_html_end");

/* =========================================================================
 * Configuration (NVS)
 * ========================================================================= */

#define WIFI_CRED_NS  "wifi_cfg"
#define WIFI_CRED_MAX 10
#define AP_CFG_NS     "ap_cfg"
#define MQTT_CFG_NS   "mqtt_cfg"
#define WEB_CFG_NS    "web_cfg"
#define HW_CFG_NS     "hw_cfg"

/* ---- WiFi credentials ---- */
typedef struct { char ssid[33]; char pass[65]; } wifi_cred_t;
static wifi_cred_t s_wifi_creds[WIFI_CRED_MAX];
static int         s_wifi_cred_count = 0;

/* ---- AP config ---- */
static char s_ap_ssid[33] = "nanoChat";
static char s_ap_pass[65] = "nanochat1";

/* ---- MQTT config ---- */
static char    s_mqtt_host[128]     = "";
static uint16_t s_mqtt_port         = 8883;
static char    s_mqtt_user[64]      = "";
static char    s_mqtt_pass[64]      = "";
static char    s_mqtt_client_id[64] = ""; /* empty = auto (nanoChat-XXXXXXXX) */
static char    s_mqtt_rx_topic[128]    = "chat/rx";
static char    s_mqtt_tx_topic[128]    = "chat/tx";
static char    s_mqtt_state_topic[128] = "chat/state";
static uint8_t s_mqtt_tls          = 1;   /* 1 = use TLS */
static char   *s_mqtt_cert          = NULL; /* PEM cert, heap-allocated */

/* ---- Web UI password ---- */
static char s_web_pass[64] = "admin";

/* ---- Hardware config ---- */
static int s_btn_pin = 9;   /* GPIO9 = boot button on many ESP32-C3 devkits */
static int s_sda_pin = 5;
static int s_scl_pin = 6;
static int s_led_pin        = 8; /* GPIO10 = LED; -1 = disabled */
static int s_disp_timeout_s = 60; /* display off after N seconds idle; 0=never */
static int s_disp_col_off   = 28; /* SSD1306 column offset (default 28 for 72px display) */
static int s_disp_row_off   = 0;  /* display start line offset (0-63) */

/* ---- Runtime state ---- */
static bool s_sta_connected   = false;
static bool s_sta_connecting  = false;
static char s_sta_ip[16]      = "";
static bool s_manual_disc     = false;
static int  s_suppress_disc   = 0;
static TaskHandle_t s_wifi_mgr_task = NULL;
static volatile bool s_scanning  = false;
static volatile bool s_scan_done = false;

/* WPS state: 0=idle 1=active 2=success 3=failed 4=timeout */
static volatile int s_wps_state = 0;

/* mqtt_simple_connected() used for status queries */
static bool s_in_chat_mode = false;  /* false = show status, true = show chat */

/* ---- Display power management ---- */
static volatile bool    s_disp_on       = true;
static volatile int64_t s_last_activity = 0; /* esp_timer_get_time() of last event */

/* Forward declarations (defined in display helpers section below) */
static void disp_lock(void);
static void disp_unlock(void);

static void disp_activity(void)
{
    s_last_activity = esp_timer_get_time();
    if (!s_disp_on) {
        s_disp_on = true;
        disp_lock();
        ssd1306_power(true);
        disp_unlock();
    }
}

/* ---- NVS helpers --------------------------------------------------------- */

static void ap_cfg_load(void) {
    nvs_handle_t h;
    if (nvs_open(AP_CFG_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len;
    len = sizeof(s_ap_ssid); nvs_get_str(h, "ssid", s_ap_ssid, &len);
    len = sizeof(s_ap_pass); nvs_get_str(h, "pass", s_ap_pass, &len);
    nvs_close(h);
}

static void ap_cfg_save(void) {
    nvs_handle_t h;
    if (nvs_open(AP_CFG_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, "ssid", s_ap_ssid);
    nvs_set_str(h, "pass", s_ap_pass);
    nvs_commit(h); nvs_close(h);
}

static void creds_save(void) {
    nvs_handle_t h;
    if (nvs_open(WIFI_CRED_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "count", (uint8_t)s_wifi_cred_count);
    for (int i = 0; i < s_wifi_cred_count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "ssid_%d", i);
        nvs_set_str(h, key, s_wifi_creds[i].ssid);
        snprintf(key, sizeof(key), "pass_%d", i);
        nvs_set_str(h, key, s_wifi_creds[i].pass);
    }
    nvs_commit(h); nvs_close(h);
}

static void creds_load(void) {
    nvs_handle_t h;
    if (nvs_open(WIFI_CRED_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t cnt = 0;
    nvs_get_u8(h, "count", &cnt);
    if (cnt > WIFI_CRED_MAX) cnt = WIFI_CRED_MAX;
    s_wifi_cred_count = cnt;
    for (int i = 0; i < cnt; i++) {
        char key[16];
        size_t len = sizeof(s_wifi_creds[i].ssid);
        snprintf(key, sizeof(key), "ssid_%d", i);
        nvs_get_str(h, key, s_wifi_creds[i].ssid, &len);
        len = sizeof(s_wifi_creds[i].pass);
        snprintf(key, sizeof(key), "pass_%d", i);
        nvs_get_str(h, key, s_wifi_creds[i].pass, &len);
    }
    nvs_close(h);
}

static void creds_add(const char *ssid, const char *pass) {
    for (int i = 0; i < s_wifi_cred_count; i++) {
        if (strcmp(s_wifi_creds[i].ssid, ssid) == 0) {
            strncpy(s_wifi_creds[i].pass, pass, sizeof(s_wifi_creds[i].pass) - 1);
            creds_save(); return;
        }
    }
    if (s_wifi_cred_count == WIFI_CRED_MAX) {
        memmove(&s_wifi_creds[0], &s_wifi_creds[1],
                sizeof(wifi_cred_t) * (WIFI_CRED_MAX - 1));
        s_wifi_cred_count--;
    }
    memset(&s_wifi_creds[s_wifi_cred_count], 0, sizeof(wifi_cred_t));
    strncpy(s_wifi_creds[s_wifi_cred_count].ssid, ssid,
            sizeof(s_wifi_creds[0].ssid) - 1);
    strncpy(s_wifi_creds[s_wifi_cred_count].pass, pass,
            sizeof(s_wifi_creds[0].pass) - 1);
    s_wifi_cred_count++;
    creds_save();
}

static void creds_delete(int idx) {
    if (idx < 0 || idx >= s_wifi_cred_count) return;
    memmove(&s_wifi_creds[idx], &s_wifi_creds[idx + 1],
            sizeof(wifi_cred_t) * (s_wifi_cred_count - idx - 1));
    memset(&s_wifi_creds[s_wifi_cred_count - 1], 0, sizeof(wifi_cred_t));
    s_wifi_cred_count--;
    creds_save();
}

static void creds_move(int idx, int dir) {
    int tgt = idx + dir;
    if (tgt < 0 || tgt >= s_wifi_cred_count) return;
    wifi_cred_t tmp = s_wifi_creds[idx];
    s_wifi_creds[idx] = s_wifi_creds[tgt];
    s_wifi_creds[tgt] = tmp;
    creds_save();
}

static void mqtt_cfg_load(void) {
    nvs_handle_t h;
    if (nvs_open(MQTT_CFG_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len;
    len = sizeof(s_mqtt_host);        nvs_get_str(h, "host",        s_mqtt_host,        &len);
    len = sizeof(s_mqtt_user);        nvs_get_str(h, "user",        s_mqtt_user,        &len);
    len = sizeof(s_mqtt_pass);        nvs_get_str(h, "pass",        s_mqtt_pass,        &len);
    len = sizeof(s_mqtt_client_id);   nvs_get_str(h, "client_id",   s_mqtt_client_id,   &len);
    len = sizeof(s_mqtt_rx_topic);    nvs_get_str(h, "rx_topic",    s_mqtt_rx_topic,    &len);
    len = sizeof(s_mqtt_tx_topic);    nvs_get_str(h, "tx_topic",    s_mqtt_tx_topic,    &len);
    len = sizeof(s_mqtt_state_topic); nvs_get_str(h, "state_topic", s_mqtt_state_topic, &len);
    nvs_get_u16(h, "port", &s_mqtt_port);
    nvs_get_u8 (h, "tls",  &s_mqtt_tls);
    /* Load cert blob */
    size_t cert_len = 0;
    if (nvs_get_blob(h, "cert", NULL, &cert_len) == ESP_OK && cert_len > 0) {
        free(s_mqtt_cert);
        s_mqtt_cert = malloc(cert_len + 1);
        if (s_mqtt_cert) {
            nvs_get_blob(h, "cert", s_mqtt_cert, &cert_len);
            s_mqtt_cert[cert_len] = '\0';
        }
    }
    nvs_close(h);
}

static void mqtt_cfg_save(void) {
    nvs_handle_t h;
    if (nvs_open(MQTT_CFG_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str (h, "host",        s_mqtt_host);
    nvs_set_str (h, "user",        s_mqtt_user);
    nvs_set_str (h, "pass",        s_mqtt_pass);
    nvs_set_str (h, "client_id",   s_mqtt_client_id);
    nvs_set_str (h, "rx_topic",    s_mqtt_rx_topic);
    nvs_set_str (h, "tx_topic",    s_mqtt_tx_topic);
    nvs_set_str (h, "state_topic", s_mqtt_state_topic);
    nvs_set_u16 (h, "port",     s_mqtt_port);
    nvs_set_u8  (h, "tls",      s_mqtt_tls);
    if (s_mqtt_cert && s_mqtt_cert[0])
        nvs_set_blob(h, "cert", s_mqtt_cert, strlen(s_mqtt_cert));
    nvs_commit(h); nvs_close(h);
}

static void web_cfg_load(void) {
    nvs_handle_t h;
    if (nvs_open(WEB_CFG_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len = sizeof(s_web_pass);
    nvs_get_str(h, "pass", s_web_pass, &len);
    nvs_close(h);
}

static void web_cfg_save(void) {
    nvs_handle_t h;
    if (nvs_open(WEB_CFG_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, "pass", s_web_pass);
    nvs_commit(h); nvs_close(h);
}

static void hw_cfg_load(void) {
    nvs_handle_t h;
    if (nvs_open(HW_CFG_NS, NVS_READONLY, &h) != ESP_OK) return;
    int8_t v;
    if (nvs_get_i8(h, "btn", &v) == ESP_OK) s_btn_pin = v;
    if (nvs_get_i8(h, "sda", &v) == ESP_OK) s_sda_pin = v;
    if (nvs_get_i8(h, "scl", &v) == ESP_OK) s_scl_pin = v;
    if (nvs_get_i8(h, "led", &v) == ESP_OK) s_led_pin = v;
    if (nvs_get_i8(h, "dcol", &v) == ESP_OK) s_disp_col_off = v;
    if (nvs_get_i8(h, "drow", &v) == ESP_OK) s_disp_row_off = v;
    int16_t v16;
    if (nvs_get_i16(h, "disp_to", &v16) == ESP_OK) s_disp_timeout_s = v16;
    nvs_close(h);
}

static void hw_cfg_save(void) {
    nvs_handle_t h;
    if (nvs_open(HW_CFG_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i8(h, "btn", (int8_t)s_btn_pin);
    nvs_set_i8(h, "sda", (int8_t)s_sda_pin);
    nvs_set_i8(h, "scl", (int8_t)s_scl_pin);
    nvs_set_i8 (h, "led",     (int8_t) s_led_pin);
    nvs_set_i8 (h, "dcol",    (int8_t) s_disp_col_off);
    nvs_set_i8 (h, "drow",    (int8_t) s_disp_row_off);
    nvs_set_i16(h, "disp_to", (int16_t)s_disp_timeout_s);
    nvs_commit(h); nvs_close(h);
}

/* =========================================================================
 * Display helpers
 * ========================================================================= */

/* Chat history: rows 0-6 hold incoming messages, row 7 holds morse input. */
#define CHAT_ROWS   (OLED_ROWS - 1)   /* 7 */
#define MORSE_ROW   (OLED_ROWS - 1)   /* 7 */

static SemaphoreHandle_t s_disp_mutex;

static void disp_lock(void)   { xSemaphoreTake(s_disp_mutex, portMAX_DELAY); }
static void disp_unlock(void) { xSemaphoreGive(s_disp_mutex); }

/* ---- Incoming text pager ------------------------------------------------- */
#define PAGER_CAPACITY  64

static char s_pager_buf[PAGER_CAPACITY][OLED_COLS + 1];
static int  s_pager_head    = 0;   /* next write index */
static int  s_pager_tail    = 0;   /* next read index */
static int  s_pager_count   = 0;   /* lines buffered */
static int  s_page_row      = 0;   /* next display row on current page */
static bool s_pager_waiting = false; /* display full, waiting for button */

/* Call under disp_lock. Drains buffer onto display rows. */
static void pager_pump_locked(void)
{
    while (s_pager_count > 0 && s_page_row < CHAT_ROWS) {
        ssd1306_clear_row(s_page_row);
        ssd1306_puts(0, s_page_row, s_pager_buf[s_pager_tail]);
        s_pager_tail = (s_pager_tail + 1) % PAGER_CAPACITY;
        s_pager_count--;
        s_page_row++;
    }
    /* Page full and more lines pending: show [more] in morse row */
    if (s_page_row == CHAT_ROWS && s_pager_count > 0) {
        ssd1306_clear_row(MORSE_ROW);
        ssd1306_puts(0, MORSE_ROW, "[MORE]");
        s_pager_waiting = true;
    }
    ssd1306_flush();
}

static void pager_enqueue(const char *line)
{
    if (!s_in_chat_mode) return;
    disp_lock();
    if (s_pager_count >= PAGER_CAPACITY) {
        /* Drop oldest when buffer full */
        s_pager_tail = (s_pager_tail + 1) % PAGER_CAPACITY;
        s_pager_count--;
    }
    strncpy(s_pager_buf[s_pager_head], line, OLED_COLS);
    s_pager_buf[s_pager_head][OLED_COLS] = '\0';
    s_pager_head = (s_pager_head + 1) % PAGER_CAPACITY;
    s_pager_count++;
    if (!s_pager_waiting)
        pager_pump_locked();
    disp_unlock();
}

/* Called when button pressed while display is waiting. */
static void pager_advance(void)
{
    disp_lock();
    s_pager_waiting = false;
    s_page_row = 0;
    ssd1306_clear_row(MORSE_ROW);
    for (int r = 0; r < CHAT_ROWS; r++)
        ssd1306_clear_row(r);
    pager_pump_locked();
    disp_unlock();
}

/* Reset pager state and clear chat area (e.g. on new MQTT connection). */
static void pager_clear(void)
{
    disp_lock();
    s_pager_head    = 0;
    s_pager_tail    = 0;
    s_pager_count   = 0;
    s_page_row      = 0;
    s_pager_waiting = false;
    for (int r = 0; r < CHAT_ROWS; r++)
        ssd1306_clear_row(r);
    ssd1306_clear_row(MORSE_ROW);
    ssd1306_flush();
    disp_unlock();
}

/* Update the morse input row. buf = current morse chars decoded so far. */
static void morse_row_update(const char *buf)
{
    char line[OLED_COLS + 1];
    snprintf(line, sizeof(line), ">%.*s", OLED_COLS - 1, buf);
    int l = strlen(line);
    while (l < OLED_COLS) line[l++] = ' ';
    line[OLED_COLS] = '\0';

    disp_lock();
    ssd1306_clear_row(MORSE_ROW);
    ssd1306_puts(0, MORSE_ROW, line);
    ssd1306_flush();
    disp_unlock();
}

/* Show connection status on rows 0..CHAT_ROWS-1. Adapts to any display size. */
static void status_display(void)
{
    char line[OLED_COLS + 1];
    int r = 0;
    disp_lock();

    /* Row 0: title (+ version if wide enough) */
    ssd1306_clear_row(r);
    snprintf(line, sizeof(line), "nanoChat %.*s", OLED_COLS - 9, VERSION);
    ssd1306_puts(0, r++, line);

    /* Row 1: WiFi status */
    if (r < CHAT_ROWS) {
        ssd1306_clear_row(r);
        if (s_sta_connected)       ssd1306_puts(0, r, "WiFi:OK");
        else if (s_sta_connecting) ssd1306_puts(0, r, "WiFi:conn");
        else                       ssd1306_puts(0, r, "WiFi:off");
        r++;
    }

    /* Row 2 (wide displays only): IP address */
    if (CHAT_ROWS >= 6 && r < CHAT_ROWS) {
        ssd1306_clear_row(r);
        snprintf(line, sizeof(line), "IP:%.*s",
                 OLED_COLS - 3, s_sta_ip[0] ? s_sta_ip : "--");
        ssd1306_puts(0, r++, line);
    }

    /* MQTT status */
    if (r < CHAT_ROWS) {
        ssd1306_clear_row(r);
        if (mqtt_simple_connected()) ssd1306_puts(0, r, "MQTT:up");
        else if (s_mqtt_host[0])     ssd1306_puts(0, r, "MQTT:off");
        else                         ssd1306_puts(0, r, "MQTT:n/a");
        r++;
    }

    /* Broker host (only if room) */
    if (CHAT_ROWS >= 5 && r < CHAT_ROWS && s_mqtt_host[0]) {
        ssd1306_clear_row(r);
        snprintf(line, sizeof(line), "%.*s", OLED_COLS, s_mqtt_host);
        ssd1306_puts(0, r++, line);
    }

    /* AP SSID */
    if (r < CHAT_ROWS) {
        ssd1306_clear_row(r);
        snprintf(line, sizeof(line), "AP:%.*s", OLED_COLS - 3, s_ap_ssid);
        ssd1306_puts(0, r++, line);
    }

    /* IP / AP-IP (remaining row) */
    if (r < CHAT_ROWS) {
        ssd1306_clear_row(r);
        snprintf(line, sizeof(line), "%.*s",
                 OLED_COLS, s_sta_ip[0] ? s_sta_ip : "192.168.4.1");
        ssd1306_puts(0, r++, line);
    }

    /* Clear any leftover rows */
    while (r < CHAT_ROWS) ssd1306_clear_row(r++);

    ssd1306_flush();
    disp_unlock();
}

/* =========================================================================
 * Morse decoder
 * ========================================================================= */

#define MORSE_MAX_ELEM  7   /* max dots/dashes per character */
#define MORSE_QUEUE_DEPTH 16

typedef struct {
    bool    press;          /* true=button down, false=button up */
    int64_t time_us;        /* esp_timer_get_time() at event */
} morse_evt_t;

static QueueHandle_t s_morse_queue;
static volatile int  s_btn_pin_val;

static void IRAM_ATTR morse_isr(void *arg)
{
    morse_evt_t evt = {
        .press   = (gpio_get_level(s_btn_pin_val) == 0), /* active-low */
        .time_us = esp_timer_get_time(),
    };
    xQueueSendFromISR(s_morse_queue, &evt, NULL);
}

static char decode_morse(const char *pattern)
{
    static const struct { const char *c; char ch; } tab[] = {
        {".-",   'A'}, {"-...", 'B'}, {"-.-.", 'C'}, {"-..",  'D'},
        {".",    'E'}, {"..-.", 'F'}, {"--.",  'G'}, {"....", 'H'},
        {"..",   'I'}, {".---", 'J'}, {"-.-",  'K'}, {".-..", 'L'},
        {"--",   'M'}, {"-.",   'N'}, {"---",  'O'}, {".--.", 'P'},
        {"--.-", 'Q'}, {".-.",  'R'}, {"...",  'S'}, {"-",    'T'},
        {"..-",  'U'}, {"...-", 'V'}, {".--",  'W'}, {"-..-", 'X'},
        {"-.--", 'Y'}, {"--..", 'Z'},
        {"-----",'0'}, {".----",'1'}, {"..---",'2'}, {"...--",'3'},
        {"....-",'4'}, {".....", '5'}, {"-....", '6'}, {"--...", '7'},
        {"---..", '8'}, {"----.", '9'}, {NULL, 0}
    };
    for (int i = 0; tab[i].c; i++)
        if (strcmp(pattern, tab[i].c) == 0) return tab[i].ch;
    return '?';
}

/* Forward declaration */
static void pager_advance(void);

static void morse_task(void *arg)
{
    (void)arg;
    morse_evt_t evt;
    int64_t press_start  = 0;
    int64_t last_release = 0;
    bool    in_press     = false;

    char    elem_buf[MORSE_MAX_ELEM + 1] = "";  /* dots/dashes for current char */
    int     elem_pos = 0;
    char    decoded_buf[64] = "";               /* decoded chars for display row */
    int     decoded_pos = 0;
    char    word_buf[64] = "";                  /* chars of current word for TX */
    int     word_pos = 0;

    int64_t dot_unit_us = 150000LL;  /* initial estimate: 150 ms dot */

    for (;;) {
        /* Wait up to 4× dot_unit for the next event (letter-gap detection) */
        TickType_t wait_ticks = pdMS_TO_TICKS((uint32_t)(dot_unit_us * 4 / 1000));
        if (wait_ticks < 2) wait_ticks = 2;

        BaseType_t got = xQueueReceive(s_morse_queue, &evt, wait_ticks);

        if (got == pdFALSE) {
            /* Timeout: check how long since last release */
            if (!in_press && elem_pos > 0) {
                int64_t gap_us = esp_timer_get_time() - last_release;
                if (gap_us >= dot_unit_us * 3) {
                    /* Letter gap – decode accumulated element */
                    elem_buf[elem_pos] = '\0';
                    char ch = decode_morse(elem_buf);
                    if (ch != '?' || elem_pos > 0) {
                        if (decoded_pos < (int)sizeof(decoded_buf) - 1) {
                            decoded_buf[decoded_pos++] = ch;
                            decoded_buf[decoded_pos] = '\0';
                        }
                        /* Accumulate into word buffer */
                        if (word_pos < (int)sizeof(word_buf) - 1) {
                            word_buf[word_pos++] = ch;
                            word_buf[word_pos] = '\0';
                        }
                        /* Publish single char on tx_topic/1 */
                        if (mqtt_simple_connected()) {
                            char ctopic[136];
                            snprintf(ctopic, sizeof(ctopic),
                                     "%s/1", s_mqtt_tx_topic);
                            char cbuf[2] = {ch, '\0'};
                            ESP_LOGI(TAG, "MQTT TX char [%s] '%c'",
                                     ctopic, ch);
                            mqtt_simple_publish(ctopic, cbuf, 1);
                        }
                        morse_row_update(decoded_buf);
                    }
                    elem_pos = 0;
                    elem_buf[0] = '\0';

                    /* Word gap: publish word on tx_topic/word */
                    if (gap_us >= dot_unit_us * 7 && word_pos > 0) {
                        char wtopic[136];
                        snprintf(wtopic, sizeof(wtopic),
                                 "%s/word", s_mqtt_tx_topic);
                        ESP_LOGI(TAG, "MQTT TX word [%s] '%s'",
                                 wtopic, word_buf);
                        mqtt_simple_publish(wtopic, word_buf, word_pos);
                        word_pos = 0;
                        word_buf[0] = '\0';
                        if (decoded_pos < (int)sizeof(decoded_buf) - 1) {
                            decoded_buf[decoded_pos++] = ' ';
                            decoded_buf[decoded_pos] = '\0';
                        }
                        morse_row_update(decoded_buf);
                    }
                }
            }
            /* After a long silence, flush any pending word and clear display */
            if (!in_press && decoded_pos > 0) {
                int64_t silence_us = esp_timer_get_time() - last_release;
                if (silence_us > dot_unit_us * 20) {
                    if (word_pos > 0) {
                        char wtopic[136];
                        snprintf(wtopic, sizeof(wtopic),
                                 "%s/word", s_mqtt_tx_topic);
                        ESP_LOGI(TAG, "MQTT TX word [%s] '%s'",
                                 wtopic, word_buf);
                        mqtt_simple_publish(wtopic, word_buf, word_pos);
                        word_pos = 0;
                        word_buf[0] = '\0';
                    }
                    decoded_pos = 0;
                    decoded_buf[0] = '\0';
                    morse_row_update("");
                }
            }
            continue;
        }

        if (evt.press) {
            /* Button pressed */
            if (!in_press) {
                press_start = evt.time_us;
                in_press = true;
            }
        } else {
            /* Button released */
            if (!in_press) continue;
            in_press = false;

            int64_t dur_us = evt.time_us - press_start;
            last_release = evt.time_us;

            if (dur_us < 20000LL) continue; /* debounce: ignore < 20 ms */

            /* Wake display on any button press */
            if (!s_disp_on) {
                disp_activity();
                continue; /* consume press, don't process as morse */
            }
            disp_activity();

            /* Very long press (>5 s): start WPS */
            if (dur_us >= 5000000LL) {
                if (s_wps_state != 1) {
                    esp_wps_config_t wps_cfg = WPS_CONFIG_INIT_DEFAULT(WPS_TYPE_PBC);
                    s_wps_state = 1;
                    esp_wifi_wps_enable(&wps_cfg);
                    esp_wifi_wps_start();
                    disp_lock();
                    ssd1306_clear();
                    ssd1306_puts(0, 2, "WPS...");
                    ssd1306_puts(0, 3, "PRESS ROUTER");
                    ssd1306_flush();
                    disp_unlock();
                }
                continue;
            }

            /* Long press (>3 s): turn display off */
            if (dur_us >= 3000000LL) {
                s_disp_on = false;
                disp_lock();
                ssd1306_power(false);
                disp_unlock();
                continue;
            }

            /* Pager: any shorter press advances page when display is full */
            if (s_pager_waiting) {
                pager_advance();
                continue;
            }

            /* Classify: dot or dash */
            if (dur_us < dot_unit_us * 2) {
                /* Dot */
                if (elem_pos < MORSE_MAX_ELEM)
                    elem_buf[elem_pos++] = '.';
                /* Update dot_unit (EMA) */
                dot_unit_us = (dot_unit_us * 7 + dur_us * 3) / 10;
            } else {
                /* Dash */
                if (elem_pos < MORSE_MAX_ELEM)
                    elem_buf[elem_pos++] = '-';
                /* Dash should be ~3× dot, update estimate */
                int64_t measured_dot = dur_us / 3;
                if (measured_dot > 10000LL && measured_dot < 2000000LL)
                    dot_unit_us = (dot_unit_us * 7 + measured_dot * 3) / 10;
            }
            elem_buf[elem_pos] = '\0';

            /* Show progress on display */
            {
                char tmp[72]; /* decoded_buf(64) + elem_buf(8) */
                snprintf(tmp, sizeof(tmp), "%s%s", decoded_buf, elem_buf);
                morse_row_update(tmp);
            }
        }
    }
}

static void led_disp_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* LED: flash while unread messages are waiting */
        if (s_led_pin >= 0 && s_pager_waiting) {
            gpio_set_level(s_led_pin, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(s_led_pin, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            if (s_led_pin >= 0) gpio_set_level(s_led_pin, 1);

            /* Display auto-off when idle */
            if (s_disp_on && s_disp_timeout_s > 0) {
                int64_t idle_us = esp_timer_get_time() - s_last_activity;
                if (idle_us >= (int64_t)s_disp_timeout_s * 1000000LL) {
                    s_disp_on = false;
                    disp_lock();
                    ssd1306_power(false);
                    disp_unlock();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

static void led_init(void)
{
    s_last_activity = esp_timer_get_time();
    if (s_led_pin >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << s_led_pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        gpio_set_level(s_led_pin, 1);
    }
    xTaskCreate(led_disp_task, "led_disp", 1024, NULL, 3, NULL);
}

static void morse_init(void)
{
    s_btn_pin_val = s_btn_pin;
    s_morse_queue = xQueueCreate(MORSE_QUEUE_DEPTH, sizeof(morse_evt_t));

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s_btn_pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(s_btn_pin, morse_isr, NULL);

    xTaskCreate(morse_task, "morse", 4096, NULL, 5, NULL);
}

/* =========================================================================
 * MQTT client
 * ========================================================================= */


static void on_mqtt_rx(const char *topic, int topic_len,
                       const char *data,  int data_len)
{
    ESP_LOGI(TAG, "MQTT RX [%.*s] %.*s", topic_len, topic, data_len, data);
    disp_activity();
    /* Split into OLED_COLS-char lines and enqueue for paged display */
    char line[OLED_COLS + 1];
    int off = 0;
    while (off < data_len) {
        int chunk = data_len - off;
        if (chunk > OLED_COLS) chunk = OLED_COLS;
        memcpy(line, data + off, chunk);
        line[chunk] = '\0';
        pager_enqueue(line);
        off += chunk;
    }
}

static void on_mqtt_connected(void)
{
    ESP_LOGI(TAG, "MQTT connected");
    mqtt_simple_subscribe(s_mqtt_rx_topic);
    /* Publish retained "online" state */
    if (s_mqtt_state_topic[0])
        mqtt_simple_publish_retained(s_mqtt_state_topic, "online", 6);
    /* Switch display to chat mode */
    s_in_chat_mode = true;
    pager_clear();
}

static void on_mqtt_disconnected(void)
{
    ESP_LOGI(TAG, "MQTT disconnected");
    s_in_chat_mode = false;
    status_display();
}

static void mqtt_start(void)
{
    if (!s_mqtt_host[0]) {
        ESP_LOGW(TAG, "MQTT host not configured");
        return;
    }
    mqtt_simple_stop(); /* stop any running instance */

    /* Build effective client_id: use configured value or derive from MAC hash */
    static char s_effective_client_id[64];
    if (s_mqtt_client_id[0]) {
        strncpy(s_effective_client_id, s_mqtt_client_id, sizeof(s_effective_client_id) - 1);
        s_effective_client_id[sizeof(s_effective_client_id) - 1] = '\0';
    } else {
        uint8_t mac[6] = {0};
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        uint32_t h = 2166136261u; /* FNV-1a 32-bit */
        for (int i = 0; i < 6; i++) { h ^= mac[i]; h *= 16777619u; }
        snprintf(s_effective_client_id, sizeof(s_effective_client_id),
                 "nanoChat-%08X", (unsigned)h);
    }

    mqtt_simple_cfg_t cfg = {
        .host            = s_mqtt_host,
        .port            = s_mqtt_port,
        .username        = s_mqtt_user[0]            ? s_mqtt_user : NULL,
        .password        = s_mqtt_pass[0]            ? s_mqtt_pass : NULL,
        .ca_cert_pem     = (s_mqtt_cert && s_mqtt_cert[0]) ? s_mqtt_cert : NULL,
        .client_id       = s_effective_client_id,
        .use_tls         = s_mqtt_tls != 0,
        .keepalive_sec   = 60,
        .will_topic      = s_mqtt_state_topic[0] ? s_mqtt_state_topic : NULL,
        .will_payload    = "offline",
        .will_retain     = true,
        .rx_cb           = on_mqtt_rx,
        .connected_cb    = on_mqtt_connected,
        .disconnected_cb = on_mqtt_disconnected,
    };
    mqtt_simple_start(&cfg);
}

/* =========================================================================
 * WiFi (AP+STA, same pattern as admkbd.c)
 * ========================================================================= */

#define WIFI_RETRY_MS    30000
#define WIFI_SCAN_TMO_MS  5000

static void wifi_do_connect(int idx)
{
    if (idx < 0 || idx >= s_wifi_cred_count) return;
    wifi_config_t cfg = {};
    strncpy((char*)cfg.sta.ssid,     s_wifi_creds[idx].ssid,
            sizeof(cfg.sta.ssid) - 1);
    strncpy((char*)cfg.sta.password, s_wifi_creds[idx].pass,
            sizeof(cfg.sta.password) - 1);
    if (s_sta_connected || s_sta_connecting) {
        s_suppress_disc++;
        esp_wifi_disconnect();
    }
    s_sta_connected   = false;
    s_sta_connecting  = true;
    s_sta_ip[0]       = 0;
    s_manual_disc     = false;
    esp_wifi_set_config(WIFI_IF_STA, &cfg);
    esp_wifi_connect();
    ESP_LOGI(TAG, "Connecting to %s", s_wifi_creds[idx].ssid);
}

static int wifi_pick_best(void)
{
    if (s_wifi_cred_count == 0) return -1;
    uint16_t n = 24;
    wifi_ap_record_t *aps = malloc(n * sizeof(wifi_ap_record_t));
    if (!aps) return 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n > 24) n = 24;
    esp_wifi_scan_get_ap_records(&n, aps);
    int best = -1; int8_t best_rssi = -127;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < s_wifi_cred_count; j++) {
            if (strcmp((char*)aps[i].ssid, s_wifi_creds[j].ssid) == 0) {
                if (aps[i].rssi > best_rssi + 5 ||
                    (aps[i].rssi >= best_rssi - 5 && j < best) ||
                    best < 0) {
                    best_rssi = aps[i].rssi; best = j;
                }
                break;
            }
        }
    }
    free(aps);
    return best;
}

static void wifi_scan_async(void)
{
    wifi_scan_config_t sc = {
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };
    s_scanning = true; s_scan_done = false;
    if (esp_wifi_scan_start(&sc, false) != ESP_OK) s_scanning = false;
}

static void wifi_mgr_task(void *arg)
{
    (void)arg;
    bool first = true;
    for (;;) {
        uint32_t block = s_scanning ? WIFI_SCAN_TMO_MS : WIFI_RETRY_MS;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(block));

        if (s_manual_disc || s_wifi_cred_count == 0) {
            first = false; s_scan_done = false; s_scanning = false; continue;
        }
        if (s_scanning && !s_scan_done) {
            s_scanning = false;
        }
        if (s_scan_done) {
            s_scan_done = false;
            if (!s_sta_connected && !s_manual_disc) {
                int idx = wifi_pick_best();
                if (idx >= 0) wifi_do_connect(idx);
            }
            first = false; continue;
        }
        if (s_sta_connected || s_sta_connecting) { first = false; continue; }
        if (!first) vTaskDelay(pdMS_TO_TICKS(1500));
        first = false;
        if (s_sta_connected || s_manual_disc) continue;
        if (s_wifi_cred_count == 1) {
            wifi_do_connect(0);
        } else {
            wifi_scan_async();
            if (!s_scanning) wifi_do_connect(0);
        }
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        esp_ip4addr_ntoa(&ev->ip_info.ip, s_sta_ip, sizeof(s_sta_ip));
        s_sta_connecting = false;
        s_sta_connected  = true;
        ESP_LOGI(TAG, "STA IP: %s", s_sta_ip);
        if (!s_in_chat_mode) status_display();
        /* Start/restart MQTT after WiFi is up */
        mqtt_start();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_SCAN_DONE) {
        s_scanning = false; s_scan_done = true;
        if (s_wifi_mgr_task) xTaskNotifyGive(s_wifi_mgr_task);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_sta_connected = false; s_sta_connecting = false; s_sta_ip[0] = 0;
        if (s_suppress_disc > 0) { s_suppress_disc--; return; }
        if (s_manual_disc)       { s_manual_disc = false; return; }
        if (!s_in_chat_mode) status_display();
        if (s_wifi_mgr_task) xTaskNotifyGive(s_wifi_mgr_task);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_WPS_ER_SUCCESS) {
        esp_wifi_wps_disable();
        wifi_event_sta_wps_er_success_t *ev =
            (wifi_event_sta_wps_er_success_t *)data;
        if (ev && ev->ap_cred_cnt > 0) {
            const char *ssid = (const char *)ev->ap_cred[0].ssid;
            const char *pass = (const char *)ev->ap_cred[0].passphrase;
            creds_add(ssid, pass);
            creds_save();
            wifi_do_connect(s_wifi_cred_count - 1);
        }
        s_wps_state = 2; /* success */
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_WPS_ER_FAILED) {
        esp_wifi_wps_disable();
        s_wps_state = 3; /* failed */
        if (s_wifi_mgr_task) xTaskNotifyGive(s_wifi_mgr_task);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_WPS_ER_TIMEOUT) {
        esp_wifi_wps_disable();
        s_wps_state = 4; /* timeout */
        if (s_wifi_mgr_task) xTaskNotifyGive(s_wifi_mgr_task);
    }
}

static void wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                wifi_event_handler, NULL);

    wifi_config_t ap_cfg = {};
    strncpy((char*)ap_cfg.ap.ssid,     s_ap_ssid, sizeof(ap_cfg.ap.ssid) - 1);
    strncpy((char*)ap_cfg.ap.password, s_ap_pass,  sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode = (strlen(s_ap_pass) >= 8) ?
                          WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;

    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    esp_wifi_start();

    creds_load();
    xTaskCreate(wifi_mgr_task, "wifi_mgr", 4096, NULL, 4, &s_wifi_mgr_task);
    if (s_wifi_cred_count > 0) xTaskNotifyGive(s_wifi_mgr_task);
}

/* =========================================================================
 * HTTP web server – helpers
 * ========================================================================= */

static int recv_body(httpd_req_t *req, char *buf, size_t sz)
{
    int total = 0, rem = (int)req->content_len;
    while (rem > 0 && total < (int)sz - 1) {
        int n = httpd_req_recv(req, buf + total,
                               (size_t)(rem < (int)(sz - total - 1) ?
                                        rem : (int)(sz - total - 1)));
        if (n <= 0) break;
        total += n; rem -= n;
    }
    buf[total] = '\0';
    return total;
}

static esp_err_t send_html(httpd_req_t *req,
                            const uint8_t *start, const uint8_t *end)
{
    size_t len = end - start;
    char *buf = malloc(len + 1);
    if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }
    memcpy(buf, start, len); buf[len] = '\0';
    /* Replace %%VERSION%% */
    const char *ph = "%%VERSION%%";
    size_t ph_len = strlen(ph), ver_len = strlen(VERSION);
    char *p;
    while ((p = strstr(buf, ph)) != NULL) {
        memmove(p + ver_len, p + ph_len, strlen(p + ph_len) + 1);
        memcpy(p, VERSION, ver_len);
    }
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, buf, strlen(buf));
    free(buf);
    return ESP_OK;
}

/* ---- Minimal base64 decoder (no external dependency) ---- */
static int b64decode(const char *in, char *out, size_t out_sz)
{
    static const char tbl[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int j = 0;
    for (int i = 0; in[i] && in[i] != '=' && j < (int)out_sz - 1; i += 4) {
        uint8_t v[4] = {0};
        for (int k = 0; k < 4 && in[i + k] && in[i + k] != '='; k++) {
            const char *pt = strchr(tbl, in[i + k]);
            v[k] = pt ? (uint8_t)(pt - tbl) : 0;
        }
        if (j < (int)out_sz - 1) out[j++] = (char)((v[0] << 2) | (v[1] >> 4));
        if (in[i+2] != '=' && j < (int)out_sz - 1)
            out[j++] = (char)((v[1] << 4) | (v[2] >> 2));
        if (in[i+3] != '=' && j < (int)out_sz - 1)
            out[j++] = (char)((v[2] << 6) | v[3]);
    }
    out[j] = '\0';
    return j;
}

static void send_401(httpd_req_t *req)
{
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"nanoChat\"");
    httpd_resp_sendstr(req, "Unauthorized");
}

static bool check_auth(httpd_req_t *req)
{
    char hdr[160];
    if (httpd_req_get_hdr_value_str(req, "Authorization",
                                    hdr, sizeof(hdr)) != ESP_OK) {
        send_401(req); return false;
    }
    if (strncmp(hdr, "Basic ", 6) != 0) { send_401(req); return false; }
    char decoded[128];
    b64decode(hdr + 6, decoded, sizeof(decoded));
    char expected[72];
    snprintf(expected, sizeof(expected), "admin:%s", s_web_pass);
    if (strcmp(decoded, expected) != 0) { send_401(req); return false; }
    return true;
}

static void schedule_restart(void)
{
    /* Delay restart to let HTTP response flush */
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
}

/* =========================================================================
 * HTTP handlers
 * ========================================================================= */

static esp_err_t h_root(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    return send_html(req, index_html_start, index_html_end);
}

static esp_err_t h_status(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    char out[256];
    snprintf(out, sizeof(out),
             "{\"sta\":%s,\"ip\":\"%s\",\"mqtt\":%s,\"ap\":\"%s\","
             "\"ver\":\"%s\"}",
             s_sta_connected ? "true" : "false",
             s_sta_ip,
             mqtt_simple_connected() ? "true" : "false",
             s_ap_ssid,
             VERSION);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

/* ---- WiFi page ---- */
static esp_err_t h_wifi(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    return send_html(req, wifi_html_start, wifi_html_end);
}

static esp_err_t h_wifi_connect(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    char buf[128]; recv_body(req, buf, sizeof(buf));
    char *sep = strchr(buf, '|');
    if (!sep) { httpd_resp_sendstr(req, "ERR"); return ESP_OK; }
    *sep = '\0';
    creds_add(buf, sep + 1);
    int idx = 0;
    for (int i = 0; i < s_wifi_cred_count; i++)
        if (strcmp(s_wifi_creds[i].ssid, buf) == 0) { idx = i; break; }
    wifi_do_connect(idx);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t h_wifi_disconnect(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    s_manual_disc = true; s_sta_connected = false; s_sta_ip[0] = 0;
    esp_wifi_disconnect();
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t h_wifi_list(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    char out[1024]; int pos = 0;
    pos += snprintf(out + pos, sizeof(out) - pos, "[");
    for (int i = 0; i < s_wifi_cred_count; i++) {
        if (i) pos += snprintf(out + pos, sizeof(out) - pos, ",");
        pos += snprintf(out + pos, sizeof(out) - pos,
                        "{\"idx\":%d,\"ssid\":\"%s\"}", i, s_wifi_creds[i].ssid);
    }
    pos += snprintf(out + pos, sizeof(out) - pos, "]");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static esp_err_t h_wifi_delete(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    char buf[8]; recv_body(req, buf, sizeof(buf));
    creds_delete(atoi(buf));
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t h_wifi_connect_idx(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    char buf[8]; recv_body(req, buf, sizeof(buf));
    wifi_do_connect(atoi(buf));
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t h_wifi_move(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    char buf[16]; recv_body(req, buf, sizeof(buf));
    char *sp, *is = strtok_r(buf, "|", &sp), *ds = strtok_r(NULL, "|", &sp);
    if (!is || !ds) { httpd_resp_sendstr(req, "ERR"); return ESP_OK; }
    creds_move(atoi(is), atoi(ds));
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t h_wifi_scan(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    wifi_scan_config_t sc = {
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };
    esp_wifi_scan_start(&sc, true);
    uint16_t count = 20;
    wifi_ap_record_t *recs = malloc(count * sizeof(wifi_ap_record_t));
    if (!recs) { httpd_resp_sendstr(req, "[]"); return ESP_OK; }
    esp_wifi_scan_get_ap_records(&count, recs);
    char *out = malloc(2048); int pos = 0;
    if (!out) { free(recs); httpd_resp_sendstr(req, "[]"); return ESP_OK; }
    pos += snprintf(out + pos, 2048 - pos, "[");
    for (int i = 0; i < count; i++) {
        if (i) pos += snprintf(out + pos, 2048 - pos, ",");
        char safe[67] = {0}; int si = 0;
        for (int j = 0; recs[i].ssid[j] && si < 64; j++) {
            if (recs[i].ssid[j] == '"') safe[si++] = '\\';
            safe[si++] = recs[i].ssid[j];
        }
        pos += snprintf(out + pos, 2048 - pos,
                        "{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":%d}",
                        safe, recs[i].rssi, recs[i].authmode);
    }
    pos += snprintf(out + pos, 2048 - pos, "]");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    free(out); free(recs);
    return ESP_OK;
}

static esp_err_t h_ap_get(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    char out[128];
    snprintf(out, sizeof(out), "{\"ssid\":\"%s\",\"pass\":\"%s\"}",
             s_ap_ssid, s_ap_pass);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static esp_err_t h_ap_set(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    char buf[128]; recv_body(req, buf, sizeof(buf));
    char *sp;
    char *ssid = strtok_r(buf, "|", &sp);
    char *pass = strtok_r(NULL, "|", &sp);
    if (ssid) strncpy(s_ap_ssid, ssid, sizeof(s_ap_ssid) - 1);
    if (pass) strncpy(s_ap_pass, pass, sizeof(s_ap_pass) - 1);
    ap_cfg_save();
    httpd_resp_sendstr(req, "OK");
    xTaskCreate((void*)schedule_restart, "restart", 1024, NULL, 3, NULL);
    return ESP_OK;
}

/* ---- Reboot ---- */
static esp_err_t h_reboot(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    httpd_resp_sendstr(req, "OK");
    xTaskCreate((void*)schedule_restart, "restart", 1024, NULL, 3, NULL);
    return ESP_OK;
}

/* ---- Config page ---- */
static esp_err_t h_config(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    return send_html(req, config_html_start, config_html_end);
}

/* ---- Help page ---- */
static esp_err_t h_help(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    return send_html(req, help_html_start, help_html_end);
}

static esp_err_t h_config_get(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    char out[960];
    snprintf(out, sizeof(out),
             "{\"host\":\"%s\",\"port\":%d,\"user\":\"%s\","
             "\"rx_topic\":\"%s\",\"tx_topic\":\"%s\",\"state_topic\":\"%s\","
             "\"tls\":%d,\"has_cert\":%d,\"has_pass\":%d,"
             "\"client_id\":\"%s\","
             "\"btn\":%d,\"sda\":%d,\"scl\":%d,"
             "\"led\":%d,\"disp_to\":%d,"
             "\"disp_col_off\":%d,\"disp_row_off\":%d}",
             s_mqtt_host, (int)s_mqtt_port, s_mqtt_user,
             s_mqtt_rx_topic, s_mqtt_tx_topic, s_mqtt_state_topic,
             (int)s_mqtt_tls, (s_mqtt_cert && s_mqtt_cert[0]) ? 1 : 0,
             s_mqtt_pass[0] ? 1 : 0,
             s_mqtt_client_id,
             s_btn_pin, s_sda_pin, s_scl_pin,
             s_led_pin, s_disp_timeout_s,
             s_disp_col_off, s_disp_row_off);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static esp_err_t h_config_set(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    /* Body format (pipe-delimited):
     * host|port|user|pass|rx|tx|state|tls|btn|sda|scl|led|disp_to|dcol|drow|web_pass|client_id|cert */
    size_t body_sz = req->content_len + 1;
    if (body_sz > 8192) body_sz = 8192;
    char *buf = malloc(body_sz);
    if (!buf) { httpd_resp_send_500(req); return ESP_OK; }
    recv_body(req, buf, body_sz);

    /* strsep (not strtok_r) so empty fields like blank password are preserved */
    char *sp        = buf;
    char *host      = strsep(&sp, "|");
    char *port_s    = strsep(&sp, "|");
    char *user      = strsep(&sp, "|");
    char *pass      = strsep(&sp, "|");
    char *rx        = strsep(&sp, "|");
    char *tx        = strsep(&sp, "|");
    char *state     = strsep(&sp, "|");
    char *tls_s     = strsep(&sp, "|");
    char *btn_s     = strsep(&sp, "|");
    char *sda_s     = strsep(&sp, "|");
    char *scl_s     = strsep(&sp, "|");
    char *led_s     = strsep(&sp, "|");
    char *disp_to_s = strsep(&sp, "|");
    char *dcol_s    = strsep(&sp, "|");
    char *drow_s    = strsep(&sp, "|");
    char *web_pass    = strsep(&sp, "|");
    char *client_id_s = strsep(&sp, "|");
    char *cert        = sp; /* remainder = PEM (may contain newlines, no pipes) */

    if (host)     strncpy(s_mqtt_host,        host,    sizeof(s_mqtt_host) - 1);
    if (port_s)   s_mqtt_port  = (uint16_t)atoi(port_s);
    if (user)     strncpy(s_mqtt_user,         user,    sizeof(s_mqtt_user) - 1);
    if (pass && pass[0]) strncpy(s_mqtt_pass,  pass,    sizeof(s_mqtt_pass) - 1);
    if (rx)       strncpy(s_mqtt_rx_topic,     rx,      sizeof(s_mqtt_rx_topic) - 1);
    if (tx)       strncpy(s_mqtt_tx_topic,     tx,      sizeof(s_mqtt_tx_topic) - 1);
    if (state)    strncpy(s_mqtt_state_topic,  state,   sizeof(s_mqtt_state_topic) - 1);
    if (tls_s)    s_mqtt_tls   = (uint8_t)(atoi(tls_s) != 0);
    if (btn_s)    s_btn_pin    = atoi(btn_s);
    if (sda_s)    s_sda_pin    = atoi(sda_s);
    if (scl_s)    s_scl_pin    = atoi(scl_s);
    if (led_s)      s_led_pin         = atoi(led_s);
    if (disp_to_s)  s_disp_timeout_s  = atoi(disp_to_s);
    if (dcol_s)     s_disp_col_off    = atoi(dcol_s);
    if (drow_s)     s_disp_row_off    = atoi(drow_s);
    if (web_pass && web_pass[0])
                  strncpy(s_web_pass, web_pass, sizeof(s_web_pass) - 1);
    if (client_id_s) strncpy(s_mqtt_client_id, client_id_s, sizeof(s_mqtt_client_id) - 1);
    if (cert && cert[0]) {
        free(s_mqtt_cert);
        s_mqtt_cert = strdup(cert);
    }

    mqtt_cfg_save();
    hw_cfg_save();
    web_cfg_save();
    free(buf);
    httpd_resp_sendstr(req, "OK");
    /* Reconnect MQTT with new settings if already connected to WiFi */
    if (s_sta_connected) mqtt_start();
    return ESP_OK;
}

/* =========================================================================
 * HTTP server startup
 * ========================================================================= */

static esp_err_t h_wifi_wps_start(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    if (s_wps_state == 1) {
        httpd_resp_sendstr(req, "busy");
        return ESP_OK;
    }
    esp_wps_config_t wps_cfg = WPS_CONFIG_INIT_DEFAULT(WPS_TYPE_PBC);
    s_wps_state = 1;
    esp_wifi_wps_enable(&wps_cfg);
    esp_wifi_wps_start();
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t h_wifi_wps_status(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    static const char *names[] = {"idle","active","success","failed","timeout"};
    int st = s_wps_state;
    if (st < 0 || st > 4) st = 0;
    httpd_resp_set_type(req, "application/json");
    char out[32];
    snprintf(out, sizeof(out), "{\"state\":\"%s\"}", names[st]);
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static void web_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 20;
    httpd_start(&server, &cfg);

#define URI(path, meth, fn) do { \
    httpd_uri_t u = {.uri = (path), .method = (meth), .handler = (fn)}; \
    httpd_register_uri_handler(server, &u); \
} while (0)

    URI("/",                  HTTP_GET,  h_root);
    URI("/status",            HTTP_GET,  h_status);
    URI("/help",              HTTP_GET,  h_help);
    URI("/wifi",              HTTP_GET,  h_wifi);
    URI("/wifi/connect",      HTTP_POST, h_wifi_connect);
    URI("/wifi/disconnect",   HTTP_POST, h_wifi_disconnect);
    URI("/wifi/list",         HTTP_GET,  h_wifi_list);
    URI("/wifi/delete",       HTTP_POST, h_wifi_delete);
    URI("/wifi/connect_idx",  HTTP_POST, h_wifi_connect_idx);
    URI("/wifi/move",         HTTP_POST, h_wifi_move);
    URI("/wifi/scan",         HTTP_GET,  h_wifi_scan);
    URI("/wifi/wps",          HTTP_POST, h_wifi_wps_start);
    URI("/wifi/wps/status",   HTTP_GET,  h_wifi_wps_status);
    URI("/ap/config",         HTTP_GET,  h_ap_get);
    URI("/ap/config",         HTTP_POST, h_ap_set);
    URI("/config",            HTTP_GET,  h_config);
    URI("/config/get",        HTTP_GET,  h_config_get);
    URI("/config/set",        HTTP_POST, h_config_set);
    URI("/reboot",            HTTP_POST, h_reboot);
#undef URI
}

/* =========================================================================
 * app_main
 * ========================================================================= */

void app_main(void)
{
    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Load all configuration */
    ap_cfg_load();
    mqtt_cfg_load();
    web_cfg_load();
    hw_cfg_load();

    /* Display */
    s_disp_mutex = xSemaphoreCreateMutex();
    ssd1306_init(s_sda_pin, s_scl_pin, 0x3C, s_disp_col_off, s_disp_row_off);

    /* Morse button */
    morse_init();

    /* Notification LED */
    led_init();

    /* WiFi (AP+STA) */
    wifi_init();

    /* Web config server */
    web_start();

    ESP_LOGI(TAG, "nanoChat v%s started. AP: %s", VERSION, s_ap_ssid);
    status_display();
}

//eof
