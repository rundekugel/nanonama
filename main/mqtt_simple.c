/* Minimal MQTT 3.1.1 client over tcp_transport (plain TCP or TLS).
 * Supports: CONNECT (user/pass, TLS), SUBSCRIBE, PUBLISH (QoS 0), PINGREQ.
 * Incoming PUBLISH packets are delivered via the rx callback.
 * Automatically reconnects when the connection drops.
 */

#include "mqtt_simple.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_transport.h"
#include "esp_transport_ssl.h"
#include "esp_transport_tcp.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"

#define TAG "mqtt_simple"

#define MQTT_CONNECT    0x10
#define MQTT_CONNACK    0x20
#define MQTT_PUBLISH    0x30
#define MQTT_SUBSCRIBE  0x82
#define MQTT_SUBACK     0x90
#define MQTT_PINGREQ    0xC0
#define MQTT_PINGRESP   0xD0
#define MQTT_DISCONNECT 0xE0

#define IO_TIMEOUT_MS   5000
#define RECONNECT_MS    5000

static mqtt_simple_cfg_t   s_cfg;
static bool                s_connected = false;
static TaskHandle_t        s_task = NULL;
static esp_transport_handle_t s_transport = NULL;
static SemaphoreHandle_t   s_mutex;  /* protects s_transport + s_connected */

/* ---- MQTT framing helpers ------------------------------------------------ */

static int encode_remlen(uint8_t *buf, int len)
{
    int i = 0;
    do {
        uint8_t b = (uint8_t)(len % 128);
        len /= 128;
        if (len > 0) b |= 0x80;
        buf[i++] = b;
    } while (len > 0 && i < 4);
    return i;
}

/* Write two-byte big-endian length prefix + string */
static int append_str(uint8_t *buf, const char *s, int slen)
{
    buf[0] = (uint8_t)(slen >> 8);
    buf[1] = (uint8_t)(slen & 0xFF);
    memcpy(buf + 2, s, slen);
    return 2 + slen;
}

static int transport_write_all(esp_transport_handle_t t,
                               const uint8_t *buf, int len)
{
    int sent = 0;
    while (sent < len) {
        int n = esp_transport_write(t, (const char *)buf + sent,
                                    len - sent, IO_TIMEOUT_MS);
        if (n <= 0) return -1;
        sent += n;
    }
    return sent;
}

static int transport_read_all(esp_transport_handle_t t, uint8_t *buf, int len)
{
    int got = 0;
    while (got < len) {
        int n = esp_transport_read(t, (char *)buf + got,
                                   len - got, IO_TIMEOUT_MS);
        if (n <= 0) return -1;
        got += n;
    }
    return got;
}

/* Read MQTT remaining-length field; returns decoded value or -1 on error */
static int read_remlen(esp_transport_handle_t t)
{
    int value = 0, mult = 1;
    for (int i = 0; i < 4; i++) {
        uint8_t b;
        if (transport_read_all(t, &b, 1) < 0) return -1;
        value += (b & 0x7F) * mult;
        mult *= 128;
        if (!(b & 0x80)) return value;
    }
    return -1; /* malformed */
}

/* ---- Build and send MQTT packets ---------------------------------------- */

static esp_err_t send_connect(esp_transport_handle_t t,
                               const char *client_id,
                               const char *user, const char *pass,
                               const char *will_topic, const char *will_payload,
                               bool will_retain,
                               int keepalive)
{
    const char *cid  = client_id ? client_id : "nanoChat";
    int cid_len   = strlen(cid);
    int user_len  = user ? strlen(user) : 0;
    int pass_len  = pass ? strlen(pass) : 0;
    int wtopic_len = will_topic   ? (int)strlen(will_topic)   : 0;
    int wpayload_len = will_payload ? (int)strlen(will_payload) : 0;

    /* Variable header: 10 bytes, then payload */
    int payload_len = 2 + cid_len;
    if (will_topic)   payload_len += 2 + wtopic_len + 2 + wpayload_len;
    if (user) payload_len += 2 + user_len;
    if (pass) payload_len += 2 + pass_len;
    int rem = 10 + payload_len;

    uint8_t buf[512];
    int pos = 0;
    buf[pos++] = MQTT_CONNECT;
    pos += encode_remlen(buf + pos, rem);

    /* Protocol name + level */
    buf[pos++] = 0x00; buf[pos++] = 0x04;
    buf[pos++] = 'M'; buf[pos++] = 'Q'; buf[pos++] = 'T'; buf[pos++] = 'T';
    buf[pos++] = 0x04; /* MQTT 3.1.1 */

    /* Connect flags */
    uint8_t flags = 0x02; /* Clean Session */
    if (will_topic) {
        flags |= 0x04;              /* Will Flag */
        if (will_retain) flags |= 0x20; /* Will Retain */
        /* Will QoS 0 (bits 4-3 = 00) */
    }
    if (user) flags |= 0x80;
    if (pass) flags |= 0x40;
    buf[pos++] = flags;

    /* Keep-alive */
    buf[pos++] = (uint8_t)(keepalive >> 8);
    buf[pos++] = (uint8_t)(keepalive & 0xFF);

    /* Payload: client ID, [will topic, will payload,] [user, pass] */
    pos += append_str(buf + pos, cid, cid_len);
    if (will_topic) {
        pos += append_str(buf + pos, will_topic, wtopic_len);
        pos += append_str(buf + pos, will_payload ? will_payload : "", wpayload_len);
    }
    if (user) pos += append_str(buf + pos, user, user_len);
    if (pass) pos += append_str(buf + pos, pass, pass_len);

    return transport_write_all(t, buf, pos) == pos ? ESP_OK : ESP_FAIL;
}

static esp_err_t send_subscribe(esp_transport_handle_t t, const char *topic)
{
    int tlen = strlen(topic);
    int rem  = 2 + 2 + tlen + 1; /* packet_id + topic_len + topic + QoS */
    uint8_t buf[128];
    int pos = 0;
    buf[pos++] = MQTT_SUBSCRIBE;
    pos += encode_remlen(buf + pos, rem);
    buf[pos++] = 0x00; buf[pos++] = 0x01; /* packet identifier = 1 */
    pos += append_str(buf + pos, topic, tlen);
    buf[pos++] = 0x00; /* QoS 0 */
    ESP_LOGI(TAG, "SUBSCRIBE -> \"%s\" (%d bytes)", topic, pos);
    esp_err_t ret = transport_write_all(t, buf, pos) == pos ? ESP_OK : ESP_FAIL;
    if (ret != ESP_OK) ESP_LOGW(TAG, "SUBSCRIBE send failed");
    return ret;
}

static esp_err_t send_publish(esp_transport_handle_t t,
                               const char *topic, const char *data, int dlen,
                               bool retain)
{
    int tlen = strlen(topic);
    int rem  = 2 + tlen + dlen;
    uint8_t hdr[8];
    int hpos = 0;
    hdr[hpos++] = MQTT_PUBLISH | (retain ? 0x01 : 0x00); /* QoS 0 */
    hpos += encode_remlen(hdr + hpos, rem);
    hdr[hpos++] = (uint8_t)(tlen >> 8);
    hdr[hpos++] = (uint8_t)(tlen & 0xFF);
    ESP_LOGI(TAG, "PUBLISH -> \"%s\" len=%d retain=%d", topic, dlen, retain);
    if (transport_write_all(t, hdr, hpos) != hpos) {
        ESP_LOGW(TAG, "PUBLISH hdr write failed");
        return ESP_FAIL;
    }
    if (transport_write_all(t, (const uint8_t *)topic, tlen) != tlen) {
        ESP_LOGW(TAG, "PUBLISH topic write failed");
        return ESP_FAIL;
    }
    if (dlen > 0 && transport_write_all(t, (const uint8_t *)data, dlen) != dlen) {
        ESP_LOGW(TAG, "PUBLISH payload write failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t send_pingreq(esp_transport_handle_t t)
{
    uint8_t buf[2] = {MQTT_PINGREQ, 0x00};
    return transport_write_all(t, buf, 2) == 2 ? ESP_OK : ESP_FAIL;
}

static esp_err_t send_disconnect(esp_transport_handle_t t)
{
    uint8_t buf[2] = {MQTT_DISCONNECT, 0x00};
    transport_write_all(t, buf, 2);
    return ESP_OK;
}

/* ---- Main reader loop ---------------------------------------------------- */

static void handle_incoming(esp_transport_handle_t t)
{
    /* Read fixed header byte 1 (already consumed by caller → passed via pkt_type) */
    /* We're called from within the loop, transport is readable */
    uint8_t type_byte;
    if (transport_read_all(t, &type_byte, 1) < 0) return;

    int rem = read_remlen(t);
    if (rem < 0) return;

    uint8_t pkt_type = type_byte & 0xF0;

    if (pkt_type == (MQTT_PUBLISH & 0xF0)) {
        /* Incoming PUBLISH — handle QoS 0 only (subscribed at QoS 0) */
        uint8_t qos = (type_byte >> 1) & 0x03;
        if (rem < 2 || rem > 8192) {
            ESP_LOGW(TAG, "PUBLISH rem=%d out of range, dropping", rem);
            uint8_t trash[64];
            while (rem > 0) {
                int chunk = rem > 64 ? 64 : rem;
                if (transport_read_all(t, trash, chunk) < 0) return;
                rem -= chunk;
            }
            return;
        }
        uint8_t *buf = malloc(rem);
        if (!buf) return;
        if (transport_read_all(t, buf, rem) < 0) { free(buf); return; }

        int tlen = (buf[0] << 8) | buf[1];
        if (tlen + 2 > rem) { free(buf); return; }
        const char *topic = (const char *)(buf + 2);
        /* For QoS > 0 there is a 2-byte packet ID before the payload */
        int payload_off = 2 + tlen + (qos > 0 ? 2 : 0);
        const char *data = (const char *)(buf + payload_off);
        int dlen = rem - payload_off;
        ESP_LOGI(TAG, "PUBLISH <- \"%.*s\" len=%d qos=%d", tlen, topic, dlen, qos);

        if (dlen >= 0 && s_cfg.rx_cb)
            s_cfg.rx_cb(topic, tlen, data, dlen);
        free(buf);
    } else if (type_byte == MQTT_PINGRESP) {
        ESP_LOGD(TAG, "PINGRESP");
    } else if (pkt_type == (MQTT_SUBACK & 0xF0)) {
        uint8_t tmp[8] = {0};
        if (rem <= (int)sizeof(tmp)) transport_read_all(t, tmp, rem);
        uint8_t rc = (rem >= 3) ? tmp[2] : 0xFF;
        if (rc == 0x80)
            ESP_LOGW(TAG, "SUBACK: broker REJECTED subscription (0x80)");
        else
            ESP_LOGI(TAG, "SUBACK: granted QoS=%d", rc);
    } else {
        ESP_LOGW(TAG, "unknown pkt 0x%02X rem=%d, draining", type_byte, rem);
        uint8_t trash[64];
        while (rem > 0) {
            int chunk = rem > 64 ? 64 : rem;
            if (transport_read_all(t, trash, chunk) < 0) return;
            rem -= chunk;
        }
    }
}

/* ---- MQTT client task ---------------------------------------------------- */

static void mqtt_task(void *arg)
{
    int keepalive = s_cfg.keepalive_sec > 0 ? s_cfg.keepalive_sec : 60;

    for (;;) {
        /* Build transport */
        esp_transport_handle_t t;
        if (s_cfg.use_tls) {
            t = esp_transport_ssl_init();
            if (s_cfg.ca_cert_pem && s_cfg.ca_cert_pem[0]) {
                esp_transport_ssl_set_cert_data(t, s_cfg.ca_cert_pem,
                                                strlen(s_cfg.ca_cert_pem));
            } else {
                /* No custom cert: use the bundled Mozilla CA store */
                esp_transport_ssl_crt_bundle_attach(t, esp_crt_bundle_attach);
            }
        } else {
            t = esp_transport_tcp_init();
        }

        ESP_LOGI(TAG, "Connecting to %s:%d", s_cfg.host, s_cfg.port);
        if (esp_transport_connect(t, s_cfg.host, s_cfg.port,
                                   IO_TIMEOUT_MS) < 0) {
            ESP_LOGW(TAG, "Connect failed");
            esp_transport_destroy(t);
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
            continue;
        }

        /* MQTT CONNECT */
        if (send_connect(t, s_cfg.client_id,
                         s_cfg.username, s_cfg.password,
                         s_cfg.will_topic, s_cfg.will_payload, s_cfg.will_retain,
                         keepalive) != ESP_OK) {
            ESP_LOGW(TAG, "CONNECT send failed");
            goto reconnect;
        }

        /* Wait for CONNACK */
        {
            uint8_t ca[4];
            if (transport_read_all(t, ca, 4) < 0 || ca[0] != MQTT_CONNACK) {
                ESP_LOGW(TAG, "No CONNACK (got 0x%02X)", ca[0]);
                goto reconnect;
            }
            if (ca[3] != 0) {
                ESP_LOGW(TAG, "CONNACK return code %d", ca[3]);
                goto reconnect;
            }
        }

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_transport = t;
        s_connected = true;
        xSemaphoreGive(s_mutex);

        ESP_LOGI(TAG, "MQTT connected");
        if (s_cfg.connected_cb) s_cfg.connected_cb();

        /* Read loop: poll for incoming data, send PINGREQ periodically */
        TickType_t last_ping = xTaskGetTickCount();
        TickType_t ping_interval = pdMS_TO_TICKS((keepalive / 2) * 1000);

        for (;;) {
            /* Poll with short timeout so we can check ping interval */
            int n = esp_transport_poll_read(t, 500 /* ms */);
            if (n < 0) {
                ESP_LOGW(TAG, "Poll error");
                break;
            }
            if (n > 0) {
                handle_incoming(t);
            }
            /* Keepalive PINGREQ — must hold mutex so it doesn't race with publish */
            if ((xTaskGetTickCount() - last_ping) >= ping_interval) {
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                esp_err_t pr = send_pingreq(t);
                xSemaphoreGive(s_mutex);
                if (pr != ESP_OK) break;
                last_ping = xTaskGetTickCount();
            }
        }

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_connected = false;
        s_transport = NULL;
        xSemaphoreGive(s_mutex);
        if (s_cfg.disconnected_cb) s_cfg.disconnected_cb();

reconnect:
        send_disconnect(t);
        esp_transport_close(t);
        esp_transport_destroy(t);
        ESP_LOGI(TAG, "MQTT reconnecting in %d ms", RECONNECT_MS);
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
    }
}

/* ---- Public API ---------------------------------------------------------- */

esp_err_t mqtt_simple_start(const mqtt_simple_cfg_t *cfg)
{
    if (s_task) return ESP_ERR_INVALID_STATE;
    s_cfg   = *cfg;
    s_mutex = xSemaphoreCreateMutex();
    xTaskCreate(mqtt_task, "mqtt", 6144, NULL, 5, &s_task);
    return ESP_OK;
}

void mqtt_simple_stop(void)
{
    if (!s_task) return;
    vTaskDelete(s_task);
    s_task = NULL;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_transport) {
        esp_transport_close(s_transport);
        esp_transport_destroy(s_transport);
        s_transport = NULL;
    }
    s_connected = false;
    xSemaphoreGive(s_mutex);
}

esp_err_t mqtt_simple_publish(const char *topic, const char *data, int len)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_transport_handle_t t = s_transport;
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    if (t && s_connected)
        ret = send_publish(t, topic, data, len, false);
    xSemaphoreGive(s_mutex);
    return ret;
}

esp_err_t mqtt_simple_publish_retained(const char *topic, const char *data, int len)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_transport_handle_t t = s_transport;
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    if (t && s_connected)
        ret = send_publish(t, topic, data, len, true);
    xSemaphoreGive(s_mutex);
    return ret;
}

esp_err_t mqtt_simple_subscribe(const char *topic)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_transport_handle_t t = s_transport;
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    if (t && s_connected)
        ret = send_subscribe(t, topic);
    xSemaphoreGive(s_mutex);
    return ret;
}

bool mqtt_simple_connected(void)
{
    return s_connected;
}
