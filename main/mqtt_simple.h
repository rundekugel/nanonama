#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* Callback types */
typedef void (*mqtt_simple_data_cb_t)(const char *topic, int topic_len,
                                      const char *data,  int data_len);
typedef void (*mqtt_simple_event_cb_t)(void); /* connected / disconnected */

typedef struct {
    const char *host;
    int         port;          /* usually 8883 for TLS */
    const char *username;      /* NULL = no auth */
    const char *password;      /* NULL = no auth */
    const char *ca_cert_pem;   /* NULL = no cert verification */
    const char *client_id;     /* NULL → "nanoChat" */
    int         keepalive_sec; /* 0 → 60 */
    bool        use_tls;

    /* Last Will and Testament — set both or neither */
    const char *will_topic;    /* NULL = no will */
    const char *will_payload;  /* message broker publishes on unclean disconnect */
    bool        will_retain;

    mqtt_simple_data_cb_t  rx_cb;
    mqtt_simple_event_cb_t connected_cb;
    mqtt_simple_event_cb_t disconnected_cb;
} mqtt_simple_cfg_t;

/** Start MQTT client task with the given config. Reconnects automatically. */
esp_err_t mqtt_simple_start(const mqtt_simple_cfg_t *cfg);

/** Stop and destroy the MQTT client task. */
void mqtt_simple_stop(void);

/** Publish a message (QoS 0). Safe to call from any task. */
esp_err_t mqtt_simple_publish(const char *topic, const char *data, int len);

/** Publish with retain flag. */
esp_err_t mqtt_simple_publish_retained(const char *topic, const char *data, int len);

/** Subscribe to a topic (called after connect, usually from connected_cb). */
esp_err_t mqtt_simple_subscribe(const char *topic);

bool mqtt_simple_connected(void);
