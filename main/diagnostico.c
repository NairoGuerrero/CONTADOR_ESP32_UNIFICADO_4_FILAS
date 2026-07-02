// ============================================================================
//  diagnostico.c — Modo diagnóstico WiFi (AP + HTTP + WebSocket)
//  Correcciones: diag_active protegido por g_lock, watchdog en task_diag_btn
// ============================================================================

#include "diagnostico.h"
#include "config.h"
#include "sensores.h"
#include "ds3231.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_task_wdt.h"
#include <unistd.h>

// ── Binario HTML embebido ──
extern const uint8_t diag_page_html_start[] asm("_binary_diag_page_html_start");
extern const uint8_t diag_page_html_end[]   asm("_binary_diag_page_html_end");

// ============================================================================
//  Estado diagnóstico (globals)
// ============================================================================

volatile bool   diag_active      = false;
uint32_t        diag_sub = 0, diag_baj = 0, diag_blk = 0;
char            diag_ssid[24]    = "CONTADOR_P1";

// ── Estado interno (static) ──
static int64_t         diag_start_ms    = 0;
static httpd_handle_t  diag_httpd       = NULL;
static TaskHandle_t    diag_push_handle = NULL;
static esp_netif_t    *diag_netif       = NULL;

static int  ws_fds[DIAG_MAX_CLIENTS];
static int  ws_fd_count = 0;
static SemaphoreHandle_t ws_fd_lock = NULL;

// ============================================================================
//  WebSocket — gestión de clientes
// ============================================================================

static void ws_add_client(int fd) {
    xSemaphoreTake(ws_fd_lock, portMAX_DELAY);
    if (ws_fd_count < DIAG_MAX_CLIENTS) {
        ws_fds[ws_fd_count++] = fd;
        ESP_LOGI(TAG, "DIAG: WS fd=%d (total=%d)", fd, ws_fd_count);
    }
    xSemaphoreGive(ws_fd_lock);
}

static void ws_remove_client(int fd) {
    xSemaphoreTake(ws_fd_lock, portMAX_DELAY);
    for (int i = 0; i < ws_fd_count; i++) {
        if (ws_fds[i] == fd) {
            ws_fds[i] = ws_fds[ws_fd_count - 1];
            ws_fd_count--;
            break;
        }
    }
    xSemaphoreGive(ws_fd_lock);
}

static void ws_broadcast(const char *json) {
    httpd_ws_frame_t pkt = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)json,
        .len     = strlen(json),
        .final   = true,
    };
    xSemaphoreTake(ws_fd_lock, portMAX_DELAY);
    int fds_copy[DIAG_MAX_CLIENTS];
    int count = ws_fd_count;
    memcpy(fds_copy, ws_fds, sizeof(int) * count);
    xSemaphoreGive(ws_fd_lock);
    for (int i = 0; i < count; i++) {
        if (httpd_ws_send_frame_async(diag_httpd, fds_copy[i], &pkt) != ESP_OK)
            ws_remove_client(fds_copy[i]);
    }
}

// ============================================================================
//  HTTP Handlers
// ============================================================================

static esp_err_t http_get_handler(httpd_req_t *req) {
    size_t html_len = diag_page_html_end - diag_page_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, (const char *)diag_page_html_start, html_len);
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ws_add_client(fd);

        char cfg_json[440];
        xSemaphoreTake(g_lock, portMAX_DELAY);
        if (puerta_id == 1) {
            snprintf(cfg_json, sizeof(cfg_json),
                "{\"type\":\"cfg\",\"pid\":%d,"
                "\"ls\":%d,\"min\":%d,\"max\":%d,\"ref\":%d,\"sim\":%d,\"hold\":%d,"
                "\"det\":%d,\"confu\":%d,\"confd\":%d,\"stu\":%d,\"std\":%d,"
                "\"tb\":%d,\"cam\":%d,\"ch\":%d}",
                puerta_id,
                g_cfg.LS, g_cfg.MIN, g_cfg.MAX, g_cfg.REF, g_cfg.SIM, g_cfg.HOLD,
                g_cfg.DET, g_cfg.CONFU, g_cfg.CONFD, g_cfg.STU, g_cfg.STD,
                g_cfg.tb_s, g_cfg.cam, g_cfg.ch);
        } else {
            snprintf(cfg_json, sizeof(cfg_json),
                "{\"type\":\"cfg\",\"pid\":%d,"
                "\"ls\":%d,\"min\":%d,\"max\":%d,\"ref\":%d,\"sim\":%d,\"hold\":%d,"
                "\"det\":%d,\"confu\":%d,\"confd\":%d,\"stu\":%d,\"std\":%d,"
                "\"tb\":%d,\"ch\":%d}",
                puerta_id,
                g_cfg.LS, g_cfg.MIN, g_cfg.MAX, g_cfg.REF, g_cfg.SIM, g_cfg.HOLD,
                g_cfg.DET, g_cfg.CONFU, g_cfg.CONFD, g_cfg.STU, g_cfg.STD,
                g_cfg.tb_s, g_cfg.ch);
        }
        xSemaphoreGive(g_lock);

        httpd_ws_frame_t pkt = {
            .type    = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)cfg_json,
            .len     = strlen(cfg_json),
            .final   = true,
        };
        httpd_ws_send_frame(req, &pkt);
        return ESP_OK;
    }

    httpd_ws_frame_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.type = HTTPD_WS_TYPE_TEXT;
    uint8_t buf[128];
    pkt.payload = buf;
    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, sizeof(buf) - 1);
    if (ret != ESP_OK) return ret;
    buf[pkt.len] = '\0';

    if (strcmp((char *)buf, "RST") == 0) {
        xSemaphoreTake(g_lock, portMAX_DELAY);
        diag_sub = 0; diag_baj = 0; diag_blk = 0;
        xSemaphoreGive(g_lock);
        ESP_LOGI(TAG, "DIAG: Contadores reseteados por WebSocket");
    }

    if (puerta_id == 1 && strncmp((char *)buf, "SETRTC:", 7) == 0 && pkt.len == 21) {
        const char *ts = (char *)buf + 7;
        int y, mo, d, h, mi, s;
        if (sscanf(ts, "%4d%2d%2d%2d%2d%2d", &y, &mo, &d, &h, &mi, &s) == 6) {
            esp_err_t err = ds3231_configurar_hora(y, mo, d, 0, h, mi, s);
            const char *resp = (err == ESP_OK) ? "{\"rtc_sync\":\"ok\"}" : "{\"rtc_sync\":\"error\"}";
            httpd_ws_frame_t ack = {
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t *)resp,
                .len = strlen(resp),
                .final = true,
            };
            httpd_ws_send_frame(req, &ack);
            if (err == ESP_OK)
                ESP_LOGI(TAG, "DIAG: RTC sync: %04d-%02d-%02d %02d:%02d:%02d", y, mo, d, h, mi, s);
            else
                ESP_LOGW(TAG, "DIAG: Error sync RTC: %s", esp_err_to_name(err));
        }
    }

    return ESP_OK;
}

static void httpd_close_fn(httpd_handle_t hd, int sockfd) {
    ws_remove_client(sockfd);
    close(sockfd);
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 4;
    config.close_fn = httpd_close_fn;
    config.stack_size = 6144;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "DIAG: Error iniciando HTTP server");
        return NULL;
    }
    httpd_uri_t uri_get = { .uri="/", .method=HTTP_GET, .handler=http_get_handler };
    httpd_register_uri_handler(server, &uri_get);
    httpd_uri_t uri_ws = { .uri="/ws", .method=HTTP_GET, .handler=ws_handler, .is_websocket=true };
    httpd_register_uri_handler(server, &uri_ws);
    ESP_LOGI(TAG, "DIAG: HTTP server en puerto %d", config.server_port);
    return server;
}

// ============================================================================
//  WiFi AP
// ============================================================================

static void wifi_ap_start(void) {
    diag_netif = esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {0};
    memcpy(wifi_config.ap.ssid, diag_ssid, strlen(diag_ssid));
    wifi_config.ap.ssid_len       = strlen(diag_ssid);
    memcpy(wifi_config.ap.password, DIAG_PASS, strlen(DIAG_PASS));
    wifi_config.ap.channel        = 6;
    wifi_config.ap.max_connection = DIAG_MAX_CLIENTS;
    wifi_config.ap.authmode       = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "DIAG: WiFi AP '%s' — http://192.168.4.1", diag_ssid);
}

static void wifi_ap_stop(void) {
    esp_wifi_stop();
    esp_wifi_deinit();
    if (diag_netif) { esp_netif_destroy_default_wifi(diag_netif); diag_netif = NULL; }
    ESP_LOGI(TAG, "DIAG: WiFi AP detenido");
}

// ============================================================================
//  Task de push periódico
// ============================================================================

static void task_diag_push(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "DIAG: Push task iniciada");

    while (diag_active) {
        int64_t t = now_ms();

        if (dt_ms(t, diag_start_ms) >= DIAG_TIMEOUT_MS) {
            ESP_LOGI(TAG, "DIAG: Timeout 10 min alcanzado");
            xSemaphoreTake(g_lock, portMAX_DELAY);
            diag_active = false;
            xSemaphoreGive(g_lock);
            break;
        }

        xSemaphoreTake(ws_fd_lock, portMAX_DELAY);
        int clients = ws_fd_count;
        xSemaphoreGive(ws_fd_lock);

        if (clients > 0) {
            int sd[NUM_SENSORES], sr[NUM_SENSORES];
            uint32_t su, ba, bl;
            estado_t fsm;
            int ttl_sec;

            xSemaphoreTake(g_lock, portMAX_DELAY);
            memcpy(sd, sensoresDeb, sizeof(sd));
            memcpy(sr, sensoresRaw, sizeof(sr));
            su = diag_sub;  ba = diag_baj;  bl = diag_blk;
            fsm = estadoActual;
            xSemaphoreGive(g_lock);

            ttl_sec = (int)((DIAG_TIMEOUT_MS - dt_ms(t, diag_start_ms)) / 1000);
            if (ttl_sec < 0) ttl_sec = 0;

            char json[320];

            if (puerta_id == 1) {
                int rtc_a=0, rtc_mo=0, rtc_d=0, rtc_h=0, rtc_mi=0, rtc_s=0;
                bool rtc_ok = (ds3231_leer_hora(&rtc_a, &rtc_mo, &rtc_d, &rtc_h, &rtc_mi, &rtc_s) == ESP_OK);
                if (rtc_ok) {
                    snprintf(json, sizeof(json),
                        "{\"s\":[%d,%d,%d,%d,%d,%d,%d,%d],"
                        "\"r\":[%d,%d,%d,%d,%d,%d,%d,%d],"
                        "\"f\":\"%s\","
                        "\"u\":%u,\"d\":%u,\"b\":%u,"
                        "\"t\":%d,"
                        "\"rtc\":\"%04d-%02d-%02d %02d:%02d:%02d\"}",
                        sd[0],sd[1],sd[2],sd[3],sd[4],sd[5],sd[6],sd[7],
                        sr[0],sr[1],sr[2],sr[3],sr[4],sr[5],sr[6],sr[7],
                        estado_str(fsm),
                        (unsigned)su, (unsigned)ba, (unsigned)bl,
                        ttl_sec,
                        rtc_a, rtc_mo, rtc_d, rtc_h, rtc_mi, rtc_s);
                } else {
                    snprintf(json, sizeof(json),
                        "{\"s\":[%d,%d,%d,%d,%d,%d,%d,%d],"
                        "\"r\":[%d,%d,%d,%d,%d,%d,%d,%d],"
                        "\"f\":\"%s\","
                        "\"u\":%u,\"d\":%u,\"b\":%u,"
                        "\"t\":%d}",
                        sd[0],sd[1],sd[2],sd[3],sd[4],sd[5],sd[6],sd[7],
                        sr[0],sr[1],sr[2],sr[3],sr[4],sr[5],sr[6],sr[7],
                        estado_str(fsm),
                        (unsigned)su, (unsigned)ba, (unsigned)bl,
                        ttl_sec);
                }
            } else {
                snprintf(json, sizeof(json),
                    "{\"s\":[%d,%d,%d,%d,%d,%d,%d,%d],"
                    "\"r\":[%d,%d,%d,%d,%d,%d,%d,%d],"
                    "\"f\":\"%s\",\"u\":%u,\"d\":%u,\"b\":%u,\"t\":%d}",
                    sd[0],sd[1],sd[2],sd[3],sd[4],sd[5],sd[6],sd[7],
                    sr[0],sr[1],sr[2],sr[3],sr[4],sr[5],sr[6],sr[7],
                    estado_str(fsm),(unsigned)su,(unsigned)ba,(unsigned)bl,ttl_sec);
            }

            ws_broadcast(json);
        }

        vTaskDelay(pdMS_TO_TICKS(DIAG_PUSH_INTERVAL));
    }

    // Cleanup si salimos por timeout
    if (!diag_active) {
        if (diag_httpd) { httpd_stop(diag_httpd); diag_httpd = NULL; }
        wifi_ap_stop();
        xSemaphoreTake(ws_fd_lock, portMAX_DELAY);
        ws_fd_count = 0;
        xSemaphoreGive(ws_fd_lock);
        ESP_LOGI(TAG, "DIAG: Modo diagnóstico finalizado (timeout)");
    }

    diag_push_handle = NULL;
    vTaskDelete(NULL);
}

// ============================================================================
//  Control del modo diagnóstico — diag_active protegido por g_lock
// ============================================================================

void diag_start(void) {
    if (diag_active) {
        ESP_LOGW(TAG, "DIAG: Ya está activo");
        return;
    }
    ESP_LOGI(TAG, "DIAG: Iniciando modo diagnóstico...");
    if (ws_fd_lock == NULL) ws_fd_lock = xSemaphoreCreateMutex();
    ws_fd_count = 0;
    wifi_ap_start();
    diag_httpd = start_webserver();
    if (!diag_httpd) {
        ESP_LOGE(TAG, "DIAG: Fallo HTTP server, abortando");
        wifi_ap_stop();
        return;
    }
    diag_start_ms = now_ms();

    // Proteger escritura de diag_active y contadores con g_lock
    xSemaphoreTake(g_lock, portMAX_DELAY);
    diag_active = true;
    diag_sub = 0; diag_baj = 0; diag_blk = 0;
    xSemaphoreGive(g_lock);

    xTaskCreate(task_diag_push, "diag_push", 5120, NULL, 7, &diag_push_handle);
    ESP_LOGI(TAG, "DIAG: ACTIVO — WiFi '%s' pass '%s'", diag_ssid, DIAG_PASS);
    ESP_LOGI(TAG, "DIAG: Abrir http://192.168.4.1 — Auto-salida en 10 min");
}

void diag_stop(void) {
    if (!diag_active) {
        ESP_LOGW(TAG, "DIAG: No está activo");
        return;
    }
    ESP_LOGI(TAG, "DIAG: Deteniendo modo diagnóstico...");

    // Proteger escritura de diag_active con g_lock
    xSemaphoreTake(g_lock, portMAX_DELAY);
    diag_active = false;
    xSemaphoreGive(g_lock);

    if (diag_push_handle) vTaskDelay(pdMS_TO_TICKS(DIAG_PUSH_INTERVAL * 2));
    if (diag_httpd) { httpd_stop(diag_httpd); diag_httpd = NULL; }
    wifi_ap_stop();
    if (ws_fd_lock) {
        xSemaphoreTake(ws_fd_lock, portMAX_DELAY);
        ws_fd_count = 0;
        xSemaphoreGive(ws_fd_lock);
    }
    ESP_LOGI(TAG, "DIAG: Modo diagnóstico DESACTIVADO");
}

// ============================================================================
//  Task: botón diagnóstico (con watchdog)
// ============================================================================

void task_diag_btn(void *arg) {
    (void)arg;
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "TASK,diag_btn,start,pin=%d", PIN_DIAG_BTN);

    bool last_level = true;
    int64_t press_ts = 0;
    const int DEBOUNCE_MS = 50;

    while (1) {
        esp_task_wdt_reset();
        bool level = gpio_get_level((gpio_num_t)PIN_DIAG_BTN) != 0;

        if (last_level && !level) {
            press_ts = now_ms();
        }

        if (!last_level && !level && press_ts) {
            if (dt_ms(now_ms(), press_ts) >= DEBOUNCE_MS) {
                if (diag_active) {
                    ESP_LOGI(TAG, "DIAG_BTN: Desactivando");
                    diag_stop();
                } else {
                    ESP_LOGI(TAG, "DIAG_BTN: Activando");
                    diag_start();
                }
                press_ts = 0;
                while (gpio_get_level((gpio_num_t)PIN_DIAG_BTN) == 0) {
                    esp_task_wdt_reset();
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            }
        }

        if (level) press_ts = 0;
        last_level = level;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}