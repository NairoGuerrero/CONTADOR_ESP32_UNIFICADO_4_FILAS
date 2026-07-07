// ============================================================================
//  main.c — Punto de entrada del firmware
//  CONTADOR DE PASAJEROS — Firmware unificado P1/P2
// ============================================================================

#include "contador_tipos.h"
#include "config.h"
#include "ds3231.h"
#include "gpio_hw.h"
#include "sensores.h"
#include "uart_comm.h"
#include "diagnostico.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_task_wdt.h"

// ============================================================================
//  app_main
// ============================================================================

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Configurar watchdog: 10s timeout, panic on trigger
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms      = 10000,
        .idle_core_mask  = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic   = true,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);

    // Leer identidad de NVS
    puerta_id_load();

    // Configurar TAG y SSID dinámicos
    TAG = (puerta_id == 1) ? "P1" : "P2";
    snprintf(diag_ssid, sizeof(diag_ssid), "CONTADOR_P%d", puerta_id);

    g_lock = xSemaphoreCreateMutex();
    config_load();
    gpio_init_all();

    // UART1 = siempre (uplink)
    uart_init_port(UART_UPLINK_NUM, UART_UPLINK_TX, UART_UPLINK_RX);

    // UART2 = solo P1 (peer P2)
    if (puerta_id == 1) {
        uart_init_port(UART_PEER_NUM, UART_PEER_TX, UART_PEER_RX);
        vTaskDelay(pdMS_TO_TICKS(100));
        uart_flush_input(UART_PEER_NUM);
    }

    // RTC = solo P1
    if (puerta_id == 1) {
        if (ds3231_init() != ESP_OK) {
            ESP_LOGE(TAG, "RTC: FALLO inicialización DS3231 — timestamps serán 0");
        } else {
            rtc_update_cache();  // Primer llenado del cache RTC
        }
    }

    ESP_LOGI(TAG, "BOOT,P%d,esp-idf,unified", puerta_id);
    ESP_LOGI(TAG, "PINS,s=[%d,%d,%d,%d,%d,%d,%d,%d],ch=%d,btn=%d",
        SENSOR_PINS[0], SENSOR_PINS[1], SENSOR_PINS[2], SENSOR_PINS[3],
        SENSOR_PINS[4], SENSOR_PINS[5], SENSOR_PINS[6], SENSOR_PINS[7],
        PIN_CH, PIN_DIAG_BTN);
    ESP_LOGI(TAG, "CFG,LS=%d,MIN=%d,MAX=%d,REF=%d,SIM=%d,HOLD=%d,"
                  "DET=%d,CONFU=%d,CONFD=%d,STU=%d,STD=%d,"
                  "TB=%d,CAM=%d,CH=%d,EPS=%d,PT?=%d",
        g_cfg.LS, g_cfg.MIN, g_cfg.MAX, g_cfg.REF, g_cfg.SIM, g_cfg.HOLD,
        g_cfg.DET, g_cfg.CONFU, g_cfg.CONFD, g_cfg.STU, g_cfg.STD,
        g_cfg.tb_s, g_cfg.cam, g_cfg.ch, MIX_EPS_MS, g_cfg.ptrasera);

    // Log RTC (P1 only)
    if (puerta_id == 1 && rtc_initialized) {
        int a, mo, d, h, mi, s;
        if (ds3231_leer_hora(&a, &mo, &d, &h, &mi, &s) == ESP_OK) {
            ESP_LOGI(TAG, "RTC: %04d-%02d-%02d %02d:%02d:%02d", a, mo, d, h, mi, s);
        } else {
            ESP_LOGW(TAG, "RTC: No se pudo leer la hora");
        }
    }

    // Tasks comunes
    xTaskCreate(task_sensores,     "sensores",     4096, NULL, 10, NULL);
    xTaskCreate(task_fsm,          "fsm",          4096, NULL,  9, NULL);
    xTaskCreate(task_bloqueos,     "bloqueos",     4096, NULL,  8, NULL);
    xTaskCreate(task_uart_uplink,  "uart_uplink",  8192, NULL, 12, NULL);
    xTaskCreate(task_diag_btn,     "diag_btn",     4096, NULL,  6, NULL);

    // Tasks solo P1
    if (puerta_id == 1) {
        xTaskCreate(task_uart_peer, "uart_peer",   4096, NULL, 11, NULL);
        xTaskCreate(task_report,    "report",      4096, NULL,  5, NULL);
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}