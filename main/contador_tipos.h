// ============================================================================
//  contador_tipos.h — Tipos, defines y constantes compartidas
//  Firmware unificado P1/P2 — Contador de pasajeros
// ============================================================================

#ifndef CONTADOR_TIPOS_H
#define CONTADOR_TIPOS_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_timer.h"

// ============================================================================
//  CONFIG HARDWARE — Pines
// ============================================================================

// UART1 = siempre "uplink" (P1→módem, P2→P1)
#define UART_UPLINK_NUM    UART_NUM_1
#define UART_UPLINK_TX     9
#define UART_UPLINK_RX     10

// UART2 = solo P1 → peer P2
#define UART_PEER_NUM      UART_NUM_2
#define UART_PEER_TX       11
#define UART_PEER_RX       12

#define UART_BAUD          57600
#define PIN_CH             7
#define PIN_DIAG_BTN       6

// I2C para DS3231 (solo P1 lo inicializa)
#define I2C_SDA_PIN        20
#define I2C_SCL_PIN        19
#define I2C_FREQ_HZ        100000
#define DS3231_ADDR        0x68

// Sensores
#define NUM_SENSORES   8
#define SENSOR_ACTIVE  1

static const int SENSOR_PINS[NUM_SENSORES] = {36, 35, 38, 37, 40, 39, 42, 41};

// ============================================================================
//  COLUMNAS
// ============================================================================

#define COL_A  0
#define COL_B  1
#define COL_UP_FIRST  COL_A
#define MIX_EPS_MS  50

// ============================================================================
//  DEFAULTS / RANGES
// ============================================================================

#define DEF_LS      25
#define MIN_LS      10
#define MAX_LS      80

#define DEF_MIN     80
#define MIN_MIN     50
#define MAX_MIN     400

#define DEF_MAX     2200
#define MIN_MAX     600
#define MAX_MAX     3000

#define DEF_REF     600
#define MIN_REF     300
#define MAX_REF     900

#define DEF_SIM     150
#define MIN_SIM     80
#define MAX_SIM     500

#define DEF_DET     2
#define MIN_DET     1
#define MAX_DET     3

#define DEF_CONFU   1
#define MIN_CONF    1
#define MAX_CONF    3
#define DEF_CONFD   1

#define DEF_STU     3
#define MIN_STREAK  1
#define MAX_STREAK  5
#define DEF_STD     2

#define DEF_HOLD    120
#define MIN_HOLD    0
#define MAX_HOLD    500

#define DEF_TB_S    7
#define MIN_TB_S    1
#define MAX_TB_S    30

#define TIEMPO_REPORTE_MS    600000
#define GRACIA_LIBERACION_MS 200

#define SAMPLES_PER_READ   5
#define SAMPLE_GAP_US      250
#define MAJORITY_MIN       ((SAMPLES_PER_READ/2)+1)

#define CONF_STREAK_TO_MS  250
#define UART_LINE_MAX      256

// ============================================================================
//  CONFIG WIFI DIAGNÓSTICO
// ============================================================================

#define DIAG_PASS           "12341234"
#define DIAG_TIMEOUT_MS     (10 * 60 * 1000)
#define DIAG_PUSH_INTERVAL  50
#define DIAG_MAX_CLIENTS    3

// ============================================================================
//  FSM: estados (solo los que realmente se usan)
// ============================================================================

typedef enum {
    REPOSO            = 1,
    SUBIDA_DETECTADA  = 2,
    BAJADA_DETECTADA  = 4,
    MIXTO             = 6
} estado_t;

static inline const char *estado_str(estado_t e) {
    switch (e) {
        case REPOSO:            return "REPOSO";
        case SUBIDA_DETECTADA:  return "SUB_DET";
        case BAJADA_DETECTADA:  return "BAJ_DET";
        case MIXTO:             return "MIXTO";
        default:                return "UNK";
    }
}

// ============================================================================
//  CONFIG (NVS) — struct + versionado
// ============================================================================

#define CONFIG_VERSION  1

typedef struct {
    int cam;
    int ch;
    int tb_s;
    int LS, MIN, MAX, REF, SIM;
    int DET, CONFU, CONFD, STU, STD;
    int HOLD;
    int ptrasera;
} config_t;

// ============================================================================
//  TIEMPO — helpers inline
// ============================================================================

static inline int64_t now_ms(void) { return esp_timer_get_time() / 1000; }
static inline int64_t dt_ms(int64_t a, int64_t b) { return a - b; }

// ============================================================================
//  FreeRTOS helper
// ============================================================================

static inline TickType_t ms_to_ticks_min1(uint32_t ms) {
    TickType_t t = pdMS_TO_TICKS(ms);
    return (t == 0) ? 1 : t;
}
#define VDELAY_MS(ms) vTaskDelay(ms_to_ticks_min1(ms))

#endif // CONTADOR_TIPOS_H