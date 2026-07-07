// ============================================================================
//  gpio_hw.c — Inicialización GPIO y puertos UART
// ============================================================================

#include "gpio_hw.h"
#include "contador_tipos.h"

#include "driver/gpio.h"
#include "esp_system.h"

// ============================================================================
//  GPIO — Inicializar todos los pines
// ============================================================================

void gpio_init_all(void) {
    // Pin de salida: CH (chicharra)
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_CH),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&out);
    gpio_set_level(PIN_CH, 0);

    // Botón de diagnóstico
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << PIN_DIAG_BTN),
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&btn);

    // Sensores (8 pines de entrada con pull-up)
    for (int i = 0; i < NUM_SENSORES; i++) {
        gpio_num_t gp = (gpio_num_t)SENSOR_PINS[i];
        gpio_config_t in = {
            .pin_bit_mask = (1ULL << gp),
            .mode         = GPIO_MODE_INPUT,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .intr_type    = GPIO_INTR_DISABLE
        };
        gpio_config(&in);
    }
}

// ============================================================================
//  UART — Inicializar un puerto
// ============================================================================

void uart_init_port(uart_port_t u, int tx, int rx) {
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(u, 2048, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(u, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(u, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}
