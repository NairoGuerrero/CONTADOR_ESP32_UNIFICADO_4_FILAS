// ============================================================================
//  gpio_hw.h — Inicialización GPIO y puertos UART
// ============================================================================

#ifndef GPIO_HW_H
#define GPIO_HW_H

#include "driver/uart.h"

// ── Funciones de inicialización ──
void gpio_init_all(void);
void uart_init_port(uart_port_t u, int tx, int rx);

#endif // GPIO_HW_H
