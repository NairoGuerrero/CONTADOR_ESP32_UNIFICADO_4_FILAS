// ============================================================================
//  uart_comm.h — Comunicación UART, comandos, envío de datos
// ============================================================================

#ifndef UART_COMM_H
#define UART_COMM_H

#include "contador_tipos.h"
#include "driver/uart.h"

// ── Funciones UART utilitarias ──
void uart_write_line(uart_port_t u, const char *s);
void uplink_post_ok(const char *msg, const char *tag_msg);
void notificar_p1(const char *evento);
void peer_send(const char *cmd);

// ── RTC cache (leer fuera del lock) ──
void rtc_update_cache(void);

// ── Envío de datos al módem (solo P1, usa RTC cache) ──
void enviarDatos_locked(bool periodico);

// ── Procesador de comandos ──
void procesarComando(const char *cmdline);

// ── Helper ──
bool starts_with(const char *s, const char *pfx);

// ── Tasks FreeRTOS ──
void task_uart_uplink(void *arg);
void task_uart_peer(void *arg);
void task_report(void *arg);

#endif // UART_COMM_H