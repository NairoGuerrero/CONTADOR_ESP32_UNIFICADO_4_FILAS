// ============================================================================
//  diagnostico.h — Modo diagnóstico WiFi (AP + HTTP + WebSocket)
// ============================================================================

#ifndef DIAGNOSTICO_H
#define DIAGNOSTICO_H

#include "contador_tipos.h"
#include <stdbool.h>

// ── Estado diagnóstico (definidos en diagnostico.c) ──
extern volatile bool   diag_active;
extern uint32_t        diag_sub, diag_baj, diag_blk;
extern char            diag_ssid[24];

// ── Control del modo diagnóstico ──
void diag_start(void);
void diag_stop(void);

// ── Tasks FreeRTOS ──
void task_diag_btn(void *arg);

#endif // DIAGNOSTICO_H
