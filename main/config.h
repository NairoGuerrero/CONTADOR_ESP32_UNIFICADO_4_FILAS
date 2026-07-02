// ============================================================================
//  config.h — Configuración NVS, identidad de puerta, globals compartidos
// ============================================================================

#ifndef CONFIG_H
#define CONFIG_H

#include "contador_tipos.h"

// ── Globals compartidos (definidos en config.c) ──
extern int              puerta_id;      // 1=P1(delantera), 2=P2(trasera)
extern config_t         g_cfg;
extern SemaphoreHandle_t g_lock;
extern const char       *TAG;

// ── Funciones de configuración ──
void     config_load(void);
void     config_save(void);
config_t default_config(void);
void     apply_sanity(config_t *c);
int      clamp_int(int v, int mn, int mx, int dv);

// ── Identidad de puerta ──
void     puerta_id_load(void);
void     puerta_id_save(int pid);

#endif // CONFIG_H
