// ============================================================================
//  sensores.h — Lectura de sensores, FSM, conteo, bloqueos
// ============================================================================

#ifndef SENSORES_H
#define SENSORES_H

#include "contador_tipos.h"

// ============================================================================
//  Estado global de sensores y contadores (definidos en sensores.c)
// ============================================================================

extern int     sensoresRaw[NUM_SENSORES];
extern int     sensoresDeb[NUM_SENSORES];
extern int64_t tCambio[NUM_SENSORES];

extern bool    enBloqueoBruto[NUM_SENSORES];
extern int64_t tInicioBloqueo[NUM_SENSORES];

extern bool    alarmaBloqueoActiva;
extern int64_t tUltimaActividad;
extern int64_t tUltimoAvisoBloqueo;
extern int     contadorBloqueos;

extern uint32_t subida, bajada, bloqueos;
extern uint32_t subidaP2, bajadaP2, bloqueosP2;  // Solo P1 las usa
extern uint32_t envio_id;

extern estado_t estadoActual;
extern int64_t  tDeteccion;
extern int64_t  tUltimoConteo;

extern bool    enRefractario;
extern int64_t refractHasta;

// ============================================================================
//  Funciones de reset (usadas también desde uart_comm)
// ============================================================================

void reset_dir_locked(void);

// ============================================================================
//  Tasks FreeRTOS
// ============================================================================

void task_sensores(void *arg);
void task_fsm(void *arg);
void task_bloqueos(void *arg);

#endif // SENSORES_H
