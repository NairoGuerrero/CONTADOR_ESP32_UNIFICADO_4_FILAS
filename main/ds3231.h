// ============================================================================
//  ds3231.h — Driver DS3231 RTC via I2C (solo P1 lo inicializa/usa)
// ============================================================================

#ifndef DS3231_H
#define DS3231_H

#include "esp_err.h"
#include <stdbool.h>

// ── Estado RTC ──
extern bool rtc_initialized;

// ── Funciones RTC ──
esp_err_t ds3231_init(void);

esp_err_t ds3231_leer_hora(int *anio, int *mes, int *dia,
                           int *hora, int *min, int *seg);

esp_err_t ds3231_configurar_hora(int anio, int mes, int dia,
                                 int dia_sem,
                                 int hora, int min, int seg);

#endif // DS3231_H
