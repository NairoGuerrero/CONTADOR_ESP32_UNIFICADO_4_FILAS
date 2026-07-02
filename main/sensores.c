// ============================================================================
//  sensores.c — Lectura de sensores, FSM, conteo, bloqueos
//  Correcciones: lectura batch (no busy-wait por sensor), conteo unificado,
//                watchdog, RTC cache pre-lock
// ============================================================================

#include "sensores.h"
#include "config.h"
#include "uart_comm.h"
#include "diagnostico.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"

// ============================================================================
//  Definición de globals
// ============================================================================

int     sensoresRaw[NUM_SENSORES];
int     sensoresDeb[NUM_SENSORES];
int64_t tCambio[NUM_SENSORES];

bool    enBloqueoBruto[NUM_SENSORES];
int64_t tInicioBloqueo[NUM_SENSORES];

bool    alarmaBloqueoActiva  = false;
int64_t tUltimaActividad     = 0;
int64_t tUltimoAvisoBloqueo  = 0;
int     contadorBloqueos      = 0;

uint32_t subida = 0, bajada = 0, bloqueos = 0;
uint32_t subidaP2 = 0, bajadaP2 = 0, bloqueosP2 = 0;
uint32_t envio_id = 0;

estado_t estadoActual  = REPOSO;
int64_t  tDeteccion    = 0;
int64_t  tUltimoConteo = 0;

bool    enRefractario = false;
int64_t refractHasta  = 0;

// ── Streaks (internas) ──
static int     sub_conf_streak  = 0;
static int     baj_conf_streak  = 0;
static int64_t sub_last_conf_ms = 0;
static int64_t baj_last_conf_ms = 0;

static int64_t col_det_ts[2] = {0, 0};

// ============================================================================
//  Lectura de sensores — BATCH (todos en cada pasada)
//  5 pasadas × 250µs = ~1.25ms total en vez de ~10ms secuencial
// ============================================================================

static void read_all_sensors_majority(int results[NUM_SENSORES]) {
    int ones[NUM_SENSORES];
    memset(ones, 0, sizeof(ones));

    for (int k = 0; k < SAMPLES_PER_READ; k++) {
        for (int i = 0; i < NUM_SENSORES; i++) {
            if (gpio_get_level((gpio_num_t)SENSOR_PINS[i]) == SENSOR_ACTIVE)
                ones[i]++;
        }
        if (k < SAMPLES_PER_READ - 1)
            esp_rom_delay_us(SAMPLE_GAP_US);
    }
    for (int i = 0; i < NUM_SENSORES; i++)
        results[i] = (ones[i] >= MAJORITY_MIN) ? 1 : 0;
}

static bool todos_inactivos_locked(void) {
    for (int i = 0; i < NUM_SENSORES; i++) if (sensoresDeb[i]) return false;
    return true;
}

static int col_count_locked(int col) {
    if (col == COL_A)
        return (sensoresDeb[0]?1:0)+(sensoresDeb[2]?1:0)+(sensoresDeb[4]?1:0)+(sensoresDeb[6]?1:0);
    else
        return (sensoresDeb[1]?1:0)+(sensoresDeb[3]?1:0)+(sensoresDeb[5]?1:0)+(sensoresDeb[7]?1:0);
}

static bool det_col_locked(int col)         { return col_count_locked(col) >= g_cfg.DET; }
static bool confirmar_subida_locked(void)   { return col_count_locked(1 - COL_UP_FIRST) >= g_cfg.CONFU; }
static bool confirmar_bajada_locked(void)   { return col_count_locked(COL_UP_FIRST) >= g_cfg.CONFD; }

// ============================================================================
//  Streaks
// ============================================================================

static void reset_streaks_locked(void) {
    sub_conf_streak = 0; baj_conf_streak = 0;
    sub_last_conf_ms = 0; baj_last_conf_ms = 0;
}

static int update_streak_locked(bool is_up, bool seen_conf) {
    int64_t t = now_ms();
    if (is_up) {
        if (sub_last_conf_ms && dt_ms(t, sub_last_conf_ms) > CONF_STREAK_TO_MS) sub_conf_streak = 0;
        if (seen_conf) { sub_conf_streak++; sub_last_conf_ms = t; }
        return sub_conf_streak;
    } else {
        if (baj_last_conf_ms && dt_ms(t, baj_last_conf_ms) > CONF_STREAK_TO_MS) baj_conf_streak = 0;
        if (seen_conf) { baj_conf_streak++; baj_last_conf_ms = t; }
        return baj_conf_streak;
    }
}

void reset_dir_locked(void) {
    col_det_ts[0] = col_det_ts[1] = 0;
    reset_streaks_locked();
}

// ============================================================================
//  Bloqueos
// ============================================================================

static void enviarBloqueo_locked(void) {
    if (diag_active) {
        diag_blk++;
        ESP_LOGI(TAG, "DIAG_BLK,blk=%u", (unsigned)diag_blk);
    } else if (puerta_id == 1) {
        if (contadorBloqueos <= 3) {
            char b[64];
            snprintf(b, sizeof(b), "AT$GCAM=%d,0", g_cfg.cam);
            uart_write_line(UART_UPLINK_NUM, b);
        } else if (contadorBloqueos >= 50) {
            contadorBloqueos = 0;
        }
        bloqueos++;
        enviarDatos_locked(false);
    } else {
        bloqueos++;
        notificar_p1("BLK");
    }
}

// ============================================================================
//  Registrar conteo — unificado subida/bajada
// ============================================================================

static void registrarConteo_locked(bool es_subida) {
    tUltimoConteo = now_ms();
    const char *tipo = es_subida ? "UP" : "DN";

    if (diag_active) {
        if (es_subida) diag_sub++; else diag_baj++;
        ESP_LOGI(TAG, "DIAG_COUNT,type=%s,count=%u,t=%lld",
                 tipo, (unsigned)(es_subida ? diag_sub : diag_baj),
                 (long long)tUltimoConteo);
    } else if (puerta_id == 1) {
        if (es_subida) subida++; else bajada++;
        ESP_LOGI(TAG, "COUNT,type=%s,sub=%u,dn=%u,t=%lld",
                 tipo, (unsigned)subida, (unsigned)bajada,
                 (long long)tUltimoConteo);
        enviarDatos_locked(false);
    } else {
        if (es_subida) subida++; else bajada++;
        ESP_LOGI(TAG, "COUNT,type=%s,count=%u,t=%lld",
                 tipo, (unsigned)(es_subida ? subida : bajada),
                 (long long)tUltimoConteo);
        notificar_p1(tipo);
    }

    estadoActual = REPOSO;
    enRefractario = true;
    refractHasta = now_ms() + g_cfg.REF;
    reset_dir_locked();
}

// ============================================================================
//  FSM: dirección
// ============================================================================

static int decide_dir_locked(void) {
    int64_t tA = col_det_ts[COL_A];
    int64_t tB = col_det_ts[COL_B];
    if (!tA || !tB) return 0;
    int64_t diff = dt_ms(tA, tB);
    if (diff > MIX_EPS_MS)    return -1;
    if (diff < -MIX_EPS_MS)   return  1;
    return 0;
}

// ============================================================================
//  TASK: Sensores — lectura batch y debounce
// ============================================================================

void task_sensores(void *arg) {
    (void)arg;
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "TASK,sensores,start");

    int64_t t0 = now_ms();
    xSemaphoreTake(g_lock, portMAX_DELAY);
    for (int i = 0; i < NUM_SENSORES; i++) {
        sensoresRaw[i] = 0; sensoresDeb[i] = 0; tCambio[i] = t0;
        enBloqueoBruto[i] = false; tInicioBloqueo[i] = 0;
    }
    tUltimaActividad = t0;
    xSemaphoreGive(g_lock);

    while (1) {
        esp_task_wdt_reset();
        int64_t t = now_ms();

        // Lectura batch FUERA del lock (~1.25ms en vez de ~10ms)
        int raw_reads[NUM_SENSORES];
        read_all_sensors_majority(raw_reads);

        typedef struct { int pin; int idx; int val; } EdgeLog_t;
        EdgeLog_t pendingLogs[NUM_SENSORES];
        int nLogs = 0;

        xSemaphoreTake(g_lock, portMAX_DELAY);
        for (int i = 0; i < NUM_SENSORES; i++) {
            int r = raw_reads[i];
            if (r != sensoresRaw[i]) {
                sensoresRaw[i] = r;
                tCambio[i] = t;
            } else if (sensoresDeb[i] != r && dt_ms(t, tCambio[i]) >= g_cfg.LS) {
                pendingLogs[nLogs++] = (EdgeLog_t){ SENSOR_PINS[i], i, r };
                sensoresDeb[i] = r;
            }
            if (sensoresDeb[i]) tUltimaActividad = t;
        }
        xSemaphoreGive(g_lock);

        for (int k = 0; k < nLogs; k++) {
            ESP_LOGI(TAG, "EDGE,ms=%lld,pin=%d,idx=%d,val=%d",
                (long long)t, pendingLogs[k].pin, pendingLogs[k].idx, pendingLogs[k].val);
        }
        VDELAY_MS(2);
    }
}

// ============================================================================
//  TASK: Bloqueos + CH
// ============================================================================

void task_bloqueos(void *arg) {
    (void)arg;
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "TASK,bloqueos,start");
    int64_t last_rtc_blk = 0;

    while (1) {
        esp_task_wdt_reset();
        int64_t t = now_ms();

        // Refrescar RTC cache fuera del lock (max 1 vez/s)
        if (puerta_id == 1 && dt_ms(t, last_rtc_blk) > 1000) {
            rtc_update_cache();
            last_rtc_blk = t;
        }

        xSemaphoreTake(g_lock, portMAX_DELAY);

        bool hayActividad = false;
        bool bloqueoConfirmado = false;

        for (int i = 0; i < NUM_SENSORES; i++) {
            if (sensoresDeb[i]) {
                hayActividad = true;
                if (!enBloqueoBruto[i]) { enBloqueoBruto[i] = true; tInicioBloqueo[i] = t; }
                if (dt_ms(t, tInicioBloqueo[i]) >= (int64_t)g_cfg.tb_s * 1000) bloqueoConfirmado = true;
            } else {
                enBloqueoBruto[i] = false; tInicioBloqueo[i] = 0;
            }
        }
        if (hayActividad) tUltimaActividad = t;

        if (bloqueoConfirmado) {
            if (!alarmaBloqueoActiva) {
                alarmaBloqueoActiva = true;
                if (g_cfg.ch == 1) gpio_set_level(PIN_CH, 1);
                tUltimoAvisoBloqueo = 0;
                ESP_LOGW(TAG, "ALARM,on=1,t=%lld", (long long)t);
            }
            if (tUltimoAvisoBloqueo == 0 || dt_ms(t, tUltimoAvisoBloqueo) >= (int64_t)g_cfg.tb_s * 1000) {
                contadorBloqueos++;
                enviarBloqueo_locked();
                tUltimoAvisoBloqueo = t;
            }
        }

        if (alarmaBloqueoActiva) {
            gpio_set_level(PIN_CH, (g_cfg.ch == 1) ? 1 : 0);
            if (dt_ms(t, tUltimaActividad) >= GRACIA_LIBERACION_MS) {
                bool act = false;
                for (int i = 0; i < NUM_SENSORES; i++) if (sensoresDeb[i]) { act = true; break; }
                if (!act) {
                    alarmaBloqueoActiva = false;
                    gpio_set_level(PIN_CH, 0);
                    contadorBloqueos = 0;
                    tUltimoAvisoBloqueo = 0;
                    ESP_LOGI(TAG, "ALARM,on=0,t=%lld", (long long)t);
                }
            }
        }

        xSemaphoreGive(g_lock);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================================
//  TASK: FSM — Máquina de estados
// ============================================================================

void task_fsm(void *arg) {
    (void)arg;
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "TASK,fsm,start");
    estado_t estadoAnterior = REPOSO;
    int64_t last_rtc_fsm = 0;

    while (1) {
        esp_task_wdt_reset();
        int64_t t = now_ms();

        // Refrescar RTC cache fuera del lock (max 1 vez/s)
        if (puerta_id == 1 && dt_ms(t, last_rtc_fsm) > 1000) {
            rtc_update_cache();
            last_rtc_fsm = t;
        }

        xSemaphoreTake(g_lock, portMAX_DELAY);

        if (enRefractario) {
            if (t < refractHasta || !todos_inactivos_locked()) {
                xSemaphoreGive(g_lock);
                VDELAY_MS(5);
                continue;
            }
            enRefractario = false;
            estadoActual  = REPOSO;
            reset_dir_locked();
        }

        if (det_col_locked(COL_A) && col_det_ts[COL_A] == 0) col_det_ts[COL_A] = t;
        if (det_col_locked(COL_B) && col_det_ts[COL_B] == 0) col_det_ts[COL_B] = t;

        bool detA = det_col_locked(COL_A);
        bool detB = det_col_locked(COL_B);

        if (estadoActual == REPOSO) {
            if (detA && detB) {
                int dir = decide_dir_locked();
                if (dir == 1) {
                    estadoActual = SUBIDA_DETECTADA;
                    tDeteccion   = col_det_ts[COL_A];
                    reset_streaks_locked();
                } else if (dir == -1) {
                    estadoActual = BAJADA_DETECTADA;
                    tDeteccion   = col_det_ts[COL_B];
                    reset_streaks_locked();
                } else {
                    estadoActual = MIXTO;
                    int64_t tA = col_det_ts[COL_A], tB = col_det_ts[COL_B];
                    tDeteccion = (tA && tB) ? (tA < tB ? tA : tB) : t;
                    reset_streaks_locked();
                }
            } else if (detA) {
                estadoActual = SUBIDA_DETECTADA;
                tDeteccion   = col_det_ts[COL_A] ? col_det_ts[COL_A] : t;
                reset_streaks_locked();
            } else if (detB) {
                estadoActual = BAJADA_DETECTADA;
                tDeteccion   = col_det_ts[COL_B] ? col_det_ts[COL_B] : t;
                reset_streaks_locked();
            } else {
                reset_dir_locked();
            }

        } else if (estadoActual == SUBIDA_DETECTADA) {
            int64_t d = dt_ms(t, tDeteccion);
            if (confirmar_subida_locked()) {
                if (d >= g_cfg.MIN && d <= g_cfg.MAX) {
                    int streak = update_streak_locked(true, true);
                    ESP_LOGI(TAG, "CONF,type=UP,d=%lld,streak=%d,need=%d",
                             (long long)d, streak, g_cfg.STU);
                    if (streak >= g_cfg.STU) {
                        reset_streaks_locked();
                        registrarConteo_locked(true);
                    }
                } else if (d > g_cfg.MAX) {
                    ESP_LOGI(TAG, "DROP,type=UP,reason=MAX,d=%lld", (long long)d);
                    estadoActual = REPOSO;
                    reset_dir_locked();
                }
            } else {
                if (detB && d <= g_cfg.SIM) {
                    ESP_LOGI(TAG, "FSM,SUB_DET->MIXTO,d=%lld", (long long)d);
                    estadoActual = MIXTO;
                    int64_t tB = col_det_ts[COL_B];
                    if (tB && tB < tDeteccion) tDeteccion = tB;
                    reset_streaks_locked();
                } else if (d > g_cfg.MAX) {
                    ESP_LOGI(TAG, "DROP,type=UP,reason=MAX,d=%lld", (long long)d);
                    estadoActual = REPOSO;
                    reset_dir_locked();
                } else if (todos_inactivos_locked() && d > g_cfg.HOLD) {
                    ESP_LOGI(TAG, "DROP,type=UP,reason=IDLE,d=%lld", (long long)d);
                    estadoActual = REPOSO;
                    reset_dir_locked();
                }
            }

        } else if (estadoActual == BAJADA_DETECTADA) {
            int64_t d = dt_ms(t, tDeteccion);
            if (confirmar_bajada_locked()) {
                if (d >= g_cfg.MIN && d <= g_cfg.MAX) {
                    int streak = update_streak_locked(false, true);
                    ESP_LOGI(TAG, "CONF,type=DN,d=%lld,streak=%d,need=%d",
                             (long long)d, streak, g_cfg.STD);
                    if (streak >= g_cfg.STD) {
                        reset_streaks_locked();
                        registrarConteo_locked(false);
                    }
                } else if (d > g_cfg.MAX) {
                    ESP_LOGI(TAG, "DROP,type=DN,reason=MAX,d=%lld", (long long)d);
                    estadoActual = REPOSO;
                    reset_dir_locked();
                }
            } else {
                if (detA && d <= g_cfg.SIM) {
                    ESP_LOGI(TAG, "FSM,BAJ_DET->MIXTO,d=%lld", (long long)d);
                    estadoActual = MIXTO;
                    int64_t tA = col_det_ts[COL_A];
                    if (tA && tA < tDeteccion) tDeteccion = tA;
                    reset_streaks_locked();
                } else if (d > g_cfg.MAX) {
                    ESP_LOGI(TAG, "DROP,type=DN,reason=MAX,d=%lld", (long long)d);
                    estadoActual = REPOSO;
                    reset_dir_locked();
                } else if (todos_inactivos_locked() && d > g_cfg.HOLD) {
                    ESP_LOGI(TAG, "DROP,type=DN,reason=IDLE,d=%lld", (long long)d);
                    estadoActual = REPOSO;
                    reset_dir_locked();
                }
            }

        } else if (estadoActual == MIXTO) {
            int64_t d = dt_ms(t, tDeteccion);
            if (d <= g_cfg.MAX) {
                bool conteoReciente = tUltimoConteo && dt_ms(t, tUltimoConteo) < g_cfg.REF;
                if (!conteoReciente) {
                    int dir = decide_dir_locked();
                    int cA = col_count_locked(COL_A);
                    int cB = col_count_locked(COL_B);

                    if (dir == 1) {
                        if (cB >= g_cfg.CONFU) {
                            int streak = update_streak_locked(true, true);
                            if (streak >= g_cfg.STU) { reset_streaks_locked(); registrarConteo_locked(true); }
                        }
                    } else if (dir == -1) {
                        if (cA >= g_cfg.CONFD) {
                            int streak = update_streak_locked(false, true);
                            if (streak >= g_cfg.STD) { reset_streaks_locked(); registrarConteo_locked(false); }
                        }
                    } else {
                        if (cA > 0 && cB == 0) {
                            int streak = update_streak_locked(false, true);
                            if (streak >= g_cfg.STD) { reset_streaks_locked(); registrarConteo_locked(false); }
                        } else if (cB > 0 && cA == 0) {
                            int streak = update_streak_locked(true, true);
                            if (streak >= g_cfg.STU) { reset_streaks_locked(); registrarConteo_locked(true); }
                        }
                    }
                }
            }
            if (d > g_cfg.MAX) {
                ESP_LOGW(TAG, "DROP,type=MIX,reason=MAX,d=%lld", (long long)d);
                estadoActual = REPOSO;
                reset_dir_locked();
            } else if (todos_inactivos_locked() && d > g_cfg.HOLD) {
                ESP_LOGI(TAG, "DROP,type=MIX,reason=IDLE,d=%lld", (long long)d);
                estadoActual = REPOSO;
                reset_dir_locked();
            }

        } else {
            estadoActual = REPOSO;
            reset_dir_locked();
        }

        if (estadoActual != estadoAnterior) {
            ESP_LOGI(TAG, "FSM,%s->%s,t=%lld,tA=%lld,tB=%lld",
                estado_str(estadoAnterior), estado_str(estadoActual),
                (long long)t, (long long)col_det_ts[COL_A], (long long)col_det_ts[COL_B]);
            estadoAnterior = estadoActual;
        }

        xSemaphoreGive(g_lock);
        VDELAY_MS(5);
    }
}