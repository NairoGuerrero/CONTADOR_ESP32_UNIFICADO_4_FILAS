// ============================================================================
//  uart_comm.c — Comunicación UART, comandos, envío de datos
//  Correcciones: RTC cache (I2C fuera del lock), watchdog en tasks
// ============================================================================

#include "uart_comm.h"
#include "config.h"
#include "sensores.h"
#include "ds3231.h"
#include "diagnostico.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include <string.h>

// ============================================================================
//  Helpers
// ============================================================================

bool starts_with(const char *s, const char *pfx) {
    return strncmp(s, pfx, strlen(pfx)) == 0;
}

void uart_write_line(uart_port_t u, const char *s) {
    uart_write_bytes(u, s, (int)strlen(s));
    uart_write_bytes(u, "\r\n", 2);
}

void uplink_post_ok(const char *msg, const char *tag_msg) {
    const char *pfx = "AT$POST=1,2,";
    uart_write_bytes(UART_UPLINK_NUM, pfx, (int)strlen(pfx));
    if (tag_msg && tag_msg[0]) uart_write_bytes(UART_UPLINK_NUM, tag_msg, (int)strlen(tag_msg));
    uart_write_bytes(UART_UPLINK_NUM, "+", 1);
    if (msg && msg[0]) uart_write_bytes(UART_UPLINK_NUM, msg, (int)strlen(msg));
    uart_write_bytes(UART_UPLINK_NUM, "\r\n", 2);
}

void notificar_p1(const char *evento) {
    char msg[16];
    snprintf(msg, sizeof(msg), "P2,%s", evento);
    uart_write_line(UART_UPLINK_NUM, msg);
}

void peer_send(const char *cmd) {
    uart_write_line(UART_PEER_NUM, cmd);
}

// ============================================================================
//  RTC Cache — leer I2C FUERA del mutex, usar valores cacheados dentro
// ============================================================================

static int  rtc_c_anio = 2000, rtc_c_mes = 1, rtc_c_dia = 1;
static int  rtc_c_hora = 0, rtc_c_min = 0, rtc_c_seg = 0;
static bool rtc_c_ok   = false;

void rtc_update_cache(void) {
    if (puerta_id != 1) return;
    int a, mo, d, h, mi, s;
    if (ds3231_leer_hora(&a, &mo, &d, &h, &mi, &s) == ESP_OK) {
        rtc_c_anio = a; rtc_c_mes = mo; rtc_c_dia = d;
        rtc_c_hora = h; rtc_c_min = mi; rtc_c_seg = s;
        rtc_c_ok = true;
    } else {
        rtc_c_ok = false;
    }
}

// ============================================================================
//  Enviar datos al módem — usa RTC CACHE (sin I2C dentro del lock)
// ============================================================================

void enviarDatos_locked(bool periodico) {
    char msg[240];
    unsigned id_envio;
    if (periodico) {
        id_envio = 0;
    } else {
        envio_id++;
        id_envio = (unsigned)envio_id;
    }

    if (!rtc_c_ok) {
        snprintf(msg, sizeof(msg),
            "AT$POST=1,2,;22,00000000000000,%06u,%06u,%06u,%06u,%06u,%06u,%u",
            (unsigned)subida, (unsigned)bajada,
            (unsigned)subidaP2, (unsigned)bajadaP2,
            (unsigned)bloqueos, (unsigned)bloqueosP2,
            id_envio);
    } else {
        snprintf(msg, sizeof(msg),
            "AT$POST=1,2,;22,%04d%02d%02d%02d%02d%02d,%06u,%06u,%06u,%06u,%06u,%06u,%u",
            rtc_c_anio, rtc_c_mes, rtc_c_dia,
            rtc_c_hora, rtc_c_min, rtc_c_seg,
            (unsigned)subida, (unsigned)bajada,
            (unsigned)subidaP2, (unsigned)bajadaP2,
            (unsigned)bloqueos, (unsigned)bloqueosP2,
            id_envio);
    }
    uart_write_line(UART_UPLINK_NUM, msg);
    ESP_LOGI(TAG, "ENVIO,id=%u,periodico=%d", id_envio, periodico ? 1 : 0);
}

// ============================================================================
//  Procesador de comandos — unificado P1/P2
// ============================================================================

void procesarComando(const char *cmdline) {
    if (!cmdline) return;
    ESP_LOGI(TAG, "CMD=%s", cmdline );
    if (starts_with(cmdline, "@P")) return;
    if (strncmp(cmdline, "$ERROR", 6) == 0) return;
    if (strncmp(cmdline, "$OK",    3) == 0) return;

    char cmd[UART_LINE_MAX];
    strncpy(cmd, cmdline, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = 0;

    size_t n = strlen(cmd);
    while (n && (cmd[n-1]=='\r' || cmd[n-1]=='\n' || cmd[n-1]==' ' || cmd[n-1]=='\t'))
        { cmd[n-1] = 0; n--; }
    if (n == 0) return;

    char cmd_fwd[UART_LINE_MAX];
    strncpy(cmd_fwd, cmd, sizeof(cmd_fwd) - 1);
    cmd_fwd[sizeof(cmd_fwd) - 1] = 0;

    char *tag     = NULL;
    char *cmdReal = cmd;
    char *sep     = strchr(cmd, '+');
    if (sep) { *sep = '\0'; tag = cmd; cmdReal = sep + 1; }
    const char *ts = (tag && tag[0]) ? tag : "";

    ESP_LOGI(TAG, "CMD,tag=%s,cmd=%s", ts, cmdReal);

    char my_sfx   = (puerta_id == 1) ? '1' : '2';
    char peer_sfx = (puerta_id == 1) ? '2' : '1';

    // ── PUERTA — configura identidad ──
    if (strcmp(cmdReal, "@CCPUERTA-1") == 0) {
        puerta_id_save(1);
        uplink_post_ok("@RC OK+PUERTA=1 REBOOT", ts);
        ESP_LOGI(TAG, "PUERTA configurada a P1 — requiere reboot");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return;
    }
    if (strcmp(cmdReal, "@CCPUERTA-2") == 0) {
        puerta_id_save(2);
        g_cfg.ptrasera = 0; config_save();
        uplink_post_ok("@RC OK+PUERTA=2 REBOOT", ts);
        ESP_LOGI(TAG, "PUERTA configurada a P2 — requiere reboot");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return;
    }

    // ── PUERTA? ──
    if (strcmp(cmdReal, "@CCPUERTA?") == 0) {
        char b[64]; snprintf(b, sizeof(b), "@RC OK+PUERTA=%d,%d", puerta_id,g_cfg.ptrasera);
        uplink_post_ok(b, ts);
        return;
    }

    // ── RESET (P1 only — resetea todo) ──
    if (puerta_id == 1 && strcmp(cmdReal, "@CCRESET") == 0) {
        xSemaphoreTake(g_lock, portMAX_DELAY);
        subida = bajada = bloqueos = subidaP2 = bajadaP2 = bloqueosP2 = 0;
        estadoActual = REPOSO; enRefractario = false;
        reset_dir_locked();
        xSemaphoreGive(g_lock);
        uplink_post_ok("@RC OK+RESET-2P", ts);
        return;
    }

    // ── RESET-N ──
    {
        char reset_cmd[16]; snprintf(reset_cmd, sizeof(reset_cmd), "@CCRESET-%c", my_sfx);
        if (strcmp(cmdReal, reset_cmd) == 0) {
            xSemaphoreTake(g_lock, portMAX_DELAY);
            subida = bajada = bloqueos = 0;
            estadoActual = REPOSO; enRefractario = false;
            reset_dir_locked();
            xSemaphoreGive(g_lock);
            char b[32]; snprintf(b, sizeof(b), "@RC OK+RESET-%c", my_sfx);
            uplink_post_ok(b, ts);
            return;
        }
    }

    // ── RESET peer (P1 resetea contadores P2 locales) ──
    if (puerta_id == 1) {
        char reset_peer[16]; snprintf(reset_peer, sizeof(reset_peer), "@CCRESET-%c", peer_sfx);
        if (strcmp(cmdReal, reset_peer) == 0) {
            xSemaphoreTake(g_lock, portMAX_DELAY);
            subidaP2 = bajadaP2 = bloqueosP2 = 0;
            xSemaphoreGive(g_lock);
            char b[32]; snprintf(b, sizeof(b), "@RC OK+RESET-%c", peer_sfx);
            uplink_post_ok(b, ts);
            return;
        }
    }

    // ── RESETID (P1 only) ──
    if (puerta_id == 1 && strcmp(cmdReal, "@CCRESETID") == 0) {
        xSemaphoreTake(g_lock, portMAX_DELAY);
        envio_id = 0;
        xSemaphoreGive(g_lock);
        uplink_post_ok("@RC OK+RESETID", ts);
        return;
    }

    // ── REBOOT-N ──
    {
        char reboot_cmd[16]; snprintf(reboot_cmd, sizeof(reboot_cmd), "@CCREBOOT-%c", my_sfx);
        if (strcmp(cmdReal, reboot_cmd) == 0) {
            char b[32]; snprintf(b, sizeof(b), "@RC OK+REBOOT-%c", my_sfx);
            uplink_post_ok(b, ts);
            vTaskDelay(pdMS_TO_TICKS(50));
            esp_restart();
            return;
        }
    }

    // ── CFG? ──
    {
        char cfg_cmd[16]; snprintf(cfg_cmd, sizeof(cfg_cmd), "@CCCFG%c?", my_sfx);
        if (strcmp(cmdReal, cfg_cmd) == 0) {
            char b[512];
            xSemaphoreTake(g_lock, portMAX_DELAY);
            if (puerta_id == 1) {
                snprintf(b, sizeof(b),
                    "CFG%c:LS=%d,MIN=%d,MAX=%d,REF=%d,SIM=%d,HOLD=%d,"
                    "DET=%d,CONFU=%d,CONFD=%d,STU=%d,STD=%d,"
                    "TB=%d,CAM=%d,CH=%d,SAMP=%d,EPS=%d,PT?=%d",
                    my_sfx,
                    g_cfg.LS, g_cfg.MIN, g_cfg.MAX, g_cfg.REF, g_cfg.SIM, g_cfg.HOLD,
                    g_cfg.DET, g_cfg.CONFU, g_cfg.CONFD, g_cfg.STU, g_cfg.STD,
                    g_cfg.tb_s, g_cfg.cam, g_cfg.ch,
                    SAMPLES_PER_READ, MIX_EPS_MS, g_cfg.ptrasera);
            } else {
                snprintf(b, sizeof(b),
                    "CFG%c:LS=%d,MIN=%d,MAX=%d,REF=%d,SIM=%d,HOLD=%d,"
                    "DET=%d,CONFU=%d,CONFD=%d,STU=%d,STD=%d,"
                    "TB=%d,CH=%d,SAMP=%d,EPS=%d",
                    my_sfx,
                    g_cfg.LS, g_cfg.MIN, g_cfg.MAX, g_cfg.REF, g_cfg.SIM, g_cfg.HOLD,
                    g_cfg.DET, g_cfg.CONFU, g_cfg.CONFD, g_cfg.STU, g_cfg.STD,
                    g_cfg.tb_s, g_cfg.ch,
                    SAMPLES_PER_READ, MIX_EPS_MS);
            }
            xSemaphoreGive(g_lock);
            uart_write_bytes(UART_UPLINK_NUM, "AT$POST=1,2,", 12);
            if (ts[0]) uart_write_bytes(UART_UPLINK_NUM, ts, (int)strlen(ts));
            uart_write_bytes(UART_UPLINK_NUM, "+@RC OK+", 8);
            uart_write_bytes(UART_UPLINK_NUM, b, (int)strlen(b));
            uart_write_bytes(UART_UPLINK_NUM, "\r\n", 2);
            return;
        }
    }

    // ── CAM (P1 only) ──
    if (puerta_id == 1) {
        char cam_pfx[16]; snprintf(cam_pfx, sizeof(cam_pfx), "@CCCAM%c-", my_sfx);
        if (starts_with(cmdReal, cam_pfx) && strlen(cmdReal) == strlen(cam_pfx) + 1 &&
            (cmdReal[strlen(cam_pfx)] == '2' || cmdReal[strlen(cam_pfx)] == '3')) {
            int v = cmdReal[strlen(cam_pfx)] - '0';
            xSemaphoreTake(g_lock, portMAX_DELAY);
            g_cfg.cam = v; config_save();
            xSemaphoreGive(g_lock);
            char b[64]; snprintf(b, sizeof(b), "@RC OK+CAM%c=%d", my_sfx, v);
            uplink_post_ok(b, ts); return;
        }
    }
    
    // -- PUERTA TRASERA (P1 ONLY) --
    
    if (puerta_id == 1) {
		if (starts_with(cmdReal, "@CCPTRASERA") && (cmdReal[12] == '1' || cmdReal[12] == '0')){
			int v = cmdReal[12] - '0';
			xSemaphoreTake(g_lock, portMAX_DELAY);
			g_cfg.ptrasera = v; config_save();
			xSemaphoreGive(g_lock);
			char b[64]; snprintf(b, sizeof(b), "@RC OK+PTRASERA=%d", v);
            uplink_post_ok(b, ts); return;
		}
	}

    // ── CH ──
    {
        char ch_on[16], ch_off[16];
        snprintf(ch_on, sizeof(ch_on), "@CCCH%c-ON", my_sfx);
        snprintf(ch_off, sizeof(ch_off), "@CCCH%c-OFF", my_sfx);
        if (strcmp(cmdReal, ch_on) == 0) {
            xSemaphoreTake(g_lock, portMAX_DELAY);
            g_cfg.ch = 1; config_save();
            if (alarmaBloqueoActiva) gpio_set_level(PIN_CH, 1);
            xSemaphoreGive(g_lock);
            char b[32]; snprintf(b, sizeof(b), "@RC OK+CH%c-ON", my_sfx);
            uplink_post_ok(b, ts); return;
        }
        if (strcmp(cmdReal, ch_off) == 0) {
            xSemaphoreTake(g_lock, portMAX_DELAY);
            g_cfg.ch = 0; config_save();
            gpio_set_level(PIN_CH, 0);
            xSemaphoreGive(g_lock);
            char b[32]; snprintf(b, sizeof(b), "@RC OK+CH%c-OFF", my_sfx);
            uplink_post_ok(b, ts); return;
        }
    }

    // ── TB ──
    {
        char tb_pfx[16]; snprintf(tb_pfx, sizeof(tb_pfx), "@CCTB%c-", my_sfx);
        if (starts_with(cmdReal, tb_pfx)) {
            int seg = atoi(cmdReal + strlen(tb_pfx));
            if (seg >= MIN_TB_S && seg <= MAX_TB_S) {
                xSemaphoreTake(g_lock, portMAX_DELAY);
                g_cfg.tb_s = seg; config_save();
                xSemaphoreGive(g_lock);
                char b[64]; snprintf(b, sizeof(b), "@RC OK+TB%c=%d", my_sfx, seg);
                uplink_post_ok(b, ts);
            } else {
                char b[64]; snprintf(b, sizeof(b), "@RC ERROR+TB%c(1..30)", my_sfx);
                uplink_post_ok(b, ts);
            }
            return;
        }
    }

    // ── CFG-KEY-VAL ──
    {
        char cfg_pfx[16]; snprintf(cfg_pfx, sizeof(cfg_pfx), "@CCCFG%c-", my_sfx);
        if (starts_with(cmdReal, cfg_pfx)) {
            char tmp[UART_LINE_MAX];
            strncpy(tmp, cmdReal + strlen(cfg_pfx), sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
            char *dash = strchr(tmp, '-');
            if (!dash) {
                char b[64]; snprintf(b, sizeof(b), "@RC ERROR+CFG%c_FORMAT", my_sfx);
                uplink_post_ok(b, ts); return;
            }
            *dash = 0;
            char key[12]; strncpy(key, tmp, sizeof(key) - 1); key[sizeof(key) - 1] = 0;
            int v = atoi(dash + 1);

            bool ok = false;
            xSemaphoreTake(g_lock, portMAX_DELAY);
            if      (!strcmp(key,"LS")    && v>=MIN_LS     && v<=MAX_LS)     { g_cfg.LS=v;    ok=true; }
            else if (!strcmp(key,"MIN")   && v>=MIN_MIN    && v<=MAX_MIN)    { g_cfg.MIN=v;   ok=true; }
            else if (!strcmp(key,"MAX")   && v>=MIN_MAX    && v<=MAX_MAX)    { g_cfg.MAX=v;   ok=true; }
            else if (!strcmp(key,"REF")   && v>=MIN_REF    && v<=MAX_REF)    { g_cfg.REF=v;   ok=true; }
            else if (!strcmp(key,"SIM")   && v>=MIN_SIM    && v<=MAX_SIM)    { g_cfg.SIM=v;   ok=true; }
            else if (!strcmp(key,"HOLD")  && v>=MIN_HOLD   && v<=MAX_HOLD)   { g_cfg.HOLD=v;  ok=true; }
            else if (!strcmp(key,"DET")   && v>=MIN_DET    && v<=MAX_DET)    { g_cfg.DET=v;   ok=true; }
            else if (!strcmp(key,"CONFU") && v>=MIN_CONF   && v<=MAX_CONF)   { g_cfg.CONFU=v; ok=true; }
            else if (!strcmp(key,"CONFD") && v>=MIN_CONF   && v<=MAX_CONF)   { g_cfg.CONFD=v; ok=true; }
            else if (!strcmp(key,"STU")   && v>=MIN_STREAK && v<=MAX_STREAK) { g_cfg.STU=v;   ok=true; }
            else if (!strcmp(key,"STD")   && v>=MIN_STREAK && v<=MAX_STREAK) { g_cfg.STD=v;   ok=true; }
            if (ok) config_save();
            xSemaphoreGive(g_lock);

            if (ok) {
                char b[120]; snprintf(b, sizeof(b), "@RC OK+CFG%c-%s=%d", my_sfx, key, v);
                uplink_post_ok(b, ts);
            } else {
                char b[64]; snprintf(b, sizeof(b), "@RC ERROR+CFG%c_RANGE", my_sfx);
                uplink_post_ok(b, ts);
            }
            return;
        }
    }

    // ── SETRTC (P1 only) ──
    if (puerta_id == 1 && starts_with(cmdReal, "@CCSETRTC-") && strlen(cmdReal) == 24) {
        const char *rtc_ts = cmdReal + 10;
        int y, mo, d, h, mi, s;
        if (sscanf(rtc_ts, "%4d%2d%2d%2d%2d%2d", &y, &mo, &d, &h, &mi, &s) == 6) {
            esp_err_t err = ds3231_configurar_hora(y, mo, d, 0, h, mi, s);
            if (err == ESP_OK) {
                rtc_update_cache();  // Refrescar cache tras setear hora
                char b[64]; snprintf(b, sizeof(b), "@RC OK+SETRTC=%s", rtc_ts);
                uplink_post_ok(b, ts);
            } else {
                uplink_post_ok("@RC ERROR+SETRTC_I2C", ts);
            }
        } else {
            uplink_post_ok("@RC ERROR+SETRTC_FORMAT", ts);
        }
        return;
    }

    // ── DIAG ON/OFF (propio) ──
    {
        char diag_on[24], diag_off[24];
        if (puerta_id == 1) {
            strcpy(diag_on, "@CCDIAG-ON");
            strcpy(diag_off, "@CCDIAG-OFF");
        } else {
            strcpy(diag_on, "@CCDIAG2-ON");
            strcpy(diag_off, "@CCDIAG2-OFF");
        }
        if (strcmp(cmdReal, diag_on) == 0) {
            diag_start();
            char b[32]; snprintf(b, sizeof(b), "@RC OK+%s", diag_on + 3);
            uplink_post_ok(b, ts); return;
        }
        if (strcmp(cmdReal, diag_off) == 0) {
            diag_stop();
            char b[32]; snprintf(b, sizeof(b), "@RC OK+%s", diag_off + 3);
            uplink_post_ok(b, ts); return;
        }
    }

    // ── Reenvío a peer (P1 only) ──
    if (puerta_id == 1) {
        char reboot_peer[16]; snprintf(reboot_peer, sizeof(reboot_peer), "@CCREBOOT-%c", peer_sfx);
        char ch_on_peer[16];  snprintf(ch_on_peer, sizeof(ch_on_peer),  "@CCCH%c-ON", peer_sfx);
        char ch_off_peer[16]; snprintf(ch_off_peer, sizeof(ch_off_peer), "@CCCH%c-OFF", peer_sfx);
        char cfg_q_peer[16];  snprintf(cfg_q_peer, sizeof(cfg_q_peer),  "@CCCFG%c?", peer_sfx);
        char diag_on_peer[24];  snprintf(diag_on_peer, sizeof(diag_on_peer), "@CCDIAG%c-ON", peer_sfx);
        char diag_off_peer[24]; snprintf(diag_off_peer, sizeof(diag_off_peer), "@CCDIAG%c-OFF", peer_sfx);
        char tb_peer[16]; snprintf(tb_peer, sizeof(tb_peer), "@CCTB%c-", peer_sfx);
        char cfg_peer[16]; snprintf(cfg_peer, sizeof(cfg_peer), "@CCCFG%c-", peer_sfx);
        char cam_peer[16]; snprintf(cam_peer, sizeof(cam_peer), "@CCCAM%c-", peer_sfx);

        if (strcmp(cmdReal, reboot_peer)==0 || strcmp(cmdReal, ch_on_peer)==0 ||
            strcmp(cmdReal, ch_off_peer)==0 || strcmp(cmdReal, cfg_q_peer)==0 ||
            strcmp(cmdReal, diag_on_peer)==0 || strcmp(cmdReal, diag_off_peer)==0 ||
            starts_with(cmdReal, tb_peer) || starts_with(cmdReal, cfg_peer) ||
            starts_with(cmdReal, cam_peer)) {
            peer_send(cmd_fwd);
            return;
        }
    }

    // ── Desconocido ──
    char b[280]; snprintf(b, sizeof(b), "@RC ERROR+CMD:%s", cmdReal);
    uplink_post_ok(b, ts);
}

// ============================================================================
//  TASK: UART uplink
// ============================================================================

void task_uart_uplink(void *arg) {
    (void)arg;
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "TASK,uart_uplink,start");

    uint8_t ch; char line[UART_LINE_MAX]; int idx = 0;

    while (1) {
        esp_task_wdt_reset();
        int n = uart_read_bytes(UART_UPLINK_NUM, &ch, 1, pdMS_TO_TICKS(20));
        if (n == 1) {
            if (ch == '\n' || ch == '\r') {
                if (idx > 0) { line[idx] = 0; procesarComando(line); idx = 0; }
            } else if (idx < (UART_LINE_MAX - 1)) line[idx++] = (char)ch;
        }
    }
}

// ============================================================================
//  TASK: UART peer (solo P1)
// ============================================================================

void task_uart_peer(void *arg) {
    (void)arg;
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "TASK,uart_peer,start");

    uint8_t ch; char line[UART_LINE_MAX]; int idx = 0;

    while (1) {
        esp_task_wdt_reset();
        int n = uart_read_bytes(UART_PEER_NUM, &ch, 1, pdMS_TO_TICKS(20));
        if (n == 1) {
            if (ch == '\n' || ch == '\r') {
                if (idx > 0) {
                    line[idx] = 0;
                    if (starts_with(line, "P2,")) {
                        const char *tipo = line + 3;
                        // Refrescar RTC cache ANTES del lock
                        rtc_update_cache();
                        xSemaphoreTake(g_lock, portMAX_DELAY);
                        if (starts_with(tipo, "UP")) {
                            subidaP2++;
                            char b[64]; snprintf(b, sizeof(b), "AT$GCAM=0,%d", g_cfg.cam);
                            uart_write_line(UART_UPLINK_NUM, b);
                            enviarDatos_locked(false);
                            ESP_LOGI(TAG, "P2_EVT,type=UP,sub2=%u", (unsigned)subidaP2);
                        } else if (starts_with(tipo, "DN")) {
                            bajadaP2++; enviarDatos_locked(false);
                            ESP_LOGI(TAG, "P2_EVT,type=DN,dn2=%u", (unsigned)bajadaP2);
                        } else if (starts_with(tipo, "BLK")) {
                            bloqueosP2++;
                            char b[64]; snprintf(b, sizeof(b), "AT$GCAM=0,%d", g_cfg.cam);
                            uart_write_line(UART_UPLINK_NUM, b);
                            enviarDatos_locked(false);
                            ESP_LOGI(TAG, "P2_EVT,type=BLK,blk2=%u", (unsigned)bloqueosP2);
                        }
                        xSemaphoreGive(g_lock);
                    } else {
                        uart_write_line(UART_UPLINK_NUM, line);
                    }
                    idx = 0;
                }
            } else if (idx < (UART_LINE_MAX - 1)) line[idx++] = (char)ch;
        }
    }
}

// ============================================================================
//  TASK: Reporte periódico (solo P1)
// ============================================================================

void task_report(void *arg) {
    (void)arg;
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "TASK,report,start");
    int64_t last = now_ms();

    while (1) {
        esp_task_wdt_reset();
        int64_t t = now_ms();
        if (dt_ms(t, last) >= TIEMPO_REPORTE_MS) {
            // Refrescar RTC cache ANTES del lock
            rtc_update_cache();
            xSemaphoreTake(g_lock, portMAX_DELAY);
            enviarDatos_locked(true);
            xSemaphoreGive(g_lock);
            last = t;
            ESP_LOGI(TAG, "REPORT,t=%lld,sub=%u,dn=%u,blk=%u",
                (long long)t, (unsigned)subida, (unsigned)bajada, (unsigned)bloqueos);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}