// ============================================================================
//  config.c — Configuración NVS con versionado, identidad de puerta
// ============================================================================

#include "config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

// ── Definición de globals compartidos ──
int               puerta_id = 1;
config_t          g_cfg;
SemaphoreHandle_t g_lock        = NULL;
SemaphoreHandle_t g_uart_tx_lock = NULL;
const char       *TAG       = "P1";

// ============================================================================
//  Helpers
// ============================================================================

int clamp_int(int v, int mn, int mx, int dv) {
    return (v < mn || v > mx) ? dv : v;
}

config_t default_config(void) {
    return (config_t){
        .cam  = 2,   .ch    = 1,
        .tb_s = DEF_TB_S,
        .LS   = DEF_LS,  .MIN  = DEF_MIN,  .MAX  = DEF_MAX,
        .REF  = DEF_REF, .SIM  = DEF_SIM,
        .DET  = DEF_DET, .CONFU= DEF_CONFU,.CONFD= DEF_CONFD,
        .STU  = DEF_STU, .STD  = DEF_STD,  .HOLD = DEF_HOLD,
        .ptrasera = 0
    };
}

void apply_sanity(config_t *c) {
    if (c->cam != 2 && c->cam != 3) c->cam = 2;
    c->ch    = (c->ch == 0) ? 0 : 1;
    c->tb_s  = clamp_int(c->tb_s, MIN_TB_S, MAX_TB_S, DEF_TB_S);
    c->LS    = clamp_int(c->LS,   MIN_LS,   MAX_LS,   DEF_LS);
    c->MIN   = clamp_int(c->MIN,  MIN_MIN,  MAX_MIN,  DEF_MIN);
    c->MAX   = clamp_int(c->MAX,  MIN_MAX,  MAX_MAX,  DEF_MAX);
    c->REF   = clamp_int(c->REF,  MIN_REF,  MAX_REF,  DEF_REF);
    c->SIM   = clamp_int(c->SIM,  MIN_SIM,  MAX_SIM,  DEF_SIM);
    c->DET   = clamp_int(c->DET,  MIN_DET,  MAX_DET,  DEF_DET);
    c->CONFU = clamp_int(c->CONFU,MIN_CONF, MAX_CONF, DEF_CONFU);
    c->CONFD = clamp_int(c->CONFD,MIN_CONF, MAX_CONF, DEF_CONFD);
    c->STU   = clamp_int(c->STU,  MIN_STREAK,MAX_STREAK,DEF_STU);
    c->STD   = clamp_int(c->STD,  MIN_STREAK,MAX_STREAK,DEF_STD);
    c->HOLD  = clamp_int(c->HOLD, MIN_HOLD, MAX_HOLD, DEF_HOLD);
    if (c->ptrasera != 1 && c->ptrasera != 0) c->ptrasera = 0;
}

// ============================================================================
//  NVS — Configuración con versionado
// ============================================================================

void config_load(void) {
    nvs_handle_t h;
    g_cfg = default_config();
    esp_err_t err = nvs_open("pcfg", NVS_READONLY, &h);
    if (err == ESP_OK) {
        // Verificar versión de config
        int32_t ver = 0;
        nvs_get_i32(h, "ver", &ver);

        size_t sz = sizeof(g_cfg);
        err = nvs_get_blob(h, "cfg", &g_cfg, &sz);
        nvs_close(h);

        if (err != ESP_OK || sz != sizeof(g_cfg) || ver != CONFIG_VERSION) {
            ESP_LOGW(TAG, "NVS: Config incompatible (ver=%d, esperada=%d), usando defaults",
                     (int)ver, CONFIG_VERSION);
            g_cfg = default_config();
            config_save();  // Guardar defaults con versión correcta
        }
    }
    apply_sanity(&g_cfg);
}

void config_save(void) {
    nvs_handle_t h;
    if (nvs_open("pcfg", NVS_READWRITE, &h) == ESP_OK) {
        apply_sanity(&g_cfg);
        nvs_set_i32(h, "ver", CONFIG_VERSION);
        nvs_set_blob(h, "cfg", &g_cfg, sizeof(g_cfg));
        nvs_commit(h);
        nvs_close(h);
    }
}
void config_save_copy(const config_t *cfg) {
    config_t tmp = *cfg;
    apply_sanity(&tmp);
    nvs_handle_t h;
    if (nvs_open("pcfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "ver", CONFIG_VERSION);
        nvs_set_blob(h, "cfg", &tmp, sizeof(tmp));
        nvs_commit(h);
        nvs_close(h);
    }
}

// ============================================================================
//  NVS — Identidad de puerta
// ============================================================================

void puerta_id_load(void) {
    nvs_handle_t h;
    puerta_id = 1;
    if (nvs_open("dev", NVS_READONLY, &h) == ESP_OK) {
        int32_t val = 1;
        if (nvs_get_i32(h, "pid", &val) == ESP_OK) {
            if (val == 2) puerta_id = 2;
        }
        nvs_close(h);
    }
}

void puerta_id_save(int pid) {
    nvs_handle_t h;
    if (nvs_open("dev", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "pid", (int32_t)pid);
        nvs_commit(h);
        nvs_close(h);
    }
}