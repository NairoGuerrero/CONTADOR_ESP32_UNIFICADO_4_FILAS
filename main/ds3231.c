// ============================================================================
//  ds3231.c — Driver DS3231 RTC via I2C (solo P1 lo inicializa/usa)
// ============================================================================

#include "ds3231.h"
#include "contador_tipos.h"
#include "config.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

// ── Estado interno (static) ──
static i2c_master_bus_handle_t  i2c_bus_handle;
static i2c_master_dev_handle_t  ds3231_dev_handle;

// ── Estado compartido ──
bool rtc_initialized = false;

// ── BCD helpers ──
static uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

// ============================================================================
//  Inicialización
// ============================================================================

esp_err_t ds3231_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port            = I2C_NUM_0,
        .sda_io_num          = I2C_SDA_PIN,
        .scl_io_num          = I2C_SCL_PIN,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RTC: Error creando bus I2C: %s", esp_err_to_name(err));
        return err;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = DS3231_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &ds3231_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RTC: Error añadiendo DS3231: %s", esp_err_to_name(err));
        return err;
    }
    rtc_initialized = true;
    ESP_LOGI(TAG, "RTC: DS3231 inicializado SDA=%d SCL=%d", I2C_SDA_PIN, I2C_SCL_PIN);
    return ESP_OK;
}

// ============================================================================
//  Leer hora
// ============================================================================

esp_err_t ds3231_leer_hora(int *anio, int *mes, int *dia,
                           int *hora, int *min, int *seg) {
    if (!rtc_initialized) return ESP_ERR_INVALID_STATE;
    uint8_t reg_addr = 0x00;
    uint8_t data[7];
    esp_err_t err = i2c_master_transmit_receive(
        ds3231_dev_handle, &reg_addr, 1, data, 7, 100);
    if (err != ESP_OK) return err;
    *seg  = bcd_to_dec(data[0] & 0x7F);
    *min  = bcd_to_dec(data[1] & 0x7F);
    *hora = bcd_to_dec(data[2] & 0x3F);
    *dia  = bcd_to_dec(data[4] & 0x3F);
    *mes  = bcd_to_dec(data[5] & 0x1F);
    *anio = bcd_to_dec(data[6]) + 2000;
    return ESP_OK;
}

// ============================================================================
//  Configurar hora
// ============================================================================

esp_err_t ds3231_configurar_hora(int anio, int mes, int dia,
                                 int dia_sem,
                                 int hora, int min, int seg) {
    if (!rtc_initialized) return ESP_ERR_INVALID_STATE;
    uint8_t data[8];
    data[0] = 0x00;
    data[1] = dec_to_bcd(seg);
    data[2] = dec_to_bcd(min);
    data[3] = dec_to_bcd(hora);
    data[4] = dec_to_bcd(dia_sem);
    data[5] = dec_to_bcd(dia);
    data[6] = dec_to_bcd(mes);
    data[7] = dec_to_bcd(anio - 2000);
    esp_err_t err = i2c_master_transmit(ds3231_dev_handle, data, 8, 100);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "RTC: Hora configurada: %04d%02d%02d %02d:%02d:%02d",
                 anio, mes, dia, hora, min, seg);
    }
    return err;
}
