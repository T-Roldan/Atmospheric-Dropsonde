#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"

static const char *TAG = "DROPSONDE";

// --- Pin Definitions (ESP32-C3 SuperMini) ---
#define LORA_SCK    4
#define LORA_MISO   5
#define LORA_MOSI   6
#define LORA_NSS    7
#define LORA_RST    10
#define LORA_DIO0   3

#define I2C_SDA     0
#define I2C_SCL     1

#define GPS_RX_PIN  20
#define GPS_TX_PIN  21
#define UART_PORT   UART_NUM_1

// --- Packed 28-Byte Telemetry Struct ---
typedef struct __attribute__((packed)) {
    uint16_t packet_id;
    int32_t  lat_e7;
    int32_t  lon_e7;
    int32_t  gps_alt_mm;
    uint16_t wind_spd_cms;
    uint16_t wind_dir_cdeg;
    uint32_t press_pa;
    int16_t  temp_c100;
    uint16_t hum_rh100;
    uint8_t  sats;
    uint8_t  fix_valid;
} telemetry_packet_t;

// --- BME280 Calibration & Driver ---
typedef struct {
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1, dig_H3; int16_t dig_H2, dig_H4, dig_H5; int8_t dig_H6;
    int32_t  t_fine;
} bme280_calib_t;

static i2c_master_dev_handle_t bme_dev = NULL;
static bme280_calib_t bme_cal;
static bool bme_ready = false;

static esp_err_t bme_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(bme_dev, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t bme_read_regs(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(bme_dev, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t bme280_init(i2c_master_bus_handle_t bus) {
    uint8_t addrs[2] = { 0x76, 0x77 };
    for (int i = 0; i < 2; i++) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = 100000,
        };
        if (i2c_master_bus_add_device(bus, &dev_cfg, &bme_dev) == ESP_OK) {
            uint8_t chip_id = 0;
            if (bme_read_regs(0xD0, &chip_id, 1) == ESP_OK && chip_id == 0x60) {
                ESP_LOGI(TAG, "BME280 found at 0x%02X (Chip ID 0x60)", addrs[i]);
                bme_ready = true;
                break;
            }
            i2c_master_bus_rm_device(bme_dev);
            bme_dev = NULL;
        }
    }
    if (!bme_ready) return ESP_FAIL;

    // Soft reset
    bme_write_reg(0xE0, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read factory calibration registers (0x88..0xA1 and 0xE1..0xE7)
    uint8_t c1[26], c2[7];
    bme_read_regs(0x88, c1, 26);
    bme_read_regs(0xE1, c2, 7);

    bme_cal.dig_T1 = (c1[1] << 8) | c1[0];
    bme_cal.dig_T2 = (c1[3] << 8) | c1[2];
    bme_cal.dig_T3 = (c1[5] << 8) | c1[4];
    bme_cal.dig_P1 = (c1[7] << 8) | c1[6];
    bme_cal.dig_P2 = (c1[9] << 8) | c1[8];
    bme_cal.dig_P3 = (c1[11] << 8) | c1[10];
    bme_cal.dig_P4 = (c1[13] << 8) | c1[12];
    bme_cal.dig_P5 = (c1[15] << 8) | c1[14];
    bme_cal.dig_P6 = (c1[17] << 8) | c1[16];
    bme_cal.dig_P7 = (c1[19] << 8) | c1[18];
    bme_cal.dig_P8 = (c1[21] << 8) | c1[20];
    bme_cal.dig_P9 = (c1[23] << 8) | c1[22];
    bme_cal.dig_H1 = c1[25];
    bme_cal.dig_H2 = (c2[1] << 8) | c2[0];
    bme_cal.dig_H3 = c2[2];
    bme_cal.dig_H4 = (c2[3] << 4) | (c2[4] & 0x0F);
    bme_cal.dig_H5 = (c2[5] << 4) | (c2[4] >> 4);
    bme_cal.dig_H6 = (int8_t)c2[6];

    // Configure oversampling: 1x Humidity, 1x Temp, 1x Pressure, Sleep mode
    bme_write_reg(0xF2, 0x01); // ctrl_hum (must be written before ctrl_meas)
    bme_write_reg(0xF5, 0x00); // config: filter off
    bme_write_reg(0xF4, 0x24); // ctrl_meas: osrs_t=1x, osrs_p=1x, mode=Sleep
    return ESP_OK;
}

static void bme280_read_forced(telemetry_packet_t *pkt) {
    if (!bme_ready) return;

    // Trigger single forced measurement (osrs_t=1x, osrs_p=1x, mode=01 Forced)
    bme_write_reg(0xF4, 0x25);
    vTaskDelay(pdMS_TO_TICKS(15)); // Wait for conversion (~10 ms max at 1x)

    uint8_t raw[8];
    if (bme_read_regs(0xF7, raw, 8) != ESP_OK) return;

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = ((int32_t)raw[6] << 8)  | (int32_t)raw[7];

    // Bosch 32-bit integer compensation formulas
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bme_cal.dig_T1 << 1))) * ((int32_t)bme_cal.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bme_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)bme_cal.dig_T1))) >> 12) * ((int32_t)bme_cal.dig_T3)) >> 14;
    bme_cal.t_fine = var1 + var2;
    pkt->temp_c100 = (int16_t)((bme_cal.t_fine * 5 + 128) >> 8);

    int64_t p_var1 = ((int64_t)bme_cal.t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)bme_cal.dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)bme_cal.dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)bme_cal.dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)bme_cal.dig_P3) >> 8) + ((p_var1 * (int64_t)bme_cal.dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)bme_cal.dig_P1) >> 33;
    if (p_var1 != 0) {
        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - p_var2) * 3125) / p_var1;
        p_var1 = (((int64_t)bme_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p_var2 = (((int64_t)bme_cal.dig_P8) * p) >> 19;
        p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)bme_cal.dig_P7) << 4);
        pkt->press_pa = (uint32_t)(p >> 8);
    }

    int32_t v_x1 = (bme_cal.t_fine - ((int32_t)76800));
    v_x1 = (((((adc_H << 14) - (((int32_t)bme_cal.dig_H4) << 20) - (((int32_t)bme_cal.dig_H5) * v_x1)) + ((int32_t)16384)) >> 15) *
           (((((((v_x1 * ((int32_t)bme_cal.dig_H6)) >> 10) * (((v_x1 * ((int32_t)bme_cal.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
           ((int32_t)bme_cal.dig_H2) + 8192) >> 14));
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)bme_cal.dig_H1)) >> 4));
    v_x1 = (v_x1 < 0 ? 0 : (v_x1 > 419430400 ? 419430400 : v_x1));
    uint32_t h_q22 = (uint32_t)(v_x1 >> 12); // Humidity in % * 1024
    pkt->hum_rh100 = (uint16_t)((h_q22 * 100) >> 10);
}

// --- SX1278 LoRa Register Helpers ---
static spi_device_handle_t lora_spi;

static void lora_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    spi_device_transmit(lora_spi, &t);
}

static uint8_t lora_read_reg(uint8_t reg) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_transmit(lora_spi, &t);
    return rx[1];
}

static esp_err_t lora_init(void) {
    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    spi_bus_config_t buscfg = {
        .miso_io_num = LORA_MISO, .mosi_io_num = LORA_MOSI, .sclk_io_num = LORA_SCK,
        .quadwp_io_num = -1, .quadhd_io_num = -1
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, .mode = 0, .spics_io_num = LORA_NSS, .queue_size = 1
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &lora_spi);

    if (lora_read_reg(0x42) != 0x12) return ESP_FAIL;

    lora_write_reg(0x01, 0x80);
    lora_write_reg(0x06, 0x6C);
    lora_write_reg(0x07, 0x40);
    lora_write_reg(0x08, 0x00);
    lora_write_reg(0x09, 0x8F);
    lora_write_reg(0x1D, 0x72);
    lora_write_reg(0x1E, 0x94);
    lora_write_reg(0x39, 0x12);
    lora_write_reg(0x01, 0x81);
    return ESP_OK;
}

static void lora_send_packet(const uint8_t *data, size_t len) {
    lora_write_reg(0x01, 0x81);
    lora_write_reg(0x0D, 0x00);
    for (size_t i = 0; i < len; i++) {
        lora_write_reg(0x00, data[i]);
    }
    lora_write_reg(0x22, (uint8_t)len);
    lora_write_reg(0x12, 0xFF);
    lora_write_reg(0x01, 0x83);

    while ((lora_read_reg(0x12) & 0x08) == 0) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    lora_write_reg(0x12, 0x08);
}

// --- Minimal NMEA ($GPGGA / $GPRMC) Parser ---
static void parse_nmea_line(char *line, telemetry_packet_t *pkt) {
    if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
        char *tok = strtok(line, ",");
        int field = 0;
        while (tok) {
            if (field == 6) pkt->fix_valid = (atoi(tok) > 0) ? 1 : 0;
            if (field == 7) pkt->sats = (uint8_t)atoi(tok);
            if (field == 9) pkt->gps_alt_mm = (int32_t)(atof(tok) * 1000.0);
            tok = strtok(NULL, ",");
            field++;
        }
    } else if (strncmp(line, "$GPRMC", 6) == 0 || strncmp(line, "$GNRMC", 6) == 0) {
        char *tok = strtok(line, ",");
        int field = 0;
        double raw_lat = 0, raw_lon = 0;
        char ns = 'N', ew = 'E';
        while (tok) {
            if (field == 3) raw_lat = atof(tok);
            if (field == 4) ns = tok[0];
            if (field == 5) raw_lon = atof(tok);
            if (field == 6) ew = tok[0];
            if (field == 7) pkt->wind_spd_cms = (uint16_t)(atof(tok) * 0.514444 * 100.0);
            if (field == 8) pkt->wind_dir_cdeg = (uint16_t)(atof(tok) * 100.0);
            tok = strtok(NULL, ",");
            field++;
        }
        int lat_deg = (int)(raw_lat / 100);
        double lat = lat_deg + (raw_lat - lat_deg * 100) / 60.0;
        if (ns == 'S') lat = -lat;
        pkt->lat_e7 = (int32_t)(lat * 1e7);

        int lon_deg = (int)(raw_lon / 100);
        double lon = lon_deg + (raw_lon - lon_deg * 100) / 60.0;
        if (ew == 'W') lon = -lon;
        pkt->lon_e7 = (int32_t)(lon * 1e7);
    }
}

void app_main(void) {
    // 1. Mount SPIFFS Flash Logger
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs", .partition_label = NULL,
        .max_files = 3, .format_if_mount_failed = true
    };
    esp_vfs_spiffs_register(&conf);

    // 2. Init UART for GY-NEO6MV2
    uart_config_t uart_cfg = {
        .baud_rate = 9600, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_cfg);
    uart_set_pin(UART_PORT, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 3. Init I2C Master & BME280
    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT, .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL, .sda_io_num = I2C_SDA,
        .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true
    };
    i2c_master_bus_handle_t i2c_bus;
    i2c_new_master_bus(&i2c_cfg, &i2c_bus);

    if (bme280_init(i2c_bus) != ESP_OK) {
        ESP_LOGE(TAG, "BME280 not found on GPIO 0/1! Check SDA/SCL wiring.");
    }

    // 4. Init SX1278 LoRa
    if (lora_init() == ESP_OK) {
        ESP_LOGI(TAG, "SX1278 LoRa initialized on 433.0 MHz");
    } else {
        ESP_LOGE(TAG, "SX1278 not detected! Check SPI wiring.");
    }

    telemetry_packet_t pkt = {0};
    uint8_t uart_buf[256];
    int64_t last_tx_us = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT, uart_buf, sizeof(uart_buf) - 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            uart_buf[len] = '\0';
            char *line = strtok((char *)uart_buf, "\r\n");
            while (line) {
                parse_nmea_line(line, &pkt);
                line = strtok(NULL, "\r\n");
            }
        }

        if ((esp_timer_get_time() - last_tx_us) >= 1000000) {
            last_tx_us = esp_timer_get_time();
            pkt.packet_id++;

            bme280_read_forced(&pkt);

            FILE *f = fopen("/spiffs/flight_log.csv", "a");
            if (f) {
                fprintf(f, "%u,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\n",
                        pkt.packet_id, pkt.lat_e7 / 1e7, pkt.lon_e7 / 1e7,
                        pkt.gps_alt_mm / 1000.0, pkt.wind_spd_cms / 100.0,
                        pkt.wind_dir_cdeg / 100.0, pkt.press_pa / 100.0,
                        pkt.temp_c100 / 100.0, pkt.hum_rh100 / 100.0, pkt.sats);
                fclose(f);
            }

            lora_send_packet((uint8_t *)&pkt, sizeof(pkt));
            ESP_LOGI(TAG, "TX #%u | Alt: %.1fm | T: %.2fC | P: %.1fhPa | RH: %.1f%% | Sats: %u",
                     pkt.packet_id, pkt.gps_alt_mm / 1000.0,
                     pkt.temp_c100 / 100.0, pkt.press_pa / 100.0,
                     pkt.hum_rh100 / 100.0, pkt.sats);
        }
    }
}