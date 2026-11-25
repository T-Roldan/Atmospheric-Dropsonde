#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

// --- Pin Definitions (Matches Dropsonde SPI Wiring) ---
#define LORA_SCK    4
#define LORA_MISO   5
#define LORA_MOSI   6
#define LORA_NSS    7
#define LORA_RST    10
#define LORA_DIO0   3

// --- Packed 28-Byte Telemetry Struct (Must match Dropsonde) ---
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

static spi_device_handle_t lora_spi;
static volatile bool packet_received = false;

static void IRAM_ATTR on_rx_done(void *arg) {
    packet_received = true;
}

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

static void lora_start_receive(void) {
    lora_write_reg(0x01, 0x81); // Standby
    lora_write_reg(0x40, 0x00); // DIO0 mapping = RxDone (00)
    lora_write_reg(0x12, 0xFF); // Clear all IRQs
    lora_write_reg(0x0D, 0x00); // Reset FIFO address pointer
    lora_write_reg(0x01, 0x85); // RX Continuous mode
}

static esp_err_t lora_init(void) {
    // Hardware reset SX1278
    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure DIO0 interrupt on rising edge
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LORA_DIO0),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(LORA_DIO0, on_rx_done, NULL);

    // Initialize SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = LORA_MISO,
        .mosi_io_num = LORA_MOSI,
        .sclk_io_num = LORA_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = LORA_NSS,
        .queue_size = 1
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &lora_spi);

    // Verify SX1278 silicon version (0x12)
    if (lora_read_reg(0x42) != 0x12) {
        return ESP_FAIL;
    }

    // Configure LoRa modem to match Dropsonde TX
    lora_write_reg(0x01, 0x80); // Sleep + LoRa mode
    lora_write_reg(0x06, 0x6C); // Frequency: 433.0 MHz (FRF = 0x6C4000)
    lora_write_reg(0x07, 0x40);
    lora_write_reg(0x08, 0x00);
    lora_write_reg(0x0C, 0x23); // LNA max gain + boost on
    lora_write_reg(0x1D, 0x72); // BW 125 kHz, CR 4/5, Explicit header
    lora_write_reg(0x1E, 0x94); // SF9, CRC On
    lora_write_reg(0x26, 0x04); // AgcAutoOn
    lora_write_reg(0x39, 0x12); // Sync word 0x12

    lora_start_receive();
    return ESP_OK;
}

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (lora_init() != ESP_OK) {
        printf("ERR:LoRa init failed (SX1278 not detected on SPI)\n");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Print CSV header for the PC Python logger (exact match to original)
    printf("HEADER:packet_id,fix_valid,lat,lon,gps_alt_m,baro_alt_m,wind_spd_ms,wind_dir_deg,press_hpa,temp_c,hum_pct,sats,rssi_dbm,snr_db\n");
    fflush(stdout);

    while (1) {
        if (packet_received) {
            packet_received = false;

            uint8_t irq = lora_read_reg(0x12);
            bool rx_done = (irq & 0x40) != 0;
            bool crc_err = (irq & 0x20) != 0;

            if (rx_done && !crc_err) {
                uint8_t len = lora_read_reg(0x13); // RegRxNbBytes
                if (len == sizeof(telemetry_packet_t)) {
                    uint8_t fifo_addr = lora_read_reg(0x10); // RegFifoRxCurrentAddr
                    lora_write_reg(0x0D, fifo_addr);

                    telemetry_packet_t pkt;
                    uint8_t *buf = (uint8_t *)&pkt;
                    for (size_t i = 0; i < sizeof(pkt); i++) {
                        buf[i] = lora_read_reg(0x00);
                    }

                    // Calculate SNR (dB) and Packet RSSI (dBm) per SX1278 datasheet (LF port < 525 MHz)
                    int8_t raw_snr = (int8_t)lora_read_reg(0x19);
                    float snr_db = raw_snr / 4.0f;
                    uint8_t raw_rssi = lora_read_reg(0x1A);
                    float rssi_dbm = -164.0f + raw_rssi + (snr_db < 0 ? snr_db : 0.0f);

                    float press_hpa = pkt.press_pa / 100.0f;
                    float baro_alt_m = 44330.0f * (1.0f - powf(press_hpa / 1013.25f, 0.1903f));

                    printf("DATA:%u,%u,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%.1f,%.2f\n",
                           pkt.packet_id,
                           pkt.fix_valid,
                           pkt.lat_e7 / 1e7,
                           pkt.lon_e7 / 1e7,
                           pkt.gps_alt_mm / 1000.0,
                           baro_alt_m,
                           pkt.wind_spd_cms / 100.0,
                           pkt.wind_dir_cdeg / 100.0,
                           press_hpa,
                           pkt.temp_c100 / 100.0,
                           pkt.hum_rh100 / 100.0,
                           pkt.sats,
                           rssi_dbm,
                           snr_db);
                    fflush(stdout);
                }
            }

            // Clear IRQs and re-arm receiver immediately
            lora_start_receive();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}