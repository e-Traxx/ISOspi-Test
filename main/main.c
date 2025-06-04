/*
 * Minimal RDSID self-test for ADBMS6830 + LTC6820 on ESP32-S3
 *
 * Wiring (4-wire SPI):
 *   ESP32-S3 GPIO35 → LTC6820 MISO
 *   ESP32-S3 GPIO36 → LTC6820 MOSI
 *   ESP32-S3 GPIO37 → LTC6820 SCK
 *   ESP32-S3 GPIO38 → LTC6820 CSB      (Port-A, forward)
 *   ESP32-S3 GPIO39 → N/C or 2nd LTC6820 CSB
 *
 * spi clock: 500 kHz, mode3 (CPOL=1 CPHA=1) – conservative but proven reliable.
 *
 * The transaction sequence follows LT/ADI app-note practice:
 *   1.  40 × 0xFF wake-tone  (≈ 320 µs at 1 Mbps, safely longer at 500 kHz)
 *   2.  CS stays low 10 µs   (tREADY)
 *   3.  12-byte frame: 00 2C 59 90 + 8 dummy clocks
 *   4.  Read 8-byte response, verify PEC15
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "string.h"

#define TAG "RDSID_TEST"

/* ----------- user-adjustable pins / parameters -------------------------- */
#define SPI_HOST_D        SPI3_HOST
#define MISO_NUM          35
#define MOSI_NUM          36
#define CLK_NUM           37
#define CS_NUM            38          /* LTC6820 CSB (Port-A) */
#define SPI_FREQ_HZ       500000      /* 500 kHz */
#define WAKE_LEN          40          /* bytes of 0xFF */

/* PEC15 polynomial x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1 (0x4599) */
static uint16_t pec15_table[256];
static void pec15_init(void)
{
    for (int i = 0; i < 256; ++i) {
        uint16_t rem = i << 7;
        for (int b = 0; b < 8; ++b)
            rem = (rem & 0x4000) ? ((rem << 1) ^ 0x4599) : (rem << 1);
        pec15_table[i] = rem & 0x7FFF;
    }
}
static uint16_t pec15_calc(const uint8_t *data, int len)
{
    uint16_t rem = 16;
    while (len--) {
        uint8_t addr = (rem >> 7) ^ *data++;
        rem = ((rem << 8) ^ pec15_table[addr]) & 0x7FFF;
    }
    return (uint16_t)(rem << 1);
}
/* ----------------------------------------------------------------------- */

static spi_device_handle_t dev = NULL;

static esp_err_t spi_init(void)
{
    spi_bus_config_t bus = {
        .miso_io_num = MISO_NUM,
        .mosi_io_num = MOSI_NUM,
        .sclk_io_num = CLK_NUM,
        .max_transfer_sz = 64,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_D, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t ifcfg = {
        .clock_speed_hz = SPI_FREQ_HZ,
        .mode           = 3,
        .spics_io_num   = CS_NUM,
        .queue_size     = 2
    };
    return spi_bus_add_device(SPI_HOST_D, &ifcfg, &dev);
}

static esp_err_t adbms6830_rdsid(uint8_t *out8)
{
    /* 1 – wake-tone */
    uint8_t wake[WAKE_LEN];
    memset(wake, 0xFF, sizeof wake);
    spi_transaction_t t_wake = {
        .length    = sizeof(wake) * 8,
        .tx_buffer = wake
    };
    
    spi_device_polling_transmit(dev, &t_wake);

    esp_rom_delay_us(10);            /* tREADY */

    /* 2 – build 12-byte frame */
    uint8_t frame[12] = { 0 };
    frame[0] = 0x00; frame[1] = 0x2C;               /* opcode */
    uint16_t pec      = pec15_calc(frame, 2);        /* 0x5990 expected */
    frame[2] = pec >> 8;
    frame[3] = pec & 0xFF;                          /* bytes 4-11 stay 0 */

    spi_transaction_t t_cmd = {
        .length    = sizeof frame * 8,
        .tx_buffer = frame,
        .rx_buffer = frame
    };
    spi_device_polling_transmit(dev, &t_cmd);

    memcpy(out8, &frame[4], 8);
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret;
    pec15_init();
    ESP_ERROR_CHECK(spi_init());
    ESP_LOGI(TAG, "SPI ready, clock %d Hz", SPI_FREQ_HZ);

    for (;;)
    {
        uint8_t id[8] = {0};
        ret = adbms6830_rdsid(id);
        if (ret == ESP_OK)
        {
            /* verify returned PEC */
            uint16_t rx_pec = (id[6] << 8) | id[7];
            uint16_t calc   = pec15_calc(id, 6);

            ESP_LOGI(TAG, "RDSID raw: %02X %02X %02X %02X %02X %02X | PEC %04X",
                     id[0], id[1], id[2], id[3], id[4], id[5], rx_pec);

            if (rx_pec == calc)
                ESP_LOGI(TAG, "✓ PASS – Serial-ID PEC matches");
            else
                ESP_LOGE(TAG, "✗ FAIL – bad PEC (expected %04X)", calc);
        }
        else
            ESP_LOGE(TAG, "spi error: %s", esp_err_to_name(ret));

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}