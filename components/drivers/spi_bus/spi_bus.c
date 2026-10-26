/*
 * ESP-Drone Firmware
 *
 * Copyright 2026 Philipp Weber
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * spi_bus.c - the board's single SPI bus, shared by several devices.
 */

#include <stdbool.h>

#include "spi_bus.h"
#include "sdkconfig.h"
#include "esp_log.h"

#ifdef CONFIG_EXT_FLOW_TESTBOARD // this test board swaps SCK and MOSI
#define SPI_BUS_SCK_PIN  CONFIG_SPI_PIN_MOSI
#define SPI_BUS_MOSI_PIN CONFIG_SPI_PIN_CLK
#else
#define SPI_BUS_SCK_PIN  CONFIG_SPI_PIN_CLK
#define SPI_BUS_MOSI_PIN CONFIG_SPI_PIN_MOSI
#endif
#define SPI_BUS_MISO_PIN CONFIG_SPI_PIN_MISO

static const char *TAG = "SPI_BUS";
static bool isInit = false;

esp_err_t spiBusInit(void)
{
    if (isInit) {
        return ESP_OK;
    }

    const spi_bus_config_t buscfg = {
        .miso_io_num = SPI_BUS_MISO_PIN,
        .mosi_io_num = SPI_BUS_MOSI_PIN,
        .sclk_io_num = SPI_BUS_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        // All transfers are short register accesses (at most 64 bytes), so the
        // bus runs without DMA. That keeps polling transfers fast and avoids
        // DMA buffer alignment rules.
        .max_transfer_sz = 64,
    };

    esp_err_t err = spi_bus_initialize(SPI_BUS_HOST, &buscfg, SPI_DMA_DISABLED);
    if (err == ESP_ERR_INVALID_STATE) {
        // Someone else initialized the host already; that is fine as long as
        // it used these pins, which is the case for all code in this project.
        err = ESP_OK;
    }

    if (err == ESP_OK) {
        isInit = true;
        ESP_LOGI(TAG, "SPI bus ready (CLK %d, MISO %d, MOSI %d)",
                 SPI_BUS_SCK_PIN, SPI_BUS_MISO_PIN, SPI_BUS_MOSI_PIN);
    } else {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
    }

    return err;
}
