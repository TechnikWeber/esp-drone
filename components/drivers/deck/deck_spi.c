/*
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2015 Bitcraze AB
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
 * deck_spi.c - Deck-API SPI communication implementation
 *
 * 2026: the bus is shared with the IMU (spi_bus.c). A deck driver holds the
 * bus from spiBeginTransaction() to spiEndTransaction(), i.e. for as long as
 * its chip select is low. Reads longer than 4 bytes use a real receive buffer.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"

#include "deck_spi.h"
#include "spi_bus.h"
#include "config.h"
#include "cfassert.h"
#include "nvicconf.h"
#define DEBUG_MODULE "DECK_SPI"
#include "debug_cf.h"

static bool isInit = false;
static SemaphoreHandle_t spiMutex;

static spi_device_handle_t spi;

void spiBegin(void)
{
    if (isInit) {
        return;
    }

    if (spiBusInit() != ESP_OK) {
        DEBUG_PRINTE("SPI bus not available\n");
        return;
    }

    spiMutex = xSemaphoreCreateMutex();

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_BAUDRATE_2MHZ,
        .mode = 3,               // PMW3901: CPOL = 1, CPHA = 1
        .spics_io_num = -1,      // chip select is driven by the deck driver
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_add_device(SPI_BUS_HOST, &devcfg, &spi);
    if (ret != ESP_OK || spiMutex == NULL) {
        DEBUG_PRINTE("SPI deck device init failed: %d\n", ret);
        return;
    }

    isInit = true;
}

bool spiTest(void)
{
    return isInit;
}

bool spiExchange(size_t length, bool is_tx, const uint8_t *data_tx, uint8_t *data_rx)
{
    if (isInit != true) {
        return false;
    }

    if (length == 0) {
        return true;    //no need to send anything
    }

    spi_transaction_t t = {
        .length = length * 8,               // Len is in bytes, transaction length is in bits.
        .tx_buffer = is_tx ? data_tx : NULL,
        .rx_buffer = is_tx ? NULL : data_rx,
    };

    return spi_device_polling_transmit(spi, &t) == ESP_OK;
}

void spiBeginTransaction(uint32_t baudRatePrescaler)
{
    // The clock is fixed per device (2 MHz); the argument is kept for the deck API.
    (void)baudRatePrescaler;
    if (!isInit) {
        return;
    }
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    spi_device_acquire_bus(spi, portMAX_DELAY);
}

void spiEndTransaction()
{
    if (!isInit) {
        return;
    }
    spi_device_release_bus(spi);
    xSemaphoreGive(spiMutex);
}
