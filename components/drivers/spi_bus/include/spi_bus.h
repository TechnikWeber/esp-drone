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
 * spi_bus.h - the board's single SPI bus, shared by several devices.
 */

#ifndef SPI_BUS_H_
#define SPI_BUS_H_

#include "sdkconfig.h"
#include "esp_err.h"
#include "driver/spi_master.h"

/*
 * All SPI devices of the drone sit on one bus: the IMU on the E1 Mini and the
 * PMW3901 flow sensor of the expansion board. Pins come from menuconfig
 * (SPI_PIN_CLK / SPI_PIN_MISO / SPI_PIN_MOSI). Every device drives its own chip
 * select by GPIO and must hold the bus (spi_device_acquire_bus) while its chip
 * select is low, otherwise two chips could answer at the same time.
 */
#if CONFIG_IDF_TARGET_ESP32
#define SPI_BUS_HOST SPI3_HOST   // VSPI: IO_MUX pins 18/19/23 (E1 Mini)
#else
#define SPI_BUS_HOST SPI2_HOST   // FSPI: IO_MUX pins 35/36/37 on ESP32-S2/S3
#endif

/* Initialize the bus once. Later calls return ESP_OK without doing anything. */
esp_err_t spiBusInit(void);

#endif /* SPI_BUS_H_ */
