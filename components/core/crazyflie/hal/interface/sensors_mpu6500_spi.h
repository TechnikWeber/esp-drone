/*
 * ESP-Drone Firmware
 *
 * Copyright 2026 Philipp Weber
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2018 Bitcraze AB
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
 * sensors_mpu6500_spi.h - sensor HAL for an MPU6500 on SPI without
 * interrupt line (E1 Mini).
 */

#ifndef __SENSORS_MPU6500_SPI_H__
#define __SENSORS_MPU6500_SPI_H__

#include "sensors.h"

void sensorsMpu6500SpiInit(void);
bool sensorsMpu6500SpiTest(void);
bool sensorsMpu6500SpiAreCalibrated(void);
bool sensorsMpu6500SpiManufacturingTest(void);
void sensorsMpu6500SpiAcquire(sensorData_t *sensors, const uint32_t tick);
void sensorsMpu6500SpiWaitDataReady(void);
bool sensorsMpu6500SpiReadGyro(Axis3f *gyro);
bool sensorsMpu6500SpiReadAcc(Axis3f *acc);
bool sensorsMpu6500SpiReadMag(Axis3f *mag);
bool sensorsMpu6500SpiReadBaro(baro_t *baro);
void sensorsMpu6500SpiSetAccMode(accModes accMode);

#endif // __SENSORS_MPU6500_SPI_H__
