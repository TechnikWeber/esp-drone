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
 * mpu6500.h - MPU6500 / MPU9250 6-axis IMU on SPI (register map rev 2.1).
 */

#ifndef MPU6500_H_
#define MPU6500_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* Registers used by this driver */
#define MPU6500_RA_SMPLRT_DIV        0x19
#define MPU6500_RA_CONFIG            0x1A
#define MPU6500_RA_GYRO_CONFIG       0x1B
#define MPU6500_RA_ACCEL_CONFIG      0x1C
#define MPU6500_RA_ACCEL_CONFIG2     0x1D
#define MPU6500_RA_INT_ENABLE        0x38
#define MPU6500_RA_INT_STATUS        0x3A
#define MPU6500_RA_ACCEL_XOUT_H      0x3B
#define MPU6500_RA_SIGNAL_PATH_RESET 0x68
#define MPU6500_RA_USER_CTRL         0x6A
#define MPU6500_RA_PWR_MGMT_1        0x6B
#define MPU6500_RA_PWR_MGMT_2        0x6C
#define MPU6500_RA_WHO_AM_I          0x75

/* Known WHO_AM_I answers of register compatible chips */
#define MPU6500_WHO_AM_I_MPU6500     0x70
#define MPU6500_WHO_AM_I_MPU9250     0x71
#define MPU6500_WHO_AM_I_MPU9255     0x73
#define MPU6500_WHO_AM_I_MPU6515     0x74

/* Full scale: +-2000 deg/s and +-16 g, as the MPU6050 path of this firmware */
#define MPU6500_GYRO_FS_2000         (3 << 3)
#define MPU6500_ACCEL_FS_16          (3 << 3)
#define MPU6500_DEG_PER_LSB_2000     (2000.0f / 32768.0f)
#define MPU6500_G_PER_LSB_16         (16.0f / 32768.0f)

/* Digital low pass filter settings (DLPF_CFG / A_DLPF_CFG), 1 kHz internal rate */
#define MPU6500_GYRO_DLPF_250HZ      0   // gyro 250 Hz, 8 kHz internal rate!
#define MPU6500_GYRO_DLPF_184HZ      1
#define MPU6500_GYRO_DLPF_92HZ       2
#define MPU6500_GYRO_DLPF_41HZ       3
#define MPU6500_ACCEL_DLPF_218HZ     1
#define MPU6500_ACCEL_DLPF_99HZ      2
#define MPU6500_ACCEL_DLPF_45HZ      3

/* Accel XYZ, temperature, gyro XYZ: 14 bytes, big endian, from ACCEL_XOUT_H */
#define MPU6500_MOTION_LEN           14

/**
 * Attach the IMU to the shared SPI bus (spi_bus.h). Does not talk to the chip.
 * @param csPin        chip select GPIO
 * @param readClockHz  SPI clock for sensor reads (max 20 MHz; writes use 1 MHz)
 */
esp_err_t mpu6500Init(int csPin, uint32_t readClockHz);

/**
 * Read WHO_AM_I. Returns true for one of the known register compatible chips.
 * @param whoAmI  receives the raw answer (for diagnostics), may be NULL
 */
bool mpu6500TestConnection(uint8_t *whoAmI);

/**
 * Reset the chip and configure it: SPI only, PLL clock, all axes on,
 * +-2000 deg/s, +-16 g, 1 kHz output rate, gyro 92 Hz / accel 99 Hz DLPF.
 */
esp_err_t mpu6500Configure(void);

/** Change the digital low pass filters (see MPU6500_*_DLPF_*). */
esp_err_t mpu6500SetDlpf(uint8_t gyroDlpf, uint8_t accelDlpf);

/** Burst read of accel, temperature and gyro (MPU6500_MOTION_LEN bytes). */
esp_err_t mpu6500ReadMotion(uint8_t *buf);

/** Register access at the slow (1 MHz) clock. */
esp_err_t mpu6500WriteReg(uint8_t reg, uint8_t value);
esp_err_t mpu6500ReadRegs(uint8_t reg, uint8_t *data, size_t len);

#endif /* MPU6500_H_ */
