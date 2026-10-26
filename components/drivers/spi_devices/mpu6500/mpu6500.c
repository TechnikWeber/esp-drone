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
 * mpu6500.c - MPU6500 / MPU9250 6-axis IMU on SPI.
 *
 * The chip allows 1 MHz for register writes and up to 20 MHz for reading the
 * sensor registers, so the driver attaches two SPI devices with different
 * clocks and drives the shared chip select pin itself.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#include "spi_bus.h"
#include "mpu6500.h"

#define MPU6500_SLOW_CLOCK_HZ  (1 * 1000 * 1000)
#define MPU6500_READ_FLAG      0x80
#define MPU6500_MAX_READ_LEN   31

#define USER_CTRL_I2C_IF_DIS   0x10
#define PWR_MGMT_1_H_RESET     0x80
#define PWR_MGMT_1_CLK_PLL     0x01

static const char *TAG = "MPU6500";

static spi_device_handle_t devSlow;
static spi_device_handle_t devFast;
static int csPin = -1;
static bool isInit = false;

static esp_err_t transfer(spi_device_handle_t dev, const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    // Hold the bus for the whole chip select window so no other device on the
    // bus (PMW3901) can transfer while the IMU is selected.
    esp_err_t err = spi_device_acquire_bus(dev, portMAX_DELAY);
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level(csPin, 0);
    err = spi_device_polling_transmit(dev, &t);
    esp_rom_delay_us(1); // chip select hold time after the last clock edge
    gpio_set_level(csPin, 1);

    spi_device_release_bus(dev);
    return err;
}

static esp_err_t readRegsWith(spi_device_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx[1 + MPU6500_MAX_READ_LEN] = {0};
    uint8_t rx[1 + MPU6500_MAX_READ_LEN] = {0};

    if (!isInit) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || len == 0 || len > MPU6500_MAX_READ_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    tx[0] = reg | MPU6500_READ_FLAG;
    esp_err_t err = transfer(dev, tx, rx, len + 1);
    if (err == ESP_OK) {
        memcpy(data, &rx[1], len);
    }
    return err;
}

esp_err_t mpu6500WriteReg(uint8_t reg, uint8_t value)
{
    if (!isInit) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t tx[2] = {reg & (uint8_t)~MPU6500_READ_FLAG, value};
    uint8_t rx[2];
    return transfer(devSlow, tx, rx, sizeof(tx));
}

esp_err_t mpu6500ReadRegs(uint8_t reg, uint8_t *data, size_t len)
{
    return readRegsWith(devSlow, reg, data, len);
}

esp_err_t mpu6500ReadMotion(uint8_t *buf)
{
    return readRegsWith(devFast, MPU6500_RA_ACCEL_XOUT_H, buf, MPU6500_MOTION_LEN);
}

esp_err_t mpu6500Init(int cs, uint32_t readClockHz)
{
    if (isInit) {
        return ESP_OK;
    }

    esp_err_t err = spiBusInit();
    if (err != ESP_OK) {
        return err;
    }

    // Chip select idles high. Set the level before switching to output so the
    // line never glitches low.
    csPin = cs;
    gpio_set_level(csPin, 1);
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << csPin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(csPin, 1);

    spi_device_interface_config_t devcfg = {
        .mode = 3,               // CPOL = 1, CPHA = 1 (mode 0 works as well)
        .clock_speed_hz = MPU6500_SLOW_CLOCK_HZ,
        .spics_io_num = -1,      // chip select is driven by this driver
        .queue_size = 1,
    };
    err = spi_bus_add_device(SPI_BUS_HOST, &devcfg, &devSlow);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add slow device failed: %s", esp_err_to_name(err));
        return err;
    }

    devcfg.clock_speed_hz = (int)readClockHz;
    err = spi_bus_add_device(SPI_BUS_HOST, &devcfg, &devFast);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add fast device failed: %s", esp_err_to_name(err));
        spi_bus_remove_device(devSlow);
        return err;
    }

    isInit = true;
    ESP_LOGI(TAG, "attached (CS GPIO%d, read clock %lu Hz)", csPin, (unsigned long)readClockHz);
    return ESP_OK;
}

bool mpu6500TestConnection(uint8_t *whoAmI)
{
    uint8_t id = 0;

    if (mpu6500ReadRegs(MPU6500_RA_WHO_AM_I, &id, 1) != ESP_OK) {
        return false;
    }
    if (whoAmI) {
        *whoAmI = id;
    }

    switch (id) {
    case MPU6500_WHO_AM_I_MPU6500:
    case MPU6500_WHO_AM_I_MPU9250:
    case MPU6500_WHO_AM_I_MPU9255:
    case MPU6500_WHO_AM_I_MPU6515:
        return true;
    default:
        return false;
    }
}

esp_err_t mpu6500SetDlpf(uint8_t gyroDlpf, uint8_t accelDlpf)
{
    // CONFIG: FIFO_MODE = 0, EXT_SYNC_SET = 0, DLPF_CFG = gyroDlpf
    esp_err_t err = mpu6500WriteReg(MPU6500_RA_CONFIG, gyroDlpf & 0x07);
    if (err == ESP_OK) {
        // ACCEL_CONFIG2: ACCEL_FCHOICE_B = 0, A_DLPF_CFG = accelDlpf
        err = mpu6500WriteReg(MPU6500_RA_ACCEL_CONFIG2, accelDlpf & 0x07);
    }
    return err;
}

esp_err_t mpu6500Configure(void)
{
    esp_err_t err;

#define WRITE_OR_RETURN(reg, val)                     \
    do {                                              \
        err = mpu6500WriteReg((reg), (val));          \
        if (err != ESP_OK) {                          \
            return err;                               \
        }                                             \
    } while (0)

    // Device reset. The register map asks for 100 ms afterwards and, in SPI
    // mode, for a reset of the signal paths.
    WRITE_OR_RETURN(MPU6500_RA_PWR_MGMT_1, PWR_MGMT_1_H_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));
    WRITE_OR_RETURN(MPU6500_RA_SIGNAL_PATH_RESET, 0x07);
    vTaskDelay(pdMS_TO_TICKS(100));

    // SPI only: stop the I2C slave from interpreting bus noise.
    WRITE_OR_RETURN(MPU6500_RA_USER_CTRL, USER_CTRL_I2C_IF_DIS);

    // Wake up with the PLL as clock source, all six axes enabled.
    WRITE_OR_RETURN(MPU6500_RA_PWR_MGMT_1, PWR_MGMT_1_CLK_PLL);
    vTaskDelay(pdMS_TO_TICKS(20));
    WRITE_OR_RETURN(MPU6500_RA_PWR_MGMT_2, 0x00);

    // 1 kHz output rate, full scale ranges, filters, no interrupts.
    WRITE_OR_RETURN(MPU6500_RA_SMPLRT_DIV, 0);
    WRITE_OR_RETURN(MPU6500_RA_GYRO_CONFIG, MPU6500_GYRO_FS_2000);
    WRITE_OR_RETURN(MPU6500_RA_ACCEL_CONFIG, MPU6500_ACCEL_FS_16);
    err = mpu6500SetDlpf(MPU6500_GYRO_DLPF_92HZ, MPU6500_ACCEL_DLPF_99HZ);
    if (err != ESP_OK) {
        return err;
    }
    WRITE_OR_RETURN(MPU6500_RA_INT_ENABLE, 0x00);

#undef WRITE_OR_RETURN

    // Read back: if the writes did not land, the wiring or SPI mode is wrong.
    uint8_t gyroCfg = 0, accCfg = 0;
    err = mpu6500ReadRegs(MPU6500_RA_GYRO_CONFIG, &gyroCfg, 1);
    if (err == ESP_OK) {
        err = mpu6500ReadRegs(MPU6500_RA_ACCEL_CONFIG, &accCfg, 1);
    }
    if (err != ESP_OK) {
        return err;
    }
    if ((gyroCfg & 0x18) != MPU6500_GYRO_FS_2000 || (accCfg & 0x18) != MPU6500_ACCEL_FS_16) {
        ESP_LOGE(TAG, "config read back mismatch (gyro 0x%02x, accel 0x%02x)", gyroCfg, accCfg);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}
