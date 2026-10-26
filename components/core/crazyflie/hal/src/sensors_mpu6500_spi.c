/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
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
 * sensors_mpu6500_spi.c - sensor HAL for an MPU6500 on SPI (E1 Mini).
 *
 * Derived from sensors_mpu6050_hm5883L_ms5611.c. Changes:
 *  - the IMU is read over SPI (mpu6500.c) instead of I2C;
 *  - the board has no data ready line, so a 1 kHz hardware timer paces the
 *    sampling (the chip runs at a 1 kHz output rate as well);
 *  - the mounting rotation is set in menuconfig (IMU_ROTATION_*);
 *  - a missing IMU makes the self test fail instead of hanging the system task,
 *    and a sensor that stops answering during flight stops the motors.
 */
#include <math.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#include "driver/gptimer.h"

#include "sensors_mpu6500_spi.h"
#include "system.h"
#include "param.h"
#include "log.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "config.h"
#include "stm32_legacy.h"
#include "usec_time.h"
#include "stabilizer.h"

#include "mpu6500.h"
#include "zranger2.h"
#include "flowdeck_v1v2.h"
#include "crtp_commander.h"
#define DEBUG_MODULE "SENSORS"
#include "debug_cf.h"
#include "static_mem.h"

/* Optional sensors of the expansion board, as in the MPU6050 implementation */
#define SENSORS_ENABLE_RANGE_VL53L1X
#define SENSORS_ENABLE_FLOW_PMW3901

#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_16
#define SENSORS_SAMPLE_PERIOD_US  1000

#define SENSORS_ACC_SCALE_SAMPLES 200

#define GYRO_NBR_OF_AXES 3
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(1 * 1000)
// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024
// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE 5000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

// Self test: the accelerometer has to see gravity (any orientation).
#define SENSORS_SELFTEST_ACC_MIN_G 0.7f
#define SENSORS_SELFTEST_ACC_MAX_G 1.3f
#define SENSORS_SELFTEST_TRIES     20

// A dead or disconnected IMU answers with all bits 0 or all bits 1.
#define SENSORS_BAD_SAMPLES_TO_STOP 100    // 100 ms at 1 kHz

#define PITCH_CALIB (CONFIG_PITCH_CALIB * 1.0 / 100)
#define ROLL_CALIB (CONFIG_ROLL_CALIB * 1.0 / 100)

// Low Pass filtering (same values as the MPU6050 implementation)
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30

typedef struct {
    Axis3f bias;
    Axis3f variance;
    Axis3f mean;
    bool isBiasValueFound;
    bool isBufferFilled;
    Axis3i16 *bufHead;
    Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));

static xSemaphoreHandle sensorsDataReady;   // timer -> sensors task
static xSemaphoreHandle dataReady;          // sensors task -> stabilizer
static gptimer_handle_t sampleTimer;

static bool isInit = false;
static bool isMpu6500Present = false;
static bool isMpu6500TestPassed = false;
static uint8_t mpu6500WhoAmI = 0;
static sensorData_t sensorData;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
static BiasObj gyroBiasRunning;
static Axis3f gyroBias;
static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

static uint32_t readErrorCount = 0;
static uint32_t badSampleCount = 0;   // consecutive periods without valid data
static bool imuFailed = false;

static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

#ifdef SENSORS_ENABLE_RANGE_VL53L1X
static bool isVl53l1xPresent = false;
#endif
#ifdef SENSORS_ENABLE_FLOW_PMW3901
static bool isPmw3901Present = false;
#endif

// Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

static uint8_t buffer[MPU6500_MOTION_LEN] = {0};

static void processAccGyroMeasurements(const uint8_t *buffer);
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj *bias);
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj *bias);
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

/* -------------------------------------------------------------------------- */
/* Axis alignment                                                             */
/* -------------------------------------------------------------------------- */

static inline int16_t neg16(int16_t v)
{
    return (v == INT16_MIN) ? INT16_MAX : (int16_t)-v;
}

/* Sensor axes -> body axes (X forward, Y left, Z up), see IMU_ROTATION_*. */
static void alignToBody(int16_t sx, int16_t sy, int16_t sz, Axis3i16 *out)
{
#if defined(CONFIG_IMU_ROTATION_90)
    out->x = neg16(sy);
    out->y = sx;
#elif defined(CONFIG_IMU_ROTATION_180)
    out->x = neg16(sx);
    out->y = neg16(sy);
#elif defined(CONFIG_IMU_ROTATION_270)
    out->x = sy;
    out->y = neg16(sx);
#else
    out->x = sx;
    out->y = sy;
#endif
    out->z = sz;
#ifdef CONFIG_IMU_UPSIDE_DOWN
    out->y = neg16(out->y);
    out->z = neg16(out->z);
#endif
}

static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static bool isDeadSample(const uint8_t *buf)
{
    bool allZero = true;
    bool allOnes = true;
    for (int i = 0; i < MPU6500_MOTION_LEN; i++) {
        allZero &= (buf[i] == 0x00);
        allOnes &= (buf[i] == 0xFF);
    }
    return allZero || allOnes;
}

/* -------------------------------------------------------------------------- */
/* Public read functions                                                      */
/* -------------------------------------------------------------------------- */

bool sensorsMpu6500SpiReadGyro(Axis3f *gyro)
{
    return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsMpu6500SpiReadAcc(Axis3f *acc)
{
    return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsMpu6500SpiReadMag(Axis3f *mag)
{
    return false; // no magnetometer
}

bool sensorsMpu6500SpiReadBaro(baro_t *baro)
{
    return false; // no barometer
}

void sensorsMpu6500SpiAcquire(sensorData_t *sensors, const uint32_t tick)
{
    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsMpu6500SpiAreCalibrated()
{
    return gyroBiasFound;
}

void sensorsMpu6500SpiWaitDataReady(void)
{
    xSemaphoreTake(dataReady, portMAX_DELAY);
}

/* -------------------------------------------------------------------------- */
/* Sampling                                                                   */
/* -------------------------------------------------------------------------- */

static bool IRAM_ATTR sampleTimerIsr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(sensorsDataReady, &woken);
    return woken == pdTRUE;
}

/* Runs in the sensors task, so the timer interrupt lands on FLIGHT_TASK_CORE. */
static bool sampleTimerStart(void)
{
    const gptimer_config_t timerConfig = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 us per tick
    };
    const gptimer_alarm_config_t alarmConfig = {
        .alarm_count = SENSORS_SAMPLE_PERIOD_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    const gptimer_event_callbacks_t callbacks = {
        .on_alarm = sampleTimerIsr,
    };

    if (gptimer_new_timer(&timerConfig, &sampleTimer) != ESP_OK ||
        gptimer_register_event_callbacks(sampleTimer, &callbacks, NULL) != ESP_OK ||
        gptimer_enable(sampleTimer) != ESP_OK ||
        gptimer_set_alarm_action(sampleTimer, &alarmConfig) != ESP_OK ||
        gptimer_start(sampleTimer) != ESP_OK) {
        DEBUG_PRINTE("IMU sample timer could not be started\n");
        return false;
    }

    DEBUG_PRINTI("IMU sample timer running at %d Hz on core %d\n",
                 1000000 / SENSORS_SAMPLE_PERIOD_US, xPortGetCoreID());
    return true;
}

static void sensorsTask(void *param)
{
    systemWaitStart();
    vTaskDelay(M2T(200));

    if (!sampleTimerStart()) {
        // Without samples the stabilizer never sees calibrated sensors and the
        // motors stay off.
        while (1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    while (1) {
        if (pdTRUE != xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {
            continue;
        }

        sensorData.interruptTimestamp = usecTimestamp();

        const bool readOk = mpu6500ReadMotion(buffer) == ESP_OK;
        if (!readOk) {
            readErrorCount++;
        }

        if (readOk && !isDeadSample(buffer)) {
            badSampleCount = 0;
            processAccGyroMeasurements(buffer);
            xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
            xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
        } else if (++badSampleCount == SENSORS_BAD_SAMPLES_TO_STOP && !imuFailed) {
            // No valid data for 100 ms (read errors, or all bits 0 / all 1):
            // the IMU is gone. Stop the motors for good.
            imuFailed = true;
            stabilizerSetEmergencyStop();
            DEBUG_PRINTE("IMU stopped answering, motors stopped\n");
        }

        // Unlock the stabilizer task every period, also without new data (it
        // then keeps the last valid sample). If it waited for data, a set
        // emergency stop would never reach the motors.
        xSemaphoreGive(dataReady);
    }
}

static void processAccGyroMeasurements(const uint8_t *buf)
{
    Axis3f accScaled;

    alignToBody(be16(&buf[0]), be16(&buf[2]), be16(&buf[4]), &accelRaw);
    alignToBody(be16(&buf[8]), be16(&buf[10]), be16(&buf[12]), &gyroRaw);

    // Gyro bias first, once the variance is below the threshold
    gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);

    // Accelerometer scale while the platform is steady
    if (gyroBiasFound) {
        processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
    }

    sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
    sensorData.gyro.y = (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
    sensorData.gyro.z = (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
    applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensorData.gyro);

    accScaled.x = (accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;
    accScaled.y = (accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
    accScaled.z = (accelRaw.z) * SENSORS_G_PER_LSB_CFG / accScale;

    // Compensate for a miss-aligned accelerometer
    sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
    applyAxis3fLpf((lpf2pData *)(&accLpf), &sensorData.acc);
}

/* -------------------------------------------------------------------------- */
/* Init and tests                                                             */
/* -------------------------------------------------------------------------- */

static bool sensorsDeviceInit(void)
{
    // MPU6500 start-up time is 100 ms; give the supply some margin.
    while (xTaskGetTickCount() < M2T(500)) {
        vTaskDelay(M2T(50));
    }

    if (mpu6500Init(CONFIG_IMU_SPI_PIN_CS, CONFIG_IMU_SPI_CLOCK_HZ) != ESP_OK) {
        DEBUG_PRINTE("MPU6500 SPI device could not be attached\n");
        return false;
    }

    for (int i = 0; i < 5 && !isMpu6500Present; i++) {
        isMpu6500Present = mpu6500TestConnection(&mpu6500WhoAmI);
        if (!isMpu6500Present) {
            vTaskDelay(M2T(20));
        }
    }
    if (!isMpu6500Present) {
        DEBUG_PRINTE("MPU6500 SPI connection [FAIL], WHO_AM_I = 0x%02X\n", mpu6500WhoAmI);
        DEBUG_PRINTE("Check CS (GPIO%d) and the SPI pins, then power cycle.\n", CONFIG_IMU_SPI_PIN_CS);
        return false;
    }
    DEBUG_PRINTI("MPU6500 SPI connection [OK], WHO_AM_I = 0x%02X\n", mpu6500WhoAmI);

    if (mpu6500Configure() != ESP_OK) {
        DEBUG_PRINTE("MPU6500 configuration [FAIL]\n");
        isMpu6500Present = false;
        return false;
    }

    for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
    }

#ifdef SENSORS_ENABLE_RANGE_VL53L1X
    zRanger2Init();
    if (zRanger2Test() == true) {
        isVl53l1xPresent = true;
        DEBUG_PRINTI("VL53L1X I2C connection [OK].\n");
    } else {
        DEBUG_PRINTW("VL53L1X I2C connection [FAIL] (no expansion board?).\n");
    }
#endif

#ifdef SENSORS_ENABLE_FLOW_PMW3901
    flowdeck2Init();
    if (flowdeck2Test() == true) {
        isPmw3901Present = true;
        setCommandermode(POSHOLD_MODE);
        DEBUG_PRINTI("PMW3901 SPI connection [OK].\n");
    } else {
        DEBUG_PRINTW("PMW3901 SPI connection [FAIL] (no expansion board?).\n");
    }
#endif

    cosPitch = cosf(PITCH_CALIB * (float)M_PI / 180);
    sinPitch = sinf(PITCH_CALIB * (float)M_PI / 180);
    cosRoll = cosf(ROLL_CALIB * (float)M_PI / 180);
    sinRoll = sinf(ROLL_CALIB * (float)M_PI / 180);
    DEBUG_PRINTI("pitch_calib = %f, roll_calib = %f", PITCH_CALIB, ROLL_CALIB);
    return true;
}

static void sensorsTaskInit(void)
{
    accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
    gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);

    STATIC_MEM_TASK_CREATE_PINNED(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL,
                                  SENSORS_TASK_PRI, FLIGHT_TASK_CORE);
}

void sensorsMpu6500SpiInit(void)
{
    if (isInit) {
        return;
    }

    sensorsDataReady = xSemaphoreCreateBinary();
    dataReady = xSemaphoreCreateBinary();
    sensorsBiasObjInit(&gyroBiasRunning);

    if (!sensorsDeviceInit()) {
        // Leave isInit false: the self test fails, the system does not start
        // and the motors stay off. The LEDs show the failed test pattern.
        return;
    }

    sensorsTaskInit();
    isInit = true;
}

/* Plausibility test: the accelerometer must measure about 1 g. */
static bool sensorsPlausibilityTest(void)
{
    uint8_t buf[MPU6500_MOTION_LEN];

    for (int i = 0; i < SENSORS_SELFTEST_TRIES; i++) {
        if (mpu6500ReadMotion(buf) == ESP_OK && !isDeadSample(buf)) {
            float ax = be16(&buf[0]) * SENSORS_G_PER_LSB_CFG;
            float ay = be16(&buf[2]) * SENSORS_G_PER_LSB_CFG;
            float az = be16(&buf[4]) * SENSORS_G_PER_LSB_CFG;
            float g = sqrtf(ax * ax + ay * ay + az * az);
            if (g > SENSORS_SELFTEST_ACC_MIN_G && g < SENSORS_SELFTEST_ACC_MAX_G) {
                return true;
            }
            DEBUG_PRINTW("MPU6500 self test: |acc| = %.2f g\n", (double)g);
        }
        vTaskDelay(M2T(10));
    }
    return false;
}

bool sensorsMpu6500SpiTest(void)
{
    if (!isInit) {
        DEBUG_PRINTE("MPU6500 not initialized, sensor test [FAIL]\n");
        return false;
    }

    isMpu6500TestPassed = sensorsPlausibilityTest();
    if (!isMpu6500TestPassed) {
        DEBUG_PRINTE("MPU6500 plausibility test [FAIL]\n");
    }
    return isMpu6500TestPassed;
}

bool sensorsMpu6500SpiManufacturingTest(void)
{
    return isMpu6500Present && sensorsPlausibilityTest();
}

void sensorsMpu6500SpiSetAccMode(accModes accMode)
{
    switch (accMode) {
    case ACC_MODE_PROPTEST:
        mpu6500SetDlpf(MPU6500_GYRO_DLPF_184HZ, MPU6500_ACCEL_DLPF_218HZ);
        for (uint8_t i = 0; i < 3; i++) {
            lpf2pInit(&accLpf[i], 1000, 250);
        }
        break;
    case ACC_MODE_FLIGHT:
    default:
        mpu6500SetDlpf(MPU6500_GYRO_DLPF_92HZ, MPU6500_ACCEL_DLPF_99HZ);
        for (uint8_t i = 0; i < 3; i++) {
            lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
        }
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Calibration (unchanged from the MPU6050 implementation)                    */
/* -------------------------------------------------------------------------- */

static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
    static bool accBiasFound = false;
    static uint32_t accScaleSumCount = 0;

    if (!accBiasFound) {
        accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
        accScaleSumCount++;

        if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
            accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
            accBiasFound = true;
        }
    }

    return accBiasFound;
}

static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
    sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

    if (!gyroBiasRunning.isBiasValueFound) {
        sensorsFindBiasValue(&gyroBiasRunning);

        if (gyroBiasRunning.isBiasValueFound) {
            soundSetEffect(SND_CALIB);
            ledseqRun(&seq_calibrated);
            DEBUG_PRINTI("isBiasValueFound!");
        }
    }

    gyroBiasOut->x = gyroBiasRunning.bias.x;
    gyroBiasOut->y = gyroBiasRunning.bias.y;
    gyroBiasOut->z = gyroBiasRunning.bias.z;

    return gyroBiasRunning.isBiasValueFound;
}

static void sensorsBiasObjInit(BiasObj *bias)
{
    bias->isBufferFilled = false;
    bias->bufHead = bias->buffer;
}

static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
    uint32_t i;
    int64_t sum[GYRO_NBR_OF_AXES] = {0};
    int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
        sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
        sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
    }

    varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

    meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
    bias->bufHead->x = x;
    bias->bufHead->y = y;
    bias->bufHead->z = z;
    bias->bufHead++;

    if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

static bool sensorsFindBiasValue(BiasObj *bias)
{
    static int32_t varianceSampleTime;
    bool foundBias = false;

    if (bias->isBufferFilled) {
        sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

        if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
                bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
                bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
                (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
            varianceSampleTime = xTaskGetTickCount();
            bias->bias.x = bias->mean.x;
            bias->bias.y = bias->mean.y;
            bias->bias.z = bias->mean.z;
            foundBias = true;
            bias->isBiasValueFound = true;
        }
    }

    return foundBias;
}

static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out)
{
    Axis3f rx;
    Axis3f ry;

    // Rotate around x-axis
    rx.x = in->x;
    rx.y = in->y * cosRoll - in->z * sinRoll;
    rx.z = in->y * sinRoll + in->z * cosRoll;

    // Rotate around y-axis
    ry.x = rx.x * cosPitch - rx.z * sinPitch;
    ry.y = rx.y;
    ry.z = -rx.x * sinPitch + rx.z * cosPitch;

    out->x = ry.x;
    out->y = ry.y;
    out->z = ry.z;
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
{
    for (uint8_t i = 0; i < 3; i++) {
        in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
    }
}

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, mpu6500, &isMpu6500Present)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, whoAmI, &mpu6500WhoAmI)
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, mpu6500, &isMpu6500TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, pmw3901, &isPmw3901Present)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, vl53l1x, &isVl53l1xPresent)
PARAM_GROUP_STOP(imu_tests)

LOG_GROUP_START(imu_raw)
LOG_ADD(LOG_INT16, gx, &gyroRaw.x)
LOG_ADD(LOG_INT16, gy, &gyroRaw.y)
LOG_ADD(LOG_INT16, gz, &gyroRaw.z)
LOG_ADD(LOG_INT16, ax, &accelRaw.x)
LOG_ADD(LOG_INT16, ay, &accelRaw.y)
LOG_ADD(LOG_INT16, az, &accelRaw.z)
LOG_ADD(LOG_UINT32, readErr, &readErrorCount)
LOG_ADD(LOG_UINT8, failed, &imuFailed)
LOG_GROUP_STOP(imu_raw)
