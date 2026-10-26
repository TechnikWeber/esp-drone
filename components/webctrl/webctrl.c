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
 * webctrl.c - control the drone from a browser.
 *
 *  - Port 80: captive portal. Every unknown URL and every foreign host name is
 *    redirected to the control page, so phones open it by themselves.
 *  - Port 81: WebSocket control channel, in its own server task so a slow page
 *    download or captive portal probes cannot delay the pilot's commands.
 *  - A 100 Hz task turns the pilot's last fresh command (plus trim) into
 *    setpoints and runs the link loss failsafe and the battery protection
 *    (webctrl_core.c). Telemetry goes out at 20 Hz.
 *  - For the desktop ground station (tools/groundstation): 100 Hz samples of
 *    the flight controller, the controller gains (read, change, store) and a
 *    motor test for the bench.
 *
 * Protocol and behaviour: docs/webcontrol.md
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "nvs.h"

#include "config.h"
#include "commander.h"
#include "log.h"
#include "param.h"
#include "platform.h"
#include "pm_esplane.h"
#include "sensors.h"
#include "sitaw.h"
#include "stabilizer_types.h"
#include "system.h"
#include "wifi_esp32.h"

#include "dns_server.h"
#include "webctrl.h"
#include "webctrl_core.h"

static const char *TAG = "WEBCTRL";

#define WEBCTRL_PERIOD_MS     10
#define WEBCTRL_MAX_CLIENTS   CONFIG_WEBCTRL_MAX_CLIENTS
#define WEBCTRL_WS_PORT       81
#define WEBCTRL_RX_MAX        64

/* Same thrust limits as the CRTP commander (crtp_commander_rpyt.c) */
#define WEB_MAX_THRUST        60000.0f
#define WEB_MIN_THRUST        1000.0f

#define WEBCTRL_NVS_NAMESPACE "webctrl"
#define WEBCTRL_NVS_TRIM      "trim"
#define WEBCTRL_NVS_GAINS     "gains"
#define WEBCTRL_TRIM_SAVE_MS  1000   /* save this long after the last change */

extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[] asm("_binary_index_html_gz_end");

typedef struct {
    int      fd;             /* -1: free slot */
    uint32_t ip;             /* peer IPv4, network byte order */
    bool     helloSeen;      /* telemetry only after HELLO: the first telemetry
                                must already show a resumed session */
    bool     armReleased;    /* ARM flag seen off since the last stop */
    uint8_t  armResult;      /* wcArmResult_t of the last arm attempt */
    uint16_t echoSeq;
    uint32_t echoClientMs;
    uint16_t rxCount;        /* accepted frames in the current second */
    uint16_t rxRate;
    bool     txPending;
    uint8_t  tx[WC_TELEMETRY_LEN];
    bool     fast;           /* wants FAST samples (STREAM) */
    bool     fastPending;    /* the current FAST frame is not sent yet */
} wcClient_t;

static const wcConfig_t cfg = {
    .okMs = CONFIG_WEBCTRL_OK_MS,
    .holdMs = CONFIG_WEBCTRL_HOLD_MS,
    .descendMs = CONFIG_WEBCTRL_DESCEND_MS,
    .offMs = CONFIG_WEBCTRL_OFF_MS,
    .staleMs = CONFIG_WEBCTRL_STALE_MS,
    .descendFactor = CONFIG_WEBCTRL_DESCEND_PERCENT / 100.0f,
    .maxAngleDeg = CONFIG_WEBCTRL_MAX_ANGLE_DEG,
    .maxYawRateDeg = CONFIG_WEBCTRL_MAX_YAW_RATE_DEG,
    .batLandMv = CONFIG_WEBCTRL_BAT_LAND_MV,
    .batArmMinMv = CONFIG_WEBCTRL_BAT_ARM_MIN_MV,
    .batLandDelayMs = CONFIG_WEBCTRL_BAT_LAND_DELAY_MS,
    .batLandTimeMs = CONFIG_WEBCTRL_BAT_LAND_TIME_MS,
};

static SemaphoreHandle_t lock;
static httpd_handle_t portalServer;
static httpd_handle_t wsServer;
static esp_netif_t *apNetif;

static wcClient_t clients[WEBCTRL_MAX_CLIENTS];

/* Pilot session. It lives on when the pilot's connection drops: the failsafe
 * runs, and the same page can come back (HELLO with the token) and fly on. */
static uint32_t pilotToken;          /* 0: no web pilot */
static int pilotIdx = -1;            /* client slot of the pilot, -1 if not connected */
static wcLink_t pilotLink;
static wcHover_t hover;
static wcStage_t pilotStage = WC_STAGE_OFF;
static wcSetpoint_t lastOut;
static wcBattery_t batState;
static uint8_t lastStopReason = WC_STOP_NONE;   /* wcStopReason_t, shown by the pages */

/* Trim of the web pilot. It is stored on the drone (NVS): it belongs to the
 * drone, and captive portal windows may forget their own storage. */
static wcTrim_t trim;
static bool trimDirty;               /* changed, not saved yet */
static uint32_t trimChangedMs;

static char apIpStr[16];
static char portalUri[40];
static char capportUri[64];

static logVarId_t logRoll, logPitch, logYaw;
static volatile bool telemetryQueued;
static uint32_t telemetrySkipped;    /* socket busy: frame not sent */

/* FAST samples for the ground station: collected every 10 ms while a client
 * wants them, sent in frames of WC_FAST_SAMPLES. */
static logVarId_t logRollSp, logPitchSp, logGyro[3], logRateSp[3], logMotor[4], logThrust;
static wcFast_t fastFrame;
static uint16_t fastSeq;
static uint8_t fastTx[WC_FAST_MAX_LEN];
static size_t fastTxLen;
static bool fastWanted;              /* at least one client asked for FAST */

/* Controller gains for the ground station. Defaults are the values the
 * firmware starts with; gains stored in NVS replace them at boot. */
static paramVarId_t gainIds[WC_GAIN_COUNT];
static float gainDefaults[WC_GAIN_COUNT];
static bool gainsReady;              /* ids resolved, defaults read, stored gains applied */
static bool gainsStored;             /* gains from NVS are in use */

/* Motor test: the motors get fixed ratios (parameter group motorPowerSet,
 * used by the power distribution while armed) for a limited time. */
static paramVarId_t motorSetEnable, motorSetRatio[4];
static bool motorTestAvailable;
static struct {
    bool     active;
    int      ownerFd;                /* connection that started it */
    uint8_t  mask;
    uint8_t  percent;
    uint32_t startMs;
    uint32_t durationMs;
} motorTest;

static uint32_t nowMs(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static float logFloatOrZero(logVarId_t id)
{
    return LOG_VARID_IS_VALID(id) ? logGetFloat(id) : 0.0f;
}

/* ------------------------------------------------------------------------ */
/* Clients (all called with the lock held)                                   */
/* ------------------------------------------------------------------------ */

static int findClient(int fd)
{
    for (int i = 0; i < WEBCTRL_MAX_CLIENTS; i++) {
        if (clients[i].fd == fd) {
            return i;
        }
    }
    return -1;
}

static int addClient(int fd, uint32_t ip)
{
    int idx = findClient(fd);
    for (int i = 0; idx < 0 && i < WEBCTRL_MAX_CLIENTS; i++) {
        if (clients[i].fd < 0) {
            idx = i;
        }
    }
    if (idx >= 0) {
        memset(&clients[idx], 0, sizeof(clients[idx]));
        clients[idx].fd = fd;
        clients[idx].ip = ip;
        // A new connection has to send ARM off once before it may arm. A page
        // that still believes it is armed (e.g. after a failsafe) cannot re-arm
        // by just reconnecting. Resuming with the session token is different.
        clients[idx].armReleased = false;
    }
    return idx;
}

/* ------------------------------------------------------------------------ */
/* Pilot session (lock held)                                                 */
/* ------------------------------------------------------------------------ */

static void sendSetpoint(const wcSetpoint_t *sp)
{
    setpoint_t s;
    memset(&s, 0, sizeof(s));

    s.mode.x = modeDisable;
    s.mode.y = modeDisable;
    s.mode.z = modeDisable;
    s.mode.roll = modeAbs;
    s.mode.pitch = modeAbs;
    s.mode.yaw = modeVelocity;

    // Crazyflie legacy conventions, as used by the app and the ESP-NOW joystick:
    // positive roll = right, negative pitch = forward, negative yaw rate = right.
    s.attitude.roll = sp->roll;
    s.attitude.pitch = -sp->pitch;
    s.attitudeRate.yaw = -sp->yawRate;

    const float thrust = sp->thrust * WEB_MAX_THRUST;
    s.thrust = (thrust < WEB_MIN_THRUST) ? 0 : thrust;

    commanderSetSetpoint(&s, COMMANDER_PRIORITY_WEB);
}

static void startPilotSession(int idx, const wcControl_t *cmd, uint32_t t)
{
    pilotToken = esp_random() | 1u;
    pilotIdx = idx;
    wcLinkReset(&pilotLink);
    pilotLink.connected = true;
    wcLinkOnControl(&pilotLink, &cfg, t, cmd);
    wcHoverReset(&hover);
    wcBatteryNewSession(&batState);
    pilotStage = WC_STAGE_OK;
    ESP_LOGI(TAG, "armed by browser (fd %d)", clients[idx].fd);
}

/* failsafe: the pilot has to release ARM before arming again */
static void endPilotSession(bool failsafe, wcStopReason_t reason, const char *why)
{
    systemReleaseArm(ARM_SOURCE_WEB);
    if (pilotIdx >= 0 && failsafe) {
        clients[pilotIdx].armReleased = false;
    }
    pilotToken = 0;
    pilotIdx = -1;
    pilotStage = WC_STAGE_OFF;
    lastStopReason = (uint8_t)reason;
    memset(&lastOut, 0, sizeof(lastOut));

    // Leave a zero setpoint in the commander so nobody can pick up the last
    // web thrust within the commander timeout.
    const wcSetpoint_t zero = {0};
    sendSetpoint(&zero);
    ESP_LOGW(TAG, "disarmed: %s", why);
}

static bool systemReady(void)
{
    return systemIsStarted() && sensorsAreCalibrated();
}

/* ------------------------------------------------------------------------ */
/* Motor test (lock held)                                                    */
/* ------------------------------------------------------------------------ */

static void motorTestSetRatios(uint8_t mask, uint8_t percent)
{
    const uint16_t ratio = wcMotorRatio(percent);
    for (int m = 0; m < 4; m++) {
        paramSetInt(motorSetRatio[m], (mask & (1u << m)) ? ratio : 0);
    }
}

static void motorTestStop(const char *why)
{
    if (!motorTest.active) {
        return;
    }
    // Disarm first: the power distribution turns the motors off in the next
    // cycle, whatever the ratios are.
    systemReleaseArm(ARM_SOURCE_WEB);
    paramSetInt(motorSetEnable, 0);
    motorTestSetRatios(0, 0);
    motorTest.active = false;
    ESP_LOGI(TAG, "motor test stopped: %s", why);
}

static wcMotorReply_t handleMotorTest(int idx, const wcMotorTest_t *req, uint32_t t)
{
    wcMotorReply_t rep = {0};
    if (req->mask == 0) {
        motorTestStop("stop request");   // every client may stop it
        return rep;
    }

    const armSource_t owner = systemGetArmOwner();
    const wcMotorTestInputs_t in = {
        .enabled = motorTestAvailable,
        .systemReady = systemReady(),
        .busy = pilotToken != 0 ||
                (owner != ARM_SOURCE_NONE && !(owner == ARM_SOURCE_WEB && motorTest.active)),
    };
    wcMotorTest_t applied;
    rep.status = (uint8_t)wcMotorTestCheck(&in, req, CONFIG_WEBCTRL_MOTOR_TEST_MAX_PERCENT, &applied);
    if (rep.status != WC_MOTOR_OK) {
        return rep;
    }
    // Arm before setting the ratios: the arm is only granted if nobody else
    // holds it, and with idle thrust 0 the motors stay still until then.
    if (!motorTest.active && !systemRequestArm(ARM_SOURCE_WEB)) {
        rep.status = WC_MOTOR_BUSY;
        return rep;
    }
    motorTestSetRatios(applied.mask, applied.percent);
    paramSetInt(motorSetEnable, 1);
    motorTest.active = true;
    motorTest.ownerFd = clients[idx].fd;
    motorTest.mask = applied.mask;
    motorTest.percent = applied.percent;
    motorTest.startMs = t;
    motorTest.durationMs = applied.durationMs;
    ESP_LOGW(TAG, "motor test: motors 0x%x at %u %% for %u ms", applied.mask, applied.percent,
             applied.durationMs);

    rep.mask = applied.mask;
    rep.percent = applied.percent;
    rep.durationMs = applied.durationMs;
    return rep;
}

static void emergencyStop(int killerIdx)
{
    // Any connected browser may stop the drone, whoever flies it.
    motorTestStop("KILL");
    systemForceDisarm();
    if (pilotToken != 0) {
        endPilotSession(true, WC_STOP_KILL, "KILL");
    } else {
        lastStopReason = WC_STOP_KILL;
        const wcSetpoint_t zero = {0};
        sendSetpoint(&zero);
        ESP_LOGW(TAG, "KILL from browser (fd %d)", clients[killerIdx].fd);
    }
    clients[killerIdx].armReleased = false;
}

static void handleControl(int idx, const wcControl_t *cmd, uint32_t t)
{
    wcClient_t *c = &clients[idx];

    if (cmd->flags & WC_FLAG_KILL) {
        c->echoSeq = cmd->seq;
        c->echoClientMs = cmd->clientMs;
        emergencyStop(idx);
        return;
    }

    if (idx == pilotIdx) {
        if (wcLinkOnControl(&pilotLink, &cfg, t, cmd) != WC_FRAME_ACCEPTED) {
            return;   // stale or old: not applied, not echoed
        }
        c->rxCount++;
        c->echoSeq = cmd->seq;
        c->echoClientMs = cmd->clientMs;
        if (!(cmd->flags & WC_FLAG_ARM)) {
            c->armReleased = true;
            endPilotSession(false, WC_STOP_RELEASED, "pilot released ARM");
        }
        return;
    }

    c->rxCount++;
    c->echoSeq = cmd->seq;
    c->echoClientMs = cmd->clientMs;

    if (!(cmd->flags & WC_FLAG_ARM)) {
        c->armReleased = true;
        c->armResult = WC_ARM_OK;
        return;
    }
    if (pilotToken != 0) {
        c->armResult = WC_ARM_OTHER_PILOT;   // another browser flies
        return;
    }
    if (motorTest.active) {
        c->armResult = WC_ARM_MOTOR_TEST;
        return;
    }

    const armSource_t owner = systemGetArmOwner();
    const wcArmInputs_t in = {
        .systemReady = systemReady(),
        .lowBattery = pmIsBatteryLow() || wcBatteryArmBlocked(&batState, &cfg),
        .tumbled = sitAwTuDetected(),
        .otherPilot = owner != ARM_SOURCE_NONE && owner != ARM_SOURCE_WEB,
        .releasedSinceStop = c->armReleased,
    };
    wcArmResult_t res = wcArmCheck(&in, cmd);
    if (res == WC_ARM_OK && !systemRequestArm(ARM_SOURCE_WEB)) {
        res = WC_ARM_OTHER_PILOT;
    }
    c->armResult = (uint8_t)res;
    if (res == WC_ARM_OK) {
        startPilotSession(idx, cmd, t);
    }
}

/* ------------------------------------------------------------------------ */
/* Parameters and controller gains                                           */
/* ------------------------------------------------------------------------ */

static void sendBinary(int fd, uint8_t *buf, size_t len)
{
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_BINARY,
        .payload = buf,
        .len = len,
        .final = true,
    };
    if (len > 0) {
        httpd_ws_send_frame_async(wsServer, fd, &frame);
    }
}

static paramVarId_t paramByName(const char *full)
{
    const paramVarId_t invalid = {.id = 0xffffu, .ptr = 0xffffu};
    char group[WC_PARAM_NAME_MAX + 1];
    const char *dot = strchr(full, '.');
    const size_t n = dot ? (size_t)(dot - full) : 0;
    if (n == 0 || n >= sizeof(group)) {
        return invalid;
    }
    memcpy(group, full, n);
    group[n] = '\0';
    char name[WC_PARAM_NAME_MAX + 1];
    strlcpy(name, dot + 1, sizeof(name));
    return paramGetVarId(group, name);
}

/* Changing gains: on the ground everyone, in flight only the web pilot. */
static bool mayChangeGains(int idx)
{
    if (pilotToken != 0) {
        return idx == pilotIdx;
    }
    return systemGetArmOwner() == ARM_SOURCE_NONE && !motorTest.active;
}

/* Saving and resetting only while nobody can fly: writing the flash stalls
 * the flight task for a few milliseconds. */
static bool mayStoreGains(void)
{
    return pilotToken == 0 && systemGetArmOwner() == ARM_SOURCE_NONE && !motorTest.active;
}

/* Once the controller is set up: read the defaults, apply stored gains. */
static void gainsInit(void)
{
    paramVarId_t ids[WC_GAIN_COUNT];
    float defaults[WC_GAIN_COUNT];
    for (int i = 0; i < WC_GAIN_COUNT; i++) {
        ids[i] = paramByName(wcGainNames[i]);
        if (!PARAM_VARID_IS_VALID(ids[i])) {
            ESP_LOGE(TAG, "gain %s not found: gains read only", wcGainNames[i]);
            return;
        }
        defaults[i] = paramGetFloat(ids[i]);
    }

    float stored[WC_GAIN_COUNT];
    bool valid[WC_GAIN_COUNT] = {false};
    bool haveStored = false;
    nvs_handle_t h;
    if (nvs_open(WEBCTRL_NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        uint8_t blob[WC_GAINS_BLOB_LEN];
        size_t len = sizeof(blob);
        haveStored = nvs_get_blob(h, WEBCTRL_NVS_GAINS, blob, &len) == ESP_OK &&
                     wcDecodeGainsBlob(blob, len, stored, valid);
        nvs_close(h);
    }

    int applied = 0;
    xSemaphoreTake(lock, portMAX_DELAY);
    memcpy(gainIds, ids, sizeof(gainIds));
    memcpy(gainDefaults, defaults, sizeof(gainDefaults));
    for (int i = 0; haveStored && i < WC_GAIN_COUNT; i++) {
        if (valid[i]) {
            paramSetFloat(gainIds[i], stored[i]);
            applied++;
        }
    }
    gainsStored = applied > 0;
    gainsReady = true;
    xSemaphoreGive(lock);
    if (applied > 0) {
        ESP_LOGI(TAG, "%d controller gains loaded from flash", applied);
    }
}

static esp_err_t gainsWriteFlash(const float *values, bool erase)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(WEBCTRL_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    if (erase) {
        err = nvs_erase_key(h, WEBCTRL_NVS_GAINS);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        }
    } else {
        uint8_t blob[WC_GAINS_BLOB_LEN];
        wcEncodeGainsBlob(values, blob, sizeof(blob));
        err = nvs_set_blob(h, WEBCTRL_NVS_GAINS, blob, sizeof(blob));
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/* SAVE stores the current gains (or erases the stored ones if all are the
 * defaults); RESET restores the defaults and erases the stored ones. */
static wcParamStatus_t gainsStore(bool reset)
{
    float values[WC_GAIN_COUNT];
    bool allDefault = true;

    xSemaphoreTake(lock, portMAX_DELAY);
    if (!gainsReady || !mayStoreGains()) {
        xSemaphoreGive(lock);
        return WC_PARAM_BUSY;
    }
    for (int i = 0; i < WC_GAIN_COUNT; i++) {
        if (reset) {
            paramSetFloat(gainIds[i], gainDefaults[i]);
        }
        values[i] = paramGetFloat(gainIds[i]);
        allDefault = allDefault && values[i] == gainDefaults[i];
    }
    xSemaphoreGive(lock);

    const esp_err_t err = gainsWriteFlash(values, allDefault);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "storing the gains failed: %s", esp_err_to_name(err));
        return WC_PARAM_FAILED;
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    gainsStored = !allDefault;
    xSemaphoreGive(lock);
    if (reset) {
        ESP_LOGI(TAG, "gains reset to the defaults");
    } else {
        ESP_LOGI(TAG, "gains stored (%s)", allDefault ? "all defaults" : "custom");
    }
    return WC_PARAM_OK;
}

/* Runs in the WebSocket server task; answers on the same connection. */
static void handleParam(int fd, int idx, const wcParamReq_t *req)
{
    wcParamReply_t rep = {.op = req->op, .reqId = req->reqId, .status = WC_PARAM_OK, .dflt = NAN};

    if (req->op == WC_PARAM_SAVE || req->op == WC_PARAM_RESET) {
        rep.status = (uint8_t)gainsStore(req->op == WC_PARAM_RESET);
    } else {
        strlcpy(rep.name, req->name, sizeof(rep.name));
        const paramVarId_t id = paramByName(req->name);
        const int gain = wcGainIndex(req->name);
        xSemaphoreTake(lock, portMAX_DELAY);
        if (!PARAM_VARID_IS_VALID(id)) {
            rep.status = WC_PARAM_UNKNOWN;
        } else {
            if (req->op == WC_PARAM_WRITE) {
                if (gain < 0) {
                    rep.status = WC_PARAM_DENIED;
                } else if (!gainsReady || !mayChangeGains(idx)) {
                    rep.status = WC_PARAM_BUSY;
                } else if (!wcGainValueValid(req->value)) {
                    rep.status = WC_PARAM_INVALID;
                } else {
                    paramSetFloat(id, req->value);
                    ESP_LOGI(TAG, "%s = %.4g", req->name, (double)req->value);
                }
            }
            rep.value = paramGetFloat(id);
            if (gain >= 0 && gainsReady) {
                rep.dflt = gainDefaults[gain];
                rep.flags |= WC_PARAMF_WRITABLE;
            }
        }
        xSemaphoreGive(lock);
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    if (gainsStored) {
        rep.flags |= WC_PARAMF_STORED;
    }
    xSemaphoreGive(lock);

    uint8_t buf[WC_PARAM_MAX_LEN];
    sendBinary(fd, buf, wcEncodeParamReply(&rep, buf, sizeof(buf)));
}

/* ------------------------------------------------------------------------ */
/* WebSocket server (port 81)                                                */
/* ------------------------------------------------------------------------ */

static void sendInfo(int fd, bool resumed)
{
    const esp_app_desc_t *app = esp_app_get_description();
    char json[512];
    int n = snprintf(json, sizeof(json),
                     "{\"t\":\"info\",\"proto\":%d,\"fw\":\"%s\",\"device\":\"%s\",\"resumed\":%s,"
                     "\"maxAngle\":%d,\"maxYawRate\":%d,\"telemetryHz\":%d,"
                     "\"failsafe\":{\"ok\":%d,\"hold\":%d,\"descend\":%d,\"off\":%d,\"stale\":%d},"
                     "\"battery\":{\"land\":%d,\"arm\":%d,\"landTime\":%d},\"trimMax\":%d,"
                     "\"fastHz\":%d,\"motorTest\":{\"maxPercent\":%d,\"maxMs\":%d}}",
                     WC_PROTOCOL_VERSION, app->version, platformConfigGetDeviceTypeName(),
                     resumed ? "true" : "false",
                     CONFIG_WEBCTRL_MAX_ANGLE_DEG, CONFIG_WEBCTRL_MAX_YAW_RATE_DEG,
                     CONFIG_WEBCTRL_TELEMETRY_HZ,
                     CONFIG_WEBCTRL_OK_MS, CONFIG_WEBCTRL_HOLD_MS, CONFIG_WEBCTRL_DESCEND_MS,
                     CONFIG_WEBCTRL_OFF_MS, CONFIG_WEBCTRL_STALE_MS,
                     CONFIG_WEBCTRL_BAT_LAND_MV, CONFIG_WEBCTRL_BAT_ARM_MIN_MV,
                     CONFIG_WEBCTRL_BAT_LAND_TIME_MS, WC_TRIM_MAX,
                     1000 / WC_FAST_PERIOD_MS,
                     motorTestAvailable ? CONFIG_WEBCTRL_MOTOR_TEST_MAX_PERCENT : 0, WC_MOTOR_TEST_MAX_MS);
    if (n <= 0 || n >= (int)sizeof(json)) {
        return;
    }
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)json,
        .len = (size_t)n,
        .final = true,
    };
    httpd_ws_send_frame_async(wsServer, fd, &frame);
}

static void updateFastWanted(void)
{
    fastWanted = false;
    for (int i = 0; i < WEBCTRL_MAX_CLIENTS; i++) {
        fastWanted = fastWanted || (clients[i].fd >= 0 && clients[i].fast);
    }
}

static void handleBinary(int fd, const uint8_t *buf, size_t len, uint32_t t)
{
    bool sendInfoFrame = false;
    bool resumed = false;
    bool paramRequest = false;
    wcParamReq_t param;
    bool motorReplyFrame = false;
    wcMotorReply_t motorReply;

    xSemaphoreTake(lock, portMAX_DELAY);
    int idx = findClient(fd);
    if (idx >= 0 && len > 0) {
        if (buf[0] == WC_MSG_CONTROL) {
            wcControl_t cmd;
            if (wcDecodeControl(buf, len, &cmd)) {
                handleControl(idx, &cmd, t);
            }
        } else if (buf[0] == WC_MSG_HELLO) {
            wcHello_t hello;
            if (wcDecodeHello(buf, len, &hello)) {
                if (pilotToken != 0 && hello.sessionToken == pilotToken) {
                    // The pilot is back on a new connection; the old one (if any
                    // is still open) only watches from now on.
                    pilotIdx = idx;
                    pilotLink.connected = true;
                    wcLinkResume(&pilotLink);
                    clients[idx].armReleased = true;
                    resumed = true;
                    ESP_LOGI(TAG, "pilot resumed (fd %d)", fd);
                }
                clients[idx].helloSeen = true;
                clients[idx].txPending = false;   // drop any snapshot from before HELLO
                sendInfoFrame = true;
            }
        } else if (buf[0] == WC_MSG_TRIM) {
            wcTrim_t tr;
            // On the ground every page may trim, in flight only the pilot.
            if (wcDecodeTrim(buf, len, &tr) && (pilotToken == 0 || idx == pilotIdx) &&
                (tr.roll != trim.roll || tr.pitch != trim.pitch)) {
                trim = tr;
                trimDirty = true;
                trimChangedMs = t;
                ESP_LOGI(TAG, "trim roll %.1f, pitch %.1f deg", tr.roll / 10.0, tr.pitch / 10.0);
            }
        } else if (buf[0] == WC_MSG_STREAM) {
            uint8_t flags;
            if (wcDecodeStream(buf, len, &flags)) {
                clients[idx].fast = (flags & WC_STREAM_FAST) != 0;
                clients[idx].fastPending = false;
                updateFastWanted();
            }
        } else if (buf[0] == WC_MSG_PARAM) {
            paramRequest = wcDecodeParamReq(buf, len, &param);
        } else if (buf[0] == WC_MSG_MOTOR_TEST) {
            wcMotorTest_t req;
            if (wcDecodeMotorTest(buf, len, &req)) {
                motorReply = handleMotorTest(idx, &req, t);
                motorReplyFrame = true;
            }
        }
    }
    xSemaphoreGive(lock);

    if (sendInfoFrame) {
        sendInfo(fd, resumed);
    }
    if (paramRequest) {
        handleParam(fd, idx, &param);
    }
    if (motorReplyFrame) {
        uint8_t out[WC_MOTOR_REPLY_LEN];
        sendBinary(fd, out, wcEncodeMotorReply(&motorReply, out, sizeof(out)));
    }
}

static esp_err_t wsHandler(httpd_req_t *req)
{
    const int fd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET) {
        // Handshake done: register the client.
        uint32_t ip = 0;
        struct sockaddr_storage addr;
        socklen_t addrLen = sizeof(addr);
        if (getpeername(fd, (struct sockaddr *)&addr, &addrLen) == 0 && addr.ss_family == AF_INET) {
            ip = ((struct sockaddr_in *)&addr)->sin_addr.s_addr;
        }
        // Telemetry should leave at once, not wait for more data (Nagle).
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        xSemaphoreTake(lock, portMAX_DELAY);
        int idx = addClient(fd, ip);
        xSemaphoreGive(lock);
        if (idx < 0) {
            ESP_LOGW(TAG, "too many browsers, connection refused");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "browser connected (fd %d, slot %d)", fd, idx);
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    uint8_t buf[WEBCTRL_RX_MAX];
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        return err;
    }
    if (frame.len > sizeof(buf)) {
        ESP_LOGW(TAG, "frame of %u bytes, closing (fd %d)", (unsigned)frame.len, fd);
        return ESP_FAIL;
    }
    frame.payload = buf;
    err = httpd_ws_recv_frame(req, &frame, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }
    if (frame.type == HTTPD_WS_TYPE_BINARY) {
        handleBinary(fd, buf, frame.len, nowMs());
    }
    return ESP_OK;
}

static void wsClose(httpd_handle_t hd, int fd)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    int idx = findClient(fd);
    if (idx >= 0) {
        if (idx == pilotIdx) {
            pilotIdx = -1;
            pilotLink.connected = false;
            ESP_LOGW(TAG, "pilot connection closed (fd %d)", fd);
        }
        if (motorTest.active && motorTest.ownerFd == fd) {
            motorTestStop("ground station left");
        }
        clients[idx].fd = -1;
        clients[idx].fast = false;
        updateFastWanted();
    }
    xSemaphoreGive(lock);
    close(fd);
}

static bool socketWritable(int fd)
{
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(fd, &wfds);
    struct timeval tv = {0, 0};
    return select(fd + 1, NULL, &wfds, NULL, &tv) > 0;
}

/* Runs in the WebSocket server task. A socket whose send buffer is full (a
 * client that does not read) is skipped instead of blocking the server. */
static void telemetryWork(void *arg)
{
    for (int i = 0; i < WEBCTRL_MAX_CLIENTS; i++) {
        uint8_t buf[WC_TELEMETRY_LEN];
        uint8_t fast[WC_FAST_MAX_LEN];
        size_t fastLen = 0;
        int fd;

        xSemaphoreTake(lock, portMAX_DELAY);
        fd = clients[i].fd;
        const bool pending = fd >= 0 && clients[i].txPending;
        if (pending) {
            memcpy(buf, clients[i].tx, sizeof(buf));
            clients[i].txPending = false;
        }
        if (fd >= 0 && clients[i].fastPending) {
            fastLen = fastTxLen;
            memcpy(fast, fastTx, fastLen);
            clients[i].fastPending = false;
        }
        xSemaphoreGive(lock);

        if (pending) {
            if (socketWritable(fd)) {
                sendBinary(fd, buf, sizeof(buf));
            } else {
                telemetrySkipped++;
            }
        }
        // FAST frames are a nice-to-have: they never wait for a busy socket.
        if (fastLen > 0 && socketWritable(fd)) {
            sendBinary(fd, fast, fastLen);
        }
    }
    telemetryQueued = false;
}

/* ------------------------------------------------------------------------ */
/* Captive portal (port 80)                                                  */
/* ------------------------------------------------------------------------ */

static esp_err_t redirectToPortal(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", portalUri);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t rootHandler(httpd_req_t *req)
{
    // Connectivity checks ask for other host names; send them to the drone's
    // address so the page always runs on the same origin.
    char host[48];
    if (httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK ||
        strcmp(host, apIpStr) != 0) {
        return redirectToPortal(req);
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)index_html_gz_start,
                           index_html_gz_end - index_html_gz_start);
}

/* RFC 8908 captive portal API, announced by DHCP option 114 (RFC 8910) */
static esp_err_t capportHandler(httpd_req_t *req)
{
    char json[96];
    snprintf(json, sizeof(json), "{\"captive\":true,\"user-portal-url\":\"%s\"}", portalUri);
    httpd_resp_set_type(req, "application/captive+json");
    httpd_resp_set_hdr(req, "Cache-Control", "private");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t noContentHandler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t notFoundHandler(httpd_req_t *req, httpd_err_code_t err)
{
    return redirectToPortal(req);
}

static bool startServers(void)
{
    httpd_config_t pc = HTTPD_DEFAULT_CONFIG();
    pc.server_port = 80;
    pc.ctrl_port = 32768;
    pc.max_open_sockets = 5;
    pc.lru_purge_enable = true;      // phones open many short connections
    pc.max_uri_handlers = 4;
    pc.core_id = NETWORK_TASK_CORE;
    pc.task_priority = WEBCTRL_HTTPD_TASK_PRI;
    pc.stack_size = WEBCTRL_HTTPD_STACKSIZE;
    pc.recv_wait_timeout = 2;
    pc.send_wait_timeout = 2;

    if (httpd_start(&portalServer, &pc) != ESP_OK) {
        ESP_LOGE(TAG, "portal server failed to start");
        return false;
    }
    const httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = rootHandler};
    const httpd_uri_t capport = {.uri = "/captive-portal/api", .method = HTTP_GET, .handler = capportHandler};
    const httpd_uri_t favicon = {.uri = "/favicon.ico", .method = HTTP_GET, .handler = noContentHandler};
    httpd_register_uri_handler(portalServer, &root);
    httpd_register_uri_handler(portalServer, &capport);
    httpd_register_uri_handler(portalServer, &favicon);
    httpd_register_err_handler(portalServer, HTTPD_404_NOT_FOUND, notFoundHandler);

    httpd_config_t wc = HTTPD_DEFAULT_CONFIG();
    wc.server_port = WEBCTRL_WS_PORT;
    wc.ctrl_port = 32769;
    wc.max_open_sockets = WEBCTRL_MAX_CLIENTS + 1;
    wc.lru_purge_enable = false;     // never drop the pilot for a newcomer
    wc.max_uri_handlers = 2;
    wc.core_id = NETWORK_TASK_CORE;
    wc.task_priority = WEBCTRL_HTTPD_TASK_PRI;
    wc.stack_size = WEBCTRL_HTTPD_STACKSIZE;
    wc.recv_wait_timeout = 2;
    wc.send_wait_timeout = 2;
    wc.close_fn = wsClose;
    // Dead peers (phone switched off) are found by TCP keep-alive after
    // about 5 + 3 * 2 s; the failsafe does not wait for this.
    wc.keep_alive_enable = true;
    wc.keep_alive_idle = 5;
    wc.keep_alive_interval = 2;
    wc.keep_alive_count = 3;

    if (httpd_start(&wsServer, &wc) != ESP_OK) {
        ESP_LOGE(TAG, "control server failed to start");
        return false;
    }
    const httpd_uri_t ws = {.uri = "/ws", .method = HTTP_GET, .handler = wsHandler, .is_websocket = true};
    httpd_register_uri_handler(wsServer, &ws);
    return true;
}

/* ------------------------------------------------------------------------ */
/* Wi-Fi events: a pilot phone leaving the access point                     */
/* ------------------------------------------------------------------------ */

static void onWifiEvent(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id != WIFI_EVENT_AP_STADISCONNECTED || apNetif == NULL) {
        return;
    }
    const wifi_event_ap_stadisconnected_t *ev = data;
    esp_netif_pair_mac_ip_t pair = {0};
    memcpy(pair.mac, ev->mac, sizeof(pair.mac));
    if (esp_netif_dhcps_get_clients_by_mac(apNetif, 1, &pair) != ESP_OK || pair.ip.addr == 0) {
        return;
    }

    xSemaphoreTake(lock, portMAX_DELAY);
    if (pilotIdx >= 0 && clients[pilotIdx].ip == pair.ip.addr) {
        // The socket is dead but would stay open until keep-alive gives up.
        pilotIdx = -1;
        pilotLink.connected = false;
        ESP_LOGW(TAG, "pilot phone left the Wi-Fi");
    }
    xSemaphoreGive(lock);
}

/* ------------------------------------------------------------------------ */
/* Supervisor task                                                           */
/* ------------------------------------------------------------------------ */

/* lock held */
static void buildTelemetry(uint32_t now)
{
    const armSource_t owner = systemGetArmOwner();
    const bool session = pilotToken != 0;
    const bool ready = systemReady();
    const bool lowBat = pmIsBatteryLow();
    const bool tumbled = sitAwTuDetected();
    const float vbat = batState.volts;   // filtered
    // "Too low to arm" only on the ground: under load the voltage sags anyway.
    const uint8_t state2 = (uint8_t)((session && batState.landing ? WC_STATE2_BAT_LANDING : 0) |
                                     (!session && wcBatteryArmBlocked(&batState, &cfg) ? WC_STATE2_BAT_NO_ARM : 0) |
                                     (motorTest.active ? WC_STATE2_MOTOR_TEST : 0));
    const int16_t roll = wcCentiDeg(logFloatOrZero(logRoll));
    const int16_t pitch = wcCentiDeg(logFloatOrZero(logPitch));
    const int16_t yaw = wcCentiDeg(logFloatOrZero(logYaw));

    for (int i = 0; i < WEBCTRL_MAX_CLIENTS; i++) {
        wcClient_t *c = &clients[i];
        if (c->fd < 0 || !c->helloSeen) {
            continue;
        }

        wcTelemetry_t t = {0};
        if (session && owner == ARM_SOURCE_WEB) {
            t.state |= WC_STATE_ARMED;
        }
        if (i == pilotIdx) {
            t.state |= WC_STATE_YOU_PILOT;
        }
        if ((owner != ARM_SOURCE_NONE && owner != ARM_SOURCE_WEB) || (session && i != pilotIdx)) {
            t.state |= WC_STATE_OTHER_PILOT;
        }
        if (session && pilotStage >= WC_STAGE_LEVEL) {
            t.state |= WC_STATE_FAILSAFE;
        }
        if (lowBat) {
            t.state |= WC_STATE_LOW_BATTERY;
        }
        if (!c->armReleased) {
            t.state |= WC_STATE_ARM_BLOCKED;
        }
        if (ready) {
            t.state |= WC_STATE_READY;
        }
        if (tumbled) {
            t.state |= WC_STATE_TUMBLED;
        }

        t.stage = (uint8_t)(session ? pilotStage : WC_STAGE_OFF);
        t.armResult = c->armResult;
        t.echoSeq = c->echoSeq;
        t.echoClientMs = c->echoClientMs;
        t.rxRate = c->rxRate;
        t.vbatMv = (uint16_t)(vbat > 0 ? vbat * 1000.0f : 0);
        t.rollCdeg = roll;
        t.pitchCdeg = pitch;
        t.yawCdeg = yaw;
        t.thrustOut = (uint16_t)(session ? lastOut.thrust * WC_THRUST_MAX : 0);
        t.state2 = state2;
        t.stopReason = lastStopReason;
        t.trim = trim;
        t.uptimeMs = now;
        if (i == pilotIdx) {
            t.staleDrops = pilotLink.staleDrops;
            t.delayMs = (uint16_t)(pilotLink.lastDelayMs > 0xFFFF ? 0xFFFF : pilotLink.lastDelayMs);
            t.sessionToken = pilotToken;
        }

        wcEncodeTelemetry(&t, c->tx, sizeof(c->tx));
        c->txPending = true;
    }
}

static void supervise(uint32_t now, float dt, uint32_t *rateWindowStart)
{
    wcSetpoint_t out = {0};
    const float vbat = pmGetBatteryVoltage();

    // Everything under the lock, including sending the setpoint: a KILL that
    // is handled in between must not be followed by a stale non-zero setpoint.
    xSemaphoreTake(lock, portMAX_DELAY);

    const bool session = pilotToken != 0;
    const bool flying = session && lastOut.thrust >= WC_HOVER_FLYING_THRUST;
    const float landFrom = wcHoverValid(&hover) ? fminf(hover.value, lastOut.thrust) : lastOut.thrust;
    const wcBatEvent_t batEvent = wcBatteryUpdate(&batState, &cfg, now, dt, vbat, session, flying, landFrom);

    if (session) {
        if (systemGetArmOwner() != ARM_SOURCE_WEB) {
            // Arm taken away elsewhere (system.forceArm cleared, emergency stop)
            endPilotSession(true, WC_STOP_ARM_LOST, "arm lost");
        } else if (sitAwTuDetected()) {
            endPilotSession(true, WC_STOP_TUMBLED, "tumbled");
        } else if (batEvent == WC_BAT_EMPTY) {
            endPilotSession(true, WC_STOP_BATTERY, "battery empty");
        } else {
            if (batEvent == WC_BAT_LAND) {
                ESP_LOGW(TAG, "battery %.2f V: forced landing from thrust %.2f",
                         (double)batState.volts, (double)batState.landFrom);
            }
            const wcStage_t stage = wcStageForAge(&cfg, wcLinkAge(&pilotLink, &cfg, now));
            wcSetpoint_t last;
            wcControlToSetpoint(&cfg, &pilotLink.last, &trim, &last);
            if (stage == WC_STAGE_OK && !batState.landing) {
                wcHoverUpdate(&hover, last.thrust, dt);
            }
            wcFailsafeOutput(&cfg, stage, &last, &hover, &trim, &out);
            out.thrust = fminf(out.thrust, wcBatteryThrustCap(&batState, &cfg, now));

            if (stage != pilotStage) {
                if (stage >= WC_STAGE_LEVEL) {
                    ESP_LOGW(TAG, "link failsafe: stage %d (hover %.2f%s)", stage,
                             (double)hover.value, wcHoverValid(&hover) ? "" : ", not learned");
                } else if (pilotStage >= WC_STAGE_LEVEL) {
                    ESP_LOGI(TAG, "link back, pilot in control");
                }
                pilotStage = stage;
            }

            if (stage == WC_STAGE_OFF) {
                endPilotSession(true, WC_STOP_LINK, "link lost, failsafe ended");
            } else if (batState.landing && (out.thrust <= 0.0f || last.thrust <= 0.0f)) {
                // The ramp ran out, or the pilot pulled the throttle down after touchdown.
                endPilotSession(true, WC_STOP_BATTERY, "battery empty, landed");
            } else {
                lastOut = out;
                sendSetpoint(&out);
            }
        }
    }

    if (motorTest.active) {
        if (now - motorTest.startMs >= motorTest.durationMs) {
            motorTestStop("time is up");
        } else if (systemGetArmOwner() != ARM_SOURCE_WEB) {
            motorTestStop("arm taken away");
        }
    }

    if (now - *rateWindowStart >= 1000) {
        *rateWindowStart = now;
        for (int i = 0; i < WEBCTRL_MAX_CLIENTS; i++) {
            clients[i].rxRate = clients[i].rxCount;
            clients[i].rxCount = 0;
        }
    }

    xSemaphoreGive(lock);
}

/* Every 10 ms while a client wants FAST samples. True when a frame is complete. */
static bool sampleFast(uint32_t now)
{
    if (!fastWanted) {
        fastFrame.count = 0;
        return false;
    }
    if (fastFrame.count == 0) {
        fastFrame.t0Ms = now;
        fastFrame.seq = fastSeq;
    }
    wcFastSample_t *s = &fastFrame.s[fastFrame.count];
    s->rollCdeg = wcCentiDeg(logFloatOrZero(logRoll));
    s->pitchCdeg = wcCentiDeg(logFloatOrZero(logPitch));
    s->yawCdeg = wcCentiDeg(logFloatOrZero(logYaw));
    s->rollSpCdeg = wcCentiDeg(logFloatOrZero(logRollSp));
    s->pitchSpCdeg = wcCentiDeg(logFloatOrZero(logPitchSp));
    // Rates as the rate controller compares them (controller_pid.c): pitch is -gyro.y.
    s->gyroDdps[0] = wcDeciDeg(logFloatOrZero(logGyro[0]));
    s->gyroDdps[1] = wcDeciDeg(-logFloatOrZero(logGyro[1]));
    s->gyroDdps[2] = wcDeciDeg(logFloatOrZero(logGyro[2]));
    for (int k = 0; k < 3; k++) {
        s->rateSpDdps[k] = wcDeciDeg(logFloatOrZero(logRateSp[k]));
    }
    for (int m = 0; m < 4; m++) {
        s->motor[m] = wcU16Sat(logFloatOrZero(logMotor[m]));
    }
    s->thrust = wcU16Sat(logFloatOrZero(logThrust));
    fastSeq++;
    if (++fastFrame.count < WC_FAST_SAMPLES) {
        return false;
    }

    fastFrame.periodMs = WC_FAST_PERIOD_MS;
    xSemaphoreTake(lock, portMAX_DELAY);
    fastTxLen = wcEncodeFast(&fastFrame, fastTx, sizeof(fastTx));
    for (int i = 0; i < WEBCTRL_MAX_CLIENTS; i++) {
        clients[i].fastPending = clients[i].fd >= 0 && clients[i].fast && clients[i].helloSeen;
    }
    xSemaphoreGive(lock);
    fastFrame.count = 0;
    return true;
}

static void lookUpVariables(void)
{
    logRoll = logGetVarId("stabilizer", "roll");
    logPitch = logGetVarId("stabilizer", "pitch");
    logYaw = logGetVarId("stabilizer", "yaw");
    logThrust = logGetVarId("stabilizer", "thrust");
    logRollSp = logGetVarId("controller", "roll");
    logPitchSp = logGetVarId("controller", "pitch");
    logRateSp[0] = logGetVarId("controller", "rollRate");
    logRateSp[1] = logGetVarId("controller", "pitchRate");
    logRateSp[2] = logGetVarId("controller", "yawRate");
    logGyro[0] = logGetVarId("gyro", "x");
    logGyro[1] = logGetVarId("gyro", "y");
    logGyro[2] = logGetVarId("gyro", "z");
    static const char *const motorNames[4] = {"m1", "m2", "m3", "m4"};
    motorSetEnable = paramByName("motorPowerSet.enable");
    bool motorsOk = PARAM_VARID_IS_VALID(motorSetEnable);
    for (int m = 0; m < 4; m++) {
        logMotor[m] = logGetVarId("motor", (char *)motorNames[m]);
        char name[24];
        snprintf(name, sizeof(name), "motorPowerSet.%s", motorNames[m]);
        motorSetRatio[m] = paramByName(name);
        motorsOk = motorsOk && PARAM_VARID_IS_VALID(motorSetRatio[m]);
    }
    motorTestAvailable = motorsOk && CONFIG_WEBCTRL_MOTOR_TEST_MAX_PERCENT > 0;
}

static void trimLoad(void)
{
    nvs_handle_t h;
    if (nvs_open(WEBCTRL_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;   // nothing stored yet
    }
    int8_t v[2];
    size_t len = sizeof(v);
    if (nvs_get_blob(h, WEBCTRL_NVS_TRIM, v, &len) == ESP_OK && len == sizeof(v) &&
        v[0] >= -WC_TRIM_MAX && v[0] <= WC_TRIM_MAX && v[1] >= -WC_TRIM_MAX && v[1] <= WC_TRIM_MAX) {
        trim.roll = v[0];
        trim.pitch = v[1];
        ESP_LOGI(TAG, "trim roll %.1f, pitch %.1f deg", trim.roll / 10.0, trim.pitch / 10.0);
    }
    nvs_close(h);
}

/*
 * Writing the flash switches the cache off for a few milliseconds, which
 * stalls the flight task. So the trim is only saved while nobody can fly.
 */
static void trimSaveWhenIdle(uint32_t now)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    const bool save = trimDirty && pilotToken == 0 && systemGetArmOwner() == ARM_SOURCE_NONE &&
                      now - trimChangedMs >= WEBCTRL_TRIM_SAVE_MS;
    const int8_t v[2] = {trim.roll, trim.pitch};
    if (save) {
        trimDirty = false;
    }
    xSemaphoreGive(lock);
    if (!save) {
        return;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(WEBCTRL_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, WEBCTRL_NVS_TRIM, v, sizeof(v));
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "saving the trim failed: %s", esp_err_to_name(err));
    }
}

static void setupCaptivePortal(void)
{
    esp_netif_ip_info_t ipInfo;
    esp_netif_get_ip_info(apNetif, &ipInfo);
    snprintf(apIpStr, sizeof(apIpStr), IPSTR, IP2STR(&ipInfo.ip));
    snprintf(portalUri, sizeof(portalUri), "http://%s/", apIpStr);
    snprintf(capportUri, sizeof(capportUri), "http://%s/captive-portal/api", apIpStr);

    // DHCP option 114 (RFC 8910). The DHCP server keeps the pointer, so the
    // string is static. It also hands out the drone as DNS server
    // (CONFIG_LWIP_DHCPS_ADD_DNS).
    esp_netif_dhcps_stop(apNetif);
    esp_netif_dhcps_option(apNetif, ESP_NETIF_OP_SET, ESP_NETIF_CAPTIVEPORTAL_URI,
                           capportUri, strlen(capportUri));
    esp_netif_dhcps_start(apNetif);

#ifdef CONFIG_WEBCTRL_CAPTIVE_DNS
    if (!dnsServerStart(ipInfo.ip.addr)) {
        ESP_LOGE(TAG, "captive DNS failed to start");
    }
#endif
}

static void webctrlTask(void *arg)
{
    // The access point and the commander are set up by the system task.
    while (!wifiTest() || !commanderTest()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    apNetif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (apNetif == NULL) {
        ESP_LOGE(TAG, "no Wi-Fi access point, web control off");
        vTaskDelete(NULL);
        return;
    }

    lookUpVariables();
    trimLoad();

    setupCaptivePortal();
    if (!startServers()) {
        vTaskDelete(NULL);
        return;
    }
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, onWifiEvent, NULL);
    ESP_LOGI(TAG, "web control ready: %s (control port %d)", portalUri, WEBCTRL_WS_PORT);

    const uint32_t telemetryPeriodMs = 1000 / CONFIG_WEBCTRL_TELEMETRY_HZ;
    uint32_t lastTelemetry = 0;
    uint32_t rateWindowStart = nowMs();
    uint32_t last = nowMs();
    bool gainsTried = false;
    TickType_t wake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(WEBCTRL_PERIOD_MS));
        const uint32_t now = nowMs();
        const float dt = (now - last) / 1000.0f;
        last = now;

        supervise(now, dt, &rateWindowStart);
        trimSaveWhenIdle(now);
        if (!gainsTried && systemIsStarted()) {
            gainsTried = true;
            gainsInit();   // the controller is set up by now
        }

        bool queue = sampleFast(now);
        if (now - lastTelemetry >= telemetryPeriodMs && !telemetryQueued) {
            lastTelemetry = now;
            xSemaphoreTake(lock, portMAX_DELAY);
            buildTelemetry(now);
            xSemaphoreGive(lock);
            queue = true;
        }
        if (queue && !telemetryQueued) {
            telemetryQueued = true;
            if (httpd_queue_work(wsServer, telemetryWork, NULL) != ESP_OK) {
                telemetryQueued = false;
            }
        }
    }
}

void webctrlLaunch(void)
{
    static bool launched = false;
    if (launched) {
        return;
    }
    launched = true;

    for (int i = 0; i < WEBCTRL_MAX_CLIENTS; i++) {
        clients[i].fd = -1;
    }
    lock = xSemaphoreCreateMutex();
    wcLinkReset(&pilotLink);
    wcHoverReset(&hover);
    wcBatteryReset(&batState);

    xTaskCreatePinnedToCore(webctrlTask, WEBCTRL_TASK_NAME, WEBCTRL_TASK_STACKSIZE, NULL,
                            WEBCTRL_TASK_PRI, NULL, NETWORK_TASK_CORE);
}

LOG_GROUP_START(webctrl)
LOG_ADD(LOG_UINT8, stage, &pilotStage)
LOG_ADD(LOG_UINT16, staleDrops, &pilotLink.staleDrops)
LOG_ADD(LOG_UINT32, delayMs, &pilotLink.lastDelayMs)
LOG_ADD(LOG_FLOAT, hover, &hover.value)
LOG_ADD(LOG_UINT32, txSkipped, &telemetrySkipped)
LOG_ADD(LOG_FLOAT, vbat, &batState.volts)
LOG_ADD(LOG_UINT8, batLanding, &batState.landing)
LOG_GROUP_STOP(webctrl)
