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
 * webctrl_core.c - platform independent part of the browser control.
 */

#include <math.h>
#include <string.h>

#include "webctrl_core.h"

/* ------------------------------------------------------------------------ */
/* Little endian helpers                                                     */
/* ------------------------------------------------------------------------ */

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void wrf(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof(u));
    wr32(p, u);
}

static float rdf(const uint8_t *p)
{
    const uint32_t u = rd32(p);
    float v;
    memcpy(&v, &u, sizeof(v));
    return v;
}

static bool axisInRange(int16_t v)
{
    return v >= -WC_AXIS_MAX && v <= WC_AXIS_MAX;
}

/* ------------------------------------------------------------------------ */
/* Protocol                                                                  */
/* ------------------------------------------------------------------------ */

bool wcDecodeControl(const uint8_t *buf, size_t len, wcControl_t *out)
{
    if (buf == NULL || out == NULL || len != WC_CONTROL_LEN || buf[0] != WC_MSG_CONTROL) {
        return false;
    }

    wcControl_t c;
    c.flags = buf[1];
    c.seq = rd16(&buf[2]);
    c.clientMs = rd32(&buf[4]);
    c.roll = (int16_t)rd16(&buf[8]);
    c.pitch = (int16_t)rd16(&buf[10]);
    c.yaw = (int16_t)rd16(&buf[12]);
    c.thrust = rd16(&buf[14]);

    if ((c.flags & ~WC_FLAGS_KNOWN) != 0 ||
        !axisInRange(c.roll) || !axisInRange(c.pitch) || !axisInRange(c.yaw) ||
        c.thrust > WC_THRUST_MAX) {
        return false;
    }

    *out = c;
    return true;
}

size_t wcEncodeControl(const wcControl_t *c, uint8_t *buf, size_t cap)
{
    if (c == NULL || buf == NULL || cap < WC_CONTROL_LEN) {
        return 0;
    }
    buf[0] = WC_MSG_CONTROL;
    buf[1] = c->flags;
    wr16(&buf[2], c->seq);
    wr32(&buf[4], c->clientMs);
    wr16(&buf[8], (uint16_t)c->roll);
    wr16(&buf[10], (uint16_t)c->pitch);
    wr16(&buf[12], (uint16_t)c->yaw);
    wr16(&buf[14], c->thrust);
    return WC_CONTROL_LEN;
}

bool wcDecodeHello(const uint8_t *buf, size_t len, wcHello_t *out)
{
    if (buf == NULL || out == NULL || len != WC_HELLO_LEN || buf[0] != WC_MSG_HELLO) {
        return false;
    }
    out->version = buf[1];
    out->sessionToken = rd32(&buf[4]);
    return true;
}

static bool trimInRange(int8_t v)
{
    return v >= -WC_TRIM_MAX && v <= WC_TRIM_MAX;
}

/* TRIM layout: 0 type, 1 roll (i8), 2 pitch (i8), 3 reserved */
bool wcDecodeTrim(const uint8_t *buf, size_t len, wcTrim_t *out)
{
    if (buf == NULL || out == NULL || len != WC_TRIM_LEN || buf[0] != WC_MSG_TRIM) {
        return false;
    }
    const wcTrim_t t = {.roll = (int8_t)buf[1], .pitch = (int8_t)buf[2]};
    if (!trimInRange(t.roll) || !trimInRange(t.pitch)) {
        return false;
    }
    *out = t;
    return true;
}

size_t wcEncodeTrim(const wcTrim_t *t, uint8_t *buf, size_t cap)
{
    if (t == NULL || buf == NULL || cap < WC_TRIM_LEN) {
        return 0;
    }
    buf[0] = WC_MSG_TRIM;
    buf[1] = (uint8_t)t->roll;
    buf[2] = (uint8_t)t->pitch;
    buf[3] = 0;
    return WC_TRIM_LEN;
}

/*
 * TELEMETRY layout:
 *  0 type      1 state     2 stage      3 armResult
 *  4 echoSeq   6 rxRate    8 echoClientMs (4)
 * 12 vbatMv   14 rollCdeg 16 pitchCdeg 18 yawCdeg
 * 20 thrustOut 22 staleDrops 24 delayMs 26 state2 27 stopReason
 * 28 uptimeMs (4)         32 sessionToken (4)
 * 36 trim roll (i8)       37 trim pitch (i8)       38 reserved (2)
 */
size_t wcEncodeTelemetry(const wcTelemetry_t *t, uint8_t *buf, size_t cap)
{
    if (t == NULL || buf == NULL || cap < WC_TELEMETRY_LEN) {
        return 0;
    }
    memset(buf, 0, WC_TELEMETRY_LEN);
    buf[0] = WC_MSG_TELEMETRY;
    buf[1] = t->state;
    buf[2] = t->stage;
    buf[3] = t->armResult;
    wr16(&buf[4], t->echoSeq);
    wr16(&buf[6], t->rxRate);
    wr32(&buf[8], t->echoClientMs);
    wr16(&buf[12], t->vbatMv);
    wr16(&buf[14], (uint16_t)t->rollCdeg);
    wr16(&buf[16], (uint16_t)t->pitchCdeg);
    wr16(&buf[18], (uint16_t)t->yawCdeg);
    wr16(&buf[20], t->thrustOut);
    wr16(&buf[22], t->staleDrops);
    wr16(&buf[24], t->delayMs);
    buf[26] = t->state2;
    buf[27] = t->stopReason;
    wr32(&buf[28], t->uptimeMs);
    wr32(&buf[32], t->sessionToken);
    buf[36] = (uint8_t)t->trim.roll;
    buf[37] = (uint8_t)t->trim.pitch;
    return WC_TELEMETRY_LEN;
}

bool wcDecodeTelemetry(const uint8_t *buf, size_t len, wcTelemetry_t *out)
{
    if (buf == NULL || out == NULL || len != WC_TELEMETRY_LEN || buf[0] != WC_MSG_TELEMETRY) {
        return false;
    }
    out->state = buf[1];
    out->stage = buf[2];
    out->armResult = buf[3];
    out->echoSeq = rd16(&buf[4]);
    out->rxRate = rd16(&buf[6]);
    out->echoClientMs = rd32(&buf[8]);
    out->vbatMv = rd16(&buf[12]);
    out->rollCdeg = (int16_t)rd16(&buf[14]);
    out->pitchCdeg = (int16_t)rd16(&buf[16]);
    out->yawCdeg = (int16_t)rd16(&buf[18]);
    out->thrustOut = rd16(&buf[20]);
    out->staleDrops = rd16(&buf[22]);
    out->delayMs = rd16(&buf[24]);
    out->state2 = buf[26];
    out->stopReason = buf[27];
    out->uptimeMs = rd32(&buf[28]);
    out->sessionToken = rd32(&buf[32]);
    out->trim.roll = (int8_t)buf[36];
    out->trim.pitch = (int8_t)buf[37];
    return true;
}

/* ------------------------------------------------------------------------ */
/* Ground station messages                                                   */
/* ------------------------------------------------------------------------ */

static size_t boundedLen(const char *s, size_t max)
{
    size_t n = 0;
    while (n < max && s[n] != '\0') {
        n++;
    }
    return n;
}

static bool nameChar(char ch)
{
    return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '_';
}

bool wcParamNameValid(const char *name)
{
    if (name == NULL) {
        return false;
    }
    size_t n = 0;
    size_t dotPos = 0;
    int dots = 0;
    for (; name[n] != '\0'; n++) {
        if (n >= WC_PARAM_NAME_MAX) {
            return false;
        }
        if (name[n] == '.') {
            dots++;
            dotPos = n;
        } else if (!nameChar(name[n])) {
            return false;
        }
    }
    return dots == 1 && dotPos > 0 && dotPos + 1 < n;
}

/*
 * PARAM layout:       0 type  1 op  2 reqId  3 reserved  4 value (f32)  8 name
 * PARAM_REPLY layout: 0 type  1 op  2 reqId  3 status  4 value (f32)  8 default (f32)
 *                     12 flags  13 reserved  14 name
 * Names are not NUL terminated on the wire; their length follows from the frame.
 */
bool wcDecodeParamReq(const uint8_t *buf, size_t len, wcParamReq_t *out)
{
    if (buf == NULL || out == NULL || len < WC_PARAM_HDR_LEN || buf[0] != WC_MSG_PARAM ||
        len > WC_PARAM_HDR_LEN + WC_PARAM_NAME_MAX || buf[1] > WC_PARAM_RESET) {
        return false;
    }
    wcParamReq_t r;
    memset(&r, 0, sizeof(r));
    r.op = buf[1];
    r.reqId = buf[2];
    r.value = rdf(&buf[4]);
    const size_t n = len - WC_PARAM_HDR_LEN;
    memcpy(r.name, &buf[WC_PARAM_HDR_LEN], n);
    r.name[n] = '\0';

    const bool needsName = r.op == WC_PARAM_READ || r.op == WC_PARAM_WRITE;
    if (needsName ? !wcParamNameValid(r.name) : n != 0) {
        return false;
    }
    *out = r;
    return true;
}

size_t wcEncodeParamReq(const wcParamReq_t *r, uint8_t *buf, size_t cap)
{
    if (r == NULL || buf == NULL) {
        return 0;
    }
    const size_t n = boundedLen(r->name, sizeof(r->name));
    if (n > WC_PARAM_NAME_MAX || cap < WC_PARAM_HDR_LEN + n) {
        return 0;
    }
    buf[0] = WC_MSG_PARAM;
    buf[1] = r->op;
    buf[2] = r->reqId;
    buf[3] = 0;
    wrf(&buf[4], r->value);
    memcpy(&buf[WC_PARAM_HDR_LEN], r->name, n);
    return WC_PARAM_HDR_LEN + n;
}

size_t wcEncodeParamReply(const wcParamReply_t *r, uint8_t *buf, size_t cap)
{
    if (r == NULL || buf == NULL) {
        return 0;
    }
    const size_t n = boundedLen(r->name, sizeof(r->name));
    if (n > WC_PARAM_NAME_MAX || cap < WC_PARAM_REPLY_HDR_LEN + n) {
        return 0;
    }
    buf[0] = WC_MSG_PARAM_REPLY;
    buf[1] = r->op;
    buf[2] = r->reqId;
    buf[3] = r->status;
    wrf(&buf[4], r->value);
    wrf(&buf[8], r->dflt);
    buf[12] = r->flags;
    buf[13] = 0;
    memcpy(&buf[WC_PARAM_REPLY_HDR_LEN], r->name, n);
    return WC_PARAM_REPLY_HDR_LEN + n;
}

bool wcDecodeParamReply(const uint8_t *buf, size_t len, wcParamReply_t *out)
{
    if (buf == NULL || out == NULL || len < WC_PARAM_REPLY_HDR_LEN || buf[0] != WC_MSG_PARAM_REPLY ||
        len > WC_PARAM_REPLY_HDR_LEN + WC_PARAM_NAME_MAX) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->op = buf[1];
    out->reqId = buf[2];
    out->status = buf[3];
    out->value = rdf(&buf[4]);
    out->dflt = rdf(&buf[8]);
    out->flags = buf[12];
    memcpy(out->name, &buf[WC_PARAM_REPLY_HDR_LEN], len - WC_PARAM_REPLY_HDR_LEN);
    return true;
}

const char *const wcGainNames[WC_GAIN_COUNT] = {
    "pid_rate.roll_kp", "pid_rate.roll_ki", "pid_rate.roll_kd",
    "pid_rate.pitch_kp", "pid_rate.pitch_ki", "pid_rate.pitch_kd",
    "pid_rate.yaw_kp", "pid_rate.yaw_ki", "pid_rate.yaw_kd",
    "pid_attitude.roll_kp", "pid_attitude.roll_ki", "pid_attitude.roll_kd",
    "pid_attitude.pitch_kp", "pid_attitude.pitch_ki", "pid_attitude.pitch_kd",
    "pid_attitude.yaw_kp", "pid_attitude.yaw_ki", "pid_attitude.yaw_kd",
};

int wcGainIndex(const char *name)
{
    if (name == NULL) {
        return -1;
    }
    for (int i = 0; i < WC_GAIN_COUNT; i++) {
        if (strcmp(name, wcGainNames[i]) == 0) {
            return i;
        }
    }
    return -1;
}

bool wcGainValueValid(float v)
{
    return isfinite(v) && v >= 0.0f && v <= WC_GAIN_MAX;
}

size_t wcEncodeGainsBlob(const float *values, uint8_t *buf, size_t cap)
{
    if (values == NULL || buf == NULL || cap < WC_GAINS_BLOB_LEN) {
        return 0;
    }
    buf[0] = WC_GAINS_BLOB_VERSION;
    buf[1] = WC_GAIN_COUNT;
    buf[2] = 0;
    buf[3] = 0;
    for (int i = 0; i < WC_GAIN_COUNT; i++) {
        wrf(&buf[4 + 4 * i], values[i]);
    }
    return WC_GAINS_BLOB_LEN;
}

bool wcDecodeGainsBlob(const uint8_t *buf, size_t len, float *values, bool *valid)
{
    if (buf == NULL || values == NULL || valid == NULL || len != WC_GAINS_BLOB_LEN ||
        buf[0] != WC_GAINS_BLOB_VERSION || buf[1] != WC_GAIN_COUNT) {
        return false;
    }
    for (int i = 0; i < WC_GAIN_COUNT; i++) {
        values[i] = rdf(&buf[4 + 4 * i]);
        valid[i] = wcGainValueValid(values[i]);
    }
    return true;
}

/* STREAM layout: 0 type  1 flags  2 reserved (2). Unknown flags are ignored. */
bool wcDecodeStream(const uint8_t *buf, size_t len, uint8_t *flags)
{
    if (buf == NULL || flags == NULL || len != WC_STREAM_LEN || buf[0] != WC_MSG_STREAM) {
        return false;
    }
    *flags = buf[1] & WC_STREAM_FAST;
    return true;
}

size_t wcEncodeStream(uint8_t flags, uint8_t *buf, size_t cap)
{
    if (buf == NULL || cap < WC_STREAM_LEN) {
        return 0;
    }
    buf[0] = WC_MSG_STREAM;
    buf[1] = flags;
    buf[2] = 0;
    buf[3] = 0;
    return WC_STREAM_LEN;
}

int16_t wcCentiDeg(float deg)
{
    return wcDeciDeg(deg * 10.0f);
}

int16_t wcDeciDeg(float deg)
{
    const float v = deg * 10.0f;
    if (isnan(v)) {
        return 0;
    }
    if (v >= 32767.0f) {
        return 32767;
    }
    if (v <= -32767.0f) {
        return -32767;
    }
    return (int16_t)lrintf(v);
}

uint16_t wcU16Sat(float v)
{
    if (isnan(v) || v <= 0.0f) {
        return 0;
    }
    if (v >= 65535.0f) {
        return 65535;
    }
    return (uint16_t)lrintf(v);
}

/*
 * FAST layout: 0 type  1 count  2 seq  4 t0Ms (4)  8 periodMs  9 reserved (3)
 * 12 samples of 32 bytes: roll pitch yaw rollSp pitchSp (i16, 1/100 deg),
 * gyro roll pitch yaw, rate setpoint roll pitch yaw (i16, 1/10 deg/s),
 * M1..M4 (u16), thrust (u16)
 */
size_t wcEncodeFast(const wcFast_t *f, uint8_t *buf, size_t cap)
{
    if (f == NULL || buf == NULL || f->count == 0 || f->count > WC_FAST_SAMPLES) {
        return 0;
    }
    const size_t len = WC_FAST_HDR_LEN + (size_t)f->count * WC_FAST_SAMPLE_LEN;
    if (cap < len) {
        return 0;
    }
    memset(buf, 0, WC_FAST_HDR_LEN);
    buf[0] = WC_MSG_FAST;
    buf[1] = f->count;
    wr16(&buf[2], f->seq);
    wr32(&buf[4], f->t0Ms);
    buf[8] = f->periodMs;
    for (uint8_t i = 0; i < f->count; i++) {
        const wcFastSample_t *s = &f->s[i];
        uint8_t *p = &buf[WC_FAST_HDR_LEN + (size_t)i * WC_FAST_SAMPLE_LEN];
        const int16_t v16[11] = {s->rollCdeg, s->pitchCdeg, s->yawCdeg, s->rollSpCdeg, s->pitchSpCdeg,
                                 s->gyroDdps[0], s->gyroDdps[1], s->gyroDdps[2],
                                 s->rateSpDdps[0], s->rateSpDdps[1], s->rateSpDdps[2]};
        for (int k = 0; k < 11; k++) {
            wr16(&p[2 * k], (uint16_t)v16[k]);
        }
        for (int m = 0; m < 4; m++) {
            wr16(&p[22 + 2 * m], s->motor[m]);
        }
        wr16(&p[30], s->thrust);
    }
    return len;
}

bool wcDecodeFast(const uint8_t *buf, size_t len, wcFast_t *out)
{
    if (buf == NULL || out == NULL || len < WC_FAST_HDR_LEN || buf[0] != WC_MSG_FAST ||
        buf[1] == 0 || buf[1] > WC_FAST_SAMPLES ||
        len != WC_FAST_HDR_LEN + (size_t)buf[1] * WC_FAST_SAMPLE_LEN) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->count = buf[1];
    out->seq = rd16(&buf[2]);
    out->t0Ms = rd32(&buf[4]);
    out->periodMs = buf[8];
    for (uint8_t i = 0; i < out->count; i++) {
        wcFastSample_t *s = &out->s[i];
        const uint8_t *p = &buf[WC_FAST_HDR_LEN + (size_t)i * WC_FAST_SAMPLE_LEN];
        s->rollCdeg = (int16_t)rd16(&p[0]);
        s->pitchCdeg = (int16_t)rd16(&p[2]);
        s->yawCdeg = (int16_t)rd16(&p[4]);
        s->rollSpCdeg = (int16_t)rd16(&p[6]);
        s->pitchSpCdeg = (int16_t)rd16(&p[8]);
        for (int k = 0; k < 3; k++) {
            s->gyroDdps[k] = (int16_t)rd16(&p[10 + 2 * k]);
            s->rateSpDdps[k] = (int16_t)rd16(&p[16 + 2 * k]);
        }
        for (int m = 0; m < 4; m++) {
            s->motor[m] = rd16(&p[22 + 2 * m]);
        }
        s->thrust = rd16(&p[30]);
    }
    return true;
}

/* MOTOR_TEST layout: 0 type  1 mask  2 percent  3 reserved  4 durationMs  6 reserved (2)
 * MOTOR_REPLY layout: 0 type  1 mask  2 percent  3 status  4 durationMs  6 reserved (2) */
bool wcDecodeMotorTest(const uint8_t *buf, size_t len, wcMotorTest_t *out)
{
    if (buf == NULL || out == NULL || len != WC_MOTOR_TEST_LEN || buf[0] != WC_MSG_MOTOR_TEST) {
        return false;
    }
    const wcMotorTest_t m = {.mask = buf[1], .percent = buf[2], .durationMs = rd16(&buf[4])};
    if (m.mask > 0x0F || m.percent > 100 || (m.mask != 0 && (m.percent == 0 || m.durationMs == 0))) {
        return false;
    }
    *out = m;
    return true;
}

size_t wcEncodeMotorTest(const wcMotorTest_t *m, uint8_t *buf, size_t cap)
{
    if (m == NULL || buf == NULL || cap < WC_MOTOR_TEST_LEN) {
        return 0;
    }
    memset(buf, 0, WC_MOTOR_TEST_LEN);
    buf[0] = WC_MSG_MOTOR_TEST;
    buf[1] = m->mask;
    buf[2] = m->percent;
    wr16(&buf[4], m->durationMs);
    return WC_MOTOR_TEST_LEN;
}

size_t wcEncodeMotorReply(const wcMotorReply_t *r, uint8_t *buf, size_t cap)
{
    if (r == NULL || buf == NULL || cap < WC_MOTOR_REPLY_LEN) {
        return 0;
    }
    memset(buf, 0, WC_MOTOR_REPLY_LEN);
    buf[0] = WC_MSG_MOTOR_REPLY;
    buf[1] = r->mask;
    buf[2] = r->percent;
    buf[3] = r->status;
    wr16(&buf[4], r->durationMs);
    return WC_MOTOR_REPLY_LEN;
}

bool wcDecodeMotorReply(const uint8_t *buf, size_t len, wcMotorReply_t *out)
{
    if (buf == NULL || out == NULL || len != WC_MOTOR_REPLY_LEN || buf[0] != WC_MSG_MOTOR_REPLY) {
        return false;
    }
    out->mask = buf[1];
    out->percent = buf[2];
    out->status = buf[3];
    out->durationMs = rd16(&buf[4]);
    return true;
}

wcMotorStatus_t wcMotorTestCheck(const wcMotorTestInputs_t *in, const wcMotorTest_t *req,
                                 uint8_t maxPercent, wcMotorTest_t *applied)
{
    if (!in->enabled || maxPercent == 0) {
        return WC_MOTOR_DISABLED;
    }
    if (in->busy) {
        return WC_MOTOR_BUSY;
    }
    if (!in->systemReady) {
        return WC_MOTOR_NOT_READY;
    }
    applied->mask = req->mask;
    applied->percent = req->percent < maxPercent ? req->percent : maxPercent;
    applied->durationMs = req->durationMs < WC_MOTOR_TEST_MAX_MS ? req->durationMs : WC_MOTOR_TEST_MAX_MS;
    return WC_MOTOR_OK;
}

uint16_t wcMotorRatio(uint8_t percent)
{
    const uint32_t p = percent > 100 ? 100u : percent;
    return (uint16_t)(p * 65535u / 100u);
}

/* ------------------------------------------------------------------------ */
/* Flips                                                                     */
/* ------------------------------------------------------------------------ */

/* FLIP layout: 0 type  1 direction  2 reserved (2)
 * FLIP_REPLY layout: 0 type  1 direction  2 status  3 reserved */
bool wcDecodeFlip(const uint8_t *buf, size_t len, uint8_t *dir)
{
    if (buf == NULL || dir == NULL || len != WC_FLIP_LEN || buf[0] != WC_MSG_FLIP || buf[1] > WC_FLIP_RIGHT) {
        return false;
    }
    *dir = buf[1];
    return true;
}

size_t wcEncodeFlip(uint8_t dir, uint8_t *buf, size_t cap)
{
    if (buf == NULL || cap < WC_FLIP_LEN) {
        return 0;
    }
    buf[0] = WC_MSG_FLIP;
    buf[1] = dir;
    buf[2] = 0;
    buf[3] = 0;
    return WC_FLIP_LEN;
}

size_t wcEncodeFlipReply(uint8_t dir, uint8_t status, uint8_t *buf, size_t cap)
{
    if (buf == NULL || cap < WC_FLIP_REPLY_LEN) {
        return 0;
    }
    buf[0] = WC_MSG_FLIP_REPLY;
    buf[1] = dir;
    buf[2] = status;
    buf[3] = 0;
    return WC_FLIP_REPLY_LEN;
}

bool wcDecodeFlipReply(const uint8_t *buf, size_t len, uint8_t *dir, uint8_t *status)
{
    if (buf == NULL || dir == NULL || status == NULL || len != WC_FLIP_REPLY_LEN || buf[0] != WC_MSG_FLIP_REPLY) {
        return false;
    }
    *dir = buf[1];
    *status = buf[2];
    return true;
}

wcFlipStatus_t wcFlipCheck(const wcFlipInputs_t *in)
{
    if (!in->enabled) {
        return WC_FLIP_DISABLED;
    }
    if (!in->pilot) {
        return WC_FLIP_NOT_PILOT;
    }
    if (in->busy) {
        return WC_FLIP_BUSY;
    }
    if (!in->hoverKnown || !in->thrustUp) {
        return WC_FLIP_NOT_FLYING;
    }
    return WC_FLIP_OK;
}

void wcFlipStart(wcFlip_t *f, uint8_t dir, uint32_t nowMs, float hover)
{
    memset(f, 0, sizeof(*f));
    f->phase = WC_FLIP_BOOST;
    f->dir = dir;
    f->phaseMs = nowMs;
    f->hover = hover;
}

static float hoverShare(float hover, uint16_t percent)
{
    const float v = hover * (float)percent / 100.0f;
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

bool wcFlipUpdate(wcFlip_t *f, const wcFlipConfig_t *cfg, uint32_t nowMs, float rollDeg, float pitchDeg,
                  wcFlipOut_t *out)
{
    const float rad = 3.14159265f / 180.0f;
    memset(out, 0, sizeof(*out));

    if (f->phase == WC_FLIP_BOOST) {
        if (nowMs - f->phaseMs < cfg->boostMs) {
            out->thrust = hoverShare(f->hover, cfg->boostPct);
            return true;
        }
        f->phase = WC_FLIP_SPIN;
        f->phaseMs = nowMs;
    }

    if (f->phase == WC_FLIP_SPIN) {
        const float up = cosf(rollDeg * rad) * cosf(pitchDeg * rad);   /* 1 upright, -1 upside down */
        if (up < -0.5f) {
            f->inverted = true;
        }
        const bool done = f->inverted && up > cosf((float)cfg->exitDeg * rad);
        f->timedOut = !done && nowMs - f->phaseMs >= cfg->timeoutMs;
        if (!done && !f->timedOut) {
            const float rate = (float)cfg->rateDps;
            out->spin = true;
            switch (f->dir) {
            case WC_FLIP_FORWARD: out->pitchRateDps = -rate; break;
            case WC_FLIP_BACK:    out->pitchRateDps = rate;  break;
            case WC_FLIP_LEFT:    out->rollRateDps = -rate;  break;
            default:              out->rollRateDps = rate;   break;
            }
            const float thrust = hoverShare(f->hover, cfg->spinPct);
            out->thrust = thrust < WC_FLIP_MIN_THRUST ? WC_FLIP_MIN_THRUST : thrust;
            return true;
        }
        f->phase = WC_FLIP_RECOVER;
        f->phaseMs = nowMs;
    }

    if (f->phase == WC_FLIP_RECOVER) {
        if (nowMs - f->phaseMs < cfg->recoverMs) {
            out->thrust = hoverShare(f->hover, cfg->recoverPct);
            return true;
        }
        f->phase = WC_FLIP_IDLE;
    }
    return false;
}

/* ------------------------------------------------------------------------ */
/* Changeable parameters                                                     */
/* ------------------------------------------------------------------------ */

const wcTunable_t wcTunables[WC_TUNABLE_COUNT] = {
    {"pid_rate.roll_kp", 0, WC_GAIN_MAX}, {"pid_rate.roll_ki", 0, WC_GAIN_MAX}, {"pid_rate.roll_kd", 0, WC_GAIN_MAX},
    {"pid_rate.pitch_kp", 0, WC_GAIN_MAX}, {"pid_rate.pitch_ki", 0, WC_GAIN_MAX}, {"pid_rate.pitch_kd", 0, WC_GAIN_MAX},
    {"pid_rate.yaw_kp", 0, WC_GAIN_MAX}, {"pid_rate.yaw_ki", 0, WC_GAIN_MAX}, {"pid_rate.yaw_kd", 0, WC_GAIN_MAX},
    {"pid_attitude.roll_kp", 0, WC_GAIN_MAX}, {"pid_attitude.roll_ki", 0, WC_GAIN_MAX},
    {"pid_attitude.roll_kd", 0, WC_GAIN_MAX}, {"pid_attitude.pitch_kp", 0, WC_GAIN_MAX},
    {"pid_attitude.pitch_ki", 0, WC_GAIN_MAX}, {"pid_attitude.pitch_kd", 0, WC_GAIN_MAX},
    {"pid_attitude.yaw_kp", 0, WC_GAIN_MAX}, {"pid_attitude.yaw_ki", 0, WC_GAIN_MAX},
    {"pid_attitude.yaw_kd", 0, WC_GAIN_MAX},
    /* the same ranges as in Kconfig.projbuild */
    {"flip.rate", 360, 2000}, {"flip.boostMs", 0, 600}, {"flip.boost", 100, 250}, {"flip.spin", 10, 150},
    {"flip.exitDeg", 10, 120}, {"flip.recoverMs", 0, 1000}, {"flip.recover", 50, 250},
    {"flip.timeoutMs", 300, 1500},
};

int wcTunableIndex(const char *name)
{
    if (name == NULL) {
        return -1;
    }
    for (int i = 0; i < WC_TUNABLE_COUNT; i++) {
        if (strcmp(name, wcTunables[i].name) == 0) {
            return i;
        }
    }
    return -1;
}

bool wcTunableValid(int index, float v)
{
    return index >= 0 && index < WC_TUNABLE_COUNT && isfinite(v) &&
           v >= wcTunables[index].min && v <= wcTunables[index].max;
}

size_t wcEncodeFlipBlob(const float *values, uint8_t *buf, size_t cap)
{
    if (values == NULL || buf == NULL || cap < WC_FLIP_BLOB_LEN) {
        return 0;
    }
    buf[0] = WC_FLIP_BLOB_VERSION;
    buf[1] = WC_FLIP_PARAM_COUNT;
    buf[2] = 0;
    buf[3] = 0;
    for (int i = 0; i < WC_FLIP_PARAM_COUNT; i++) {
        wrf(&buf[4 + 4 * i], values[i]);
    }
    return WC_FLIP_BLOB_LEN;
}

bool wcDecodeFlipBlob(const uint8_t *buf, size_t len, float *values, bool *valid)
{
    if (buf == NULL || values == NULL || valid == NULL || len != WC_FLIP_BLOB_LEN ||
        buf[0] != WC_FLIP_BLOB_VERSION || buf[1] != WC_FLIP_PARAM_COUNT) {
        return false;
    }
    for (int i = 0; i < WC_FLIP_PARAM_COUNT; i++) {
        values[i] = rdf(&buf[4 + 4 * i]);
        valid[i] = wcTunableValid(WC_GAIN_COUNT + i, values[i]);
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* Link supervision                                                          */
/* ------------------------------------------------------------------------ */

wcStage_t wcStageForAge(const wcConfig_t *cfg, uint32_t ageMs)
{
    if (ageMs < cfg->okMs) {
        return WC_STAGE_OK;
    }
    if (ageMs < cfg->holdMs) {
        return WC_STAGE_HOLD;
    }
    if (ageMs < cfg->descendMs) {
        return WC_STAGE_LEVEL;
    }
    if (ageMs < cfg->offMs) {
        return WC_STAGE_DESCEND;
    }
    return WC_STAGE_OFF;
}

static float minf(float a, float b)
{
    return a < b ? a : b;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static float trimRollDeg(const wcTrim_t *trim)
{
    return trim ? trim->roll / 10.0f : 0.0f;
}

static float trimPitchDeg(const wcTrim_t *trim)
{
    return trim ? trim->pitch / 10.0f : 0.0f;
}

void wcControlToSetpoint(const wcConfig_t *cfg, const wcControl_t *c, const wcTrim_t *trim,
                         wcSetpoint_t *out)
{
    const float maxAngle = cfg->maxAngleDeg;
    out->roll = clampf(maxAngle * (float)c->roll / WC_AXIS_MAX + trimRollDeg(trim), -maxAngle, maxAngle);
    out->pitch = clampf(maxAngle * (float)c->pitch / WC_AXIS_MAX + trimPitchDeg(trim), -maxAngle, maxAngle);
    out->yawRate = cfg->maxYawRateDeg * (float)c->yaw / WC_AXIS_MAX;
    out->thrust = (float)c->thrust / WC_THRUST_MAX;
}

void wcFailsafeOutput(const wcConfig_t *cfg, wcStage_t stage, const wcSetpoint_t *last,
                      const wcHover_t *hover, const wcTrim_t *trim, wcSetpoint_t *out)
{
    const bool hoverValid = wcHoverValid(hover);

    switch (stage) {
    case WC_STAGE_OK:
    case WC_STAGE_HOLD:
        *out = *last;
        break;

    case WC_STAGE_LEVEL:
        // Attitude level (as trimmed), yaw stopped. Keep the thrust, but never
        // more than hover: a pilot who was climbing should not keep climbing.
        out->roll = trimRollDeg(trim);
        out->pitch = trimPitchDeg(trim);
        out->yawRate = 0;
        out->thrust = hoverValid ? minf(last->thrust, hover->value) : last->thrust;
        break;

    case WC_STAGE_DESCEND: {
        // Below hover thrust: the drone sinks. Without an altitude sensor this
        // is open loop, like the "Land" failsafe of Betaflight.
        const float base = hoverValid ? hover->value : last->thrust;
        out->roll = trimRollDeg(trim);
        out->pitch = trimPitchDeg(trim);
        out->yawRate = 0;
        out->thrust = minf(last->thrust, base * cfg->descendFactor);
        break;
    }

    case WC_STAGE_OFF:
    default:
        out->roll = 0;
        out->pitch = 0;
        out->yawRate = 0;
        out->thrust = 0;
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Hover estimate                                                            */
/* ------------------------------------------------------------------------ */

void wcHoverReset(wcHover_t *h)
{
    h->value = 0;
    h->flyingS = 0;
}

void wcHoverUpdate(wcHover_t *h, float thrust, float dtS)
{
    if (thrust < WC_HOVER_FLYING_THRUST || dtS <= 0) {
        return; // on the ground or idling: nothing to learn
    }

    if (h->flyingS <= 0) {
        h->value = thrust;
    } else {
        float alpha = dtS / (WC_HOVER_TAU_S + dtS);
        h->value += alpha * (thrust - h->value);
    }
    h->flyingS += dtS;
}

bool wcHoverValid(const wcHover_t *h)
{
    return h->flyingS >= WC_HOVER_MIN_FLYING_S;
}

/* ------------------------------------------------------------------------ */
/* Battery protection                                                        */
/* ------------------------------------------------------------------------ */

void wcBatteryReset(wcBattery_t *b)
{
    memset(b, 0, sizeof(*b));
}

void wcBatteryNewSession(wcBattery_t *b)
{
    b->lowTiming = false;
    b->landing = false;
}

static bool batteryPresent(const wcBattery_t *b)
{
    return b->volts >= WC_BAT_ABSENT_V;
}

wcBatEvent_t wcBatteryUpdate(wcBattery_t *b, const wcConfig_t *cfg, uint32_t nowMs, float dtS,
                             float volts, bool session, bool flying, float landFrom)
{
    // Low pass: motor current makes the voltage jump with the throttle.
    if (volts < WC_BAT_ABSENT_V) {
        b->volts = 0;
    } else if (!batteryPresent(b)) {
        b->volts = volts;   // first reading
    } else if (dtS > 0) {
        b->volts += dtS / (WC_BAT_FILTER_TAU_S + dtS) * (volts - b->volts);
    }

    if (!session) {
        wcBatteryNewSession(b);
        return WC_BAT_NONE;
    }
    if (b->landing) {
        return WC_BAT_NONE;   // stays on until the session ends
    }

    const bool low = cfg->batLandMv > 0 && batteryPresent(b) && b->volts * 1000.0f < cfg->batLandMv;
    if (!low) {
        b->lowTiming = false;
        return WC_BAT_NONE;
    }
    if (!b->lowTiming) {
        b->lowTiming = true;
        b->lowSinceMs = nowMs;
    }
    if ((uint32_t)(nowMs - b->lowSinceMs) < cfg->batLandDelayMs) {
        return WC_BAT_NONE;
    }

    b->lowTiming = false;
    if (!flying) {
        return WC_BAT_EMPTY;
    }
    b->landing = true;
    b->landStartMs = nowMs;
    b->landFrom = landFrom;
    return WC_BAT_LAND;
}

float wcBatteryThrustCap(const wcBattery_t *b, const wcConfig_t *cfg, uint32_t nowMs)
{
    if (!b->landing) {
        return 1.0f;
    }
    const uint32_t t = nowMs - b->landStartMs;
    if (t >= cfg->batLandTimeMs) {
        return 0.0f;
    }
    return b->landFrom * (1.0f - (float)t / (float)cfg->batLandTimeMs);
}

bool wcBatteryArmBlocked(const wcBattery_t *b, const wcConfig_t *cfg)
{
    return cfg->batArmMinMv > 0 && batteryPresent(b) && b->volts * 1000.0f < cfg->batArmMinMv;
}

/* ------------------------------------------------------------------------ */
/* Delay estimation                                                          */
/* ------------------------------------------------------------------------ */

void wcDelayReset(wcDelayEst_t *d)
{
    memset(d, 0, sizeof(*d));
}

uint32_t wcDelayUpdate(wcDelayEst_t *d, uint32_t droneMs, uint32_t clientMs)
{
    const int32_t offset = (int32_t)(droneMs - clientMs);

    if (!d->started) {
        wcDelayReset(d);
        d->started = true;
        d->bucketStartMs = droneMs;
        d->current = 0;
    }

    // Move to a new bucket every WC_DELAY_BUCKET_MS. After a long pause all
    // buckets are dropped, the estimate starts over. Arrival times are
    // monotonic in the firmware; a time stamp from the past simply stays in
    // the current bucket.
    int32_t elapsed = (int32_t)(droneMs - d->bucketStartMs);
    if (elapsed >= WC_DELAY_BUCKET_MS) {
        uint32_t steps = (uint32_t)elapsed / WC_DELAY_BUCKET_MS;
        if (steps > WC_DELAY_BUCKETS) {
            steps = WC_DELAY_BUCKETS;
        }
        for (uint32_t i = 0; i < steps; i++) {
            d->current = (uint8_t)((d->current + 1) % WC_DELAY_BUCKETS);
            d->bucketUsed[d->current] = false;
        }
        d->bucketStartMs = droneMs;
    }

    if (!d->bucketUsed[d->current] || offset < d->bucketMin[d->current]) {
        d->bucketMin[d->current] = offset;
        d->bucketUsed[d->current] = true;
    }

    int32_t minOffset = offset;
    for (int i = 0; i < WC_DELAY_BUCKETS; i++) {
        if (d->bucketUsed[i] && d->bucketMin[i] < minOffset) {
            minOffset = d->bucketMin[i];
        }
    }

    return (uint32_t)(offset - minOffset);
}

/* ------------------------------------------------------------------------ */
/* Pilot link                                                                */
/* ------------------------------------------------------------------------ */

void wcLinkReset(wcLink_t *l)
{
    memset(l, 0, sizeof(*l));
    wcDelayReset(&l->delay);
}

void wcLinkResume(wcLink_t *l)
{
    wcDelayReset(&l->delay);
    l->seqValid = false;
    l->paused = false;
}

wcFrameResult_t wcLinkOnControl(wcLink_t *l, const wcConfig_t *cfg, uint32_t droneMs,
                                const wcControl_t *c)
{
    if (l->seqValid && !wcSeqNewer(c->seq, l->lastSeq)) {
        return WC_FRAME_OLD_SEQ;
    }

    l->lastDelayMs = wcDelayUpdate(&l->delay, droneMs, c->clientMs);
    if (l->lastDelayMs > cfg->staleMs) {
        // Late frame, e.g. the front of a burst after a TCP retransmission.
        // The newest frame of the burst is usually fresh again.
        l->staleDrops++;
        return WC_FRAME_STALE;
    }

    l->haveFrame = true;
    l->seqValid = true;
    l->lastSeq = c->seq;
    l->lastFreshMs = droneMs;
    l->last = *c;
    l->paused = (c->flags & WC_FLAG_PAUSE) != 0;
    return WC_FRAME_ACCEPTED;
}

uint32_t wcLinkAge(const wcLink_t *l, const wcConfig_t *cfg, uint32_t nowMs)
{
    if (!l->haveFrame) {
        return cfg->offMs;
    }

    uint32_t age = nowMs - l->lastFreshMs;
    if ((!l->connected || l->paused) && age < cfg->holdMs) {
        age = cfg->holdMs;
    }
    return age;
}

/* ------------------------------------------------------------------------ */
/* Arm check                                                                 */
/* ------------------------------------------------------------------------ */

wcArmResult_t wcArmCheck(const wcArmInputs_t *in, const wcControl_t *cmd)
{
    if (!in->releasedSinceStop) {
        return WC_ARM_RELEASE_FIRST;
    }
    if (in->otherPilot) {
        return WC_ARM_OTHER_PILOT;
    }
    if (!in->systemReady) {
        return WC_ARM_NOT_READY;
    }
    if (in->tumbled) {
        return WC_ARM_TUMBLED;
    }
    if (in->lowBattery) {
        return WC_ARM_LOW_BATTERY;
    }
    if (cmd->thrust > WC_ARM_THRUST_MAX) {
        return WC_ARM_THRUST_NOT_ZERO;
    }
    return WC_ARM_OK;
}
