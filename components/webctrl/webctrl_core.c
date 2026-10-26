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

/*
 * TELEMETRY layout:
 *  0 type      1 state     2 stage      3 armResult
 *  4 echoSeq   6 rxRate    8 echoClientMs (4)
 * 12 vbatMv   14 rollCdeg 16 pitchCdeg 18 yawCdeg
 * 20 thrustOut 22 staleDrops 24 delayMs 26 reserved
 * 28 uptimeMs (4)         32 sessionToken (4)
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
    wr32(&buf[28], t->uptimeMs);
    wr32(&buf[32], t->sessionToken);
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
    out->uptimeMs = rd32(&buf[28]);
    out->sessionToken = rd32(&buf[32]);
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

void wcControlToSetpoint(const wcConfig_t *cfg, const wcControl_t *c, wcSetpoint_t *out)
{
    out->roll = cfg->maxAngleDeg * (float)c->roll / WC_AXIS_MAX;
    out->pitch = cfg->maxAngleDeg * (float)c->pitch / WC_AXIS_MAX;
    out->yawRate = cfg->maxYawRateDeg * (float)c->yaw / WC_AXIS_MAX;
    out->thrust = (float)c->thrust / WC_THRUST_MAX;
}

static float minf(float a, float b)
{
    return a < b ? a : b;
}

void wcFailsafeOutput(const wcConfig_t *cfg, wcStage_t stage, const wcSetpoint_t *last,
                      const wcHover_t *hover, wcSetpoint_t *out)
{
    const bool hoverValid = wcHoverValid(hover);

    switch (stage) {
    case WC_STAGE_OK:
    case WC_STAGE_HOLD:
        *out = *last;
        break;

    case WC_STAGE_LEVEL:
        // Attitude level, yaw stopped. Keep the thrust, but never more than
        // hover: a pilot who was climbing should not keep climbing.
        out->roll = 0;
        out->pitch = 0;
        out->yawRate = 0;
        out->thrust = hoverValid ? minf(last->thrust, hover->value) : last->thrust;
        break;

    case WC_STAGE_DESCEND: {
        // Below hover thrust: the drone sinks. Without an altitude sensor this
        // is open loop, like the "Land" failsafe of Betaflight.
        const float base = hoverValid ? hover->value : last->thrust;
        out->roll = 0;
        out->pitch = 0;
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
