/*
 * Host unit tests for webctrl_core.c (protocol, failsafe, delay estimation).
 * Build and run: make -C components/webctrl/host_test
 *
 * Copyright 2026 Philipp Weber
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "webctrl_core.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        checks++;                                                            \
        if (!(cond)) {                                                       \
            failures++;                                                      \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(a, b, eps) CHECK(fabsf((float)(a) - (float)(b)) <= (eps))

static const wcConfig_t cfg = {
    .okMs = 100,
    .holdMs = 300,
    .descendMs = 1000,
    .offMs = 4000,
    .staleMs = 250,
    .descendFactor = 0.85f,
    .maxAngleDeg = 15.0f,
    .maxYawRateDeg = 120.0f,
    .batLandMv = 3000,
    .batArmMinMv = 3400,
    .batLandDelayMs = 1000,
    .batLandTimeMs = 10000,
};

static wcControl_t ctrl(uint16_t seq, uint32_t clientMs, uint16_t thrust)
{
    wcControl_t c = {0};
    c.seq = seq;
    c.clientMs = clientMs;
    c.thrust = thrust;
    return c;
}

static void testControlRoundTrip(void)
{
    puts("control encode/decode");
    wcControl_t in = {.flags = WC_FLAG_ARM, .seq = 65535, .clientMs = 0xDEADBEEF,
                      .roll = -1000, .pitch = 1000, .yaw = -1, .thrust = 1000};
    uint8_t buf[WC_CONTROL_LEN];
    wcControl_t out;

    CHECK(wcEncodeControl(&in, buf, sizeof(buf)) == WC_CONTROL_LEN);
    CHECK(buf[0] == WC_MSG_CONTROL);
    CHECK(wcDecodeControl(buf, sizeof(buf), &out));
    CHECK(out.flags == in.flags && out.seq == in.seq && out.clientMs == in.clientMs);
    CHECK(out.roll == in.roll && out.pitch == in.pitch && out.yaw == in.yaw);
    CHECK(out.thrust == in.thrust);

    // wrong length, wrong type
    CHECK(!wcDecodeControl(buf, sizeof(buf) - 1, &out));
    uint8_t longer[WC_CONTROL_LEN + 1] = {0};
    memcpy(longer, buf, sizeof(buf));
    CHECK(!wcDecodeControl(longer, sizeof(longer), &out));
    buf[0] = WC_MSG_HELLO;
    CHECK(!wcDecodeControl(buf, sizeof(buf), &out));
    CHECK(!wcDecodeControl(NULL, WC_CONTROL_LEN, &out));

    // out of range values and unknown flags are rejected
    wcControl_t bad = in;
    bad.thrust = 1001;
    wcEncodeControl(&bad, buf, sizeof(buf));
    CHECK(!wcDecodeControl(buf, sizeof(buf), &out));
    bad = in;
    bad.roll = 1001;
    wcEncodeControl(&bad, buf, sizeof(buf));
    CHECK(!wcDecodeControl(buf, sizeof(buf), &out));
    bad = in;
    bad.yaw = -1001;
    wcEncodeControl(&bad, buf, sizeof(buf));
    CHECK(!wcDecodeControl(buf, sizeof(buf), &out));
    bad = in;
    bad.flags = 0x80;
    wcEncodeControl(&bad, buf, sizeof(buf));
    CHECK(!wcDecodeControl(buf, sizeof(buf), &out));
}

static void testHelloAndTelemetry(void)
{
    puts("hello / telemetry");
    const uint8_t hello[WC_HELLO_LEN] = {WC_MSG_HELLO, 2, 0, 0, 0x78, 0x56, 0x34, 0x12};
    wcHello_t h;
    CHECK(wcDecodeHello(hello, sizeof(hello), &h));
    CHECK(h.version == 2 && h.sessionToken == 0x12345678u);
    CHECK(!wcDecodeHello(hello, sizeof(hello) - 1, &h));

    wcTelemetry_t t = {.state = WC_STATE_ARMED | WC_STATE_YOU_PILOT, .stage = WC_STAGE_LEVEL,
                       .armResult = WC_ARM_OK, .echoSeq = 4711, .rxRate = 50,
                       .echoClientMs = 123456789, .vbatMv = 3912, .rollCdeg = -1234,
                       .pitchCdeg = 567, .yawCdeg = -17999, .thrustOut = 512,
                       .staleDrops = 3, .delayMs = 42, .state2 = WC_STATE2_BAT_LANDING | WC_STATE2_BAT_NO_ARM,
                       .stopReason = WC_STOP_BATTERY, .uptimeMs = 987654, .sessionToken = 0xCAFEF00D,
                       .trim = {.roll = -50, .pitch = 49}};
    uint8_t buf[64];
    wcTelemetry_t back;
    CHECK(wcEncodeTelemetry(&t, buf, sizeof(buf)) == WC_TELEMETRY_LEN);
    CHECK(wcEncodeTelemetry(&t, buf, WC_TELEMETRY_LEN - 1) == 0);
    CHECK(wcDecodeTelemetry(buf, WC_TELEMETRY_LEN, &back));
    CHECK(back.state == t.state && back.stage == t.stage && back.armResult == t.armResult);
    CHECK(back.echoSeq == t.echoSeq && back.rxRate == t.rxRate && back.echoClientMs == t.echoClientMs);
    CHECK(back.vbatMv == t.vbatMv && back.rollCdeg == t.rollCdeg && back.pitchCdeg == t.pitchCdeg);
    CHECK(back.yawCdeg == t.yawCdeg && back.thrustOut == t.thrustOut && back.staleDrops == t.staleDrops);
    CHECK(back.delayMs == t.delayMs && back.uptimeMs == t.uptimeMs && back.sessionToken == t.sessionToken);
    CHECK(back.state2 == t.state2 && back.stopReason == t.stopReason);
    CHECK(back.trim.roll == -50 && back.trim.pitch == 49);
    CHECK(buf[38] == 0 && buf[39] == 0);   // reserved bytes are zero
}

static void testSeq(void)
{
    puts("sequence numbers");
    CHECK(wcSeqNewer(1, 0));
    CHECK(!wcSeqNewer(0, 0));
    CHECK(!wcSeqNewer(0, 1));
    CHECK(wcSeqNewer(0, 65535));      // wrap
    CHECK(wcSeqNewer(10, 65530));     // wrap
    CHECK(!wcSeqNewer(65530, 10));
}

static void testStages(void)
{
    puts("failsafe stages");
    CHECK(wcStageForAge(&cfg, 0) == WC_STAGE_OK);
    CHECK(wcStageForAge(&cfg, 99) == WC_STAGE_OK);
    CHECK(wcStageForAge(&cfg, 100) == WC_STAGE_HOLD);
    CHECK(wcStageForAge(&cfg, 299) == WC_STAGE_HOLD);
    CHECK(wcStageForAge(&cfg, 300) == WC_STAGE_LEVEL);
    CHECK(wcStageForAge(&cfg, 999) == WC_STAGE_LEVEL);
    CHECK(wcStageForAge(&cfg, 1000) == WC_STAGE_DESCEND);
    CHECK(wcStageForAge(&cfg, 3999) == WC_STAGE_DESCEND);
    CHECK(wcStageForAge(&cfg, 4000) == WC_STAGE_OFF);
    CHECK(wcStageForAge(&cfg, 0xFFFFFFFFu) == WC_STAGE_OFF);
}

static void testFailsafeOutput(void)
{
    puts("failsafe outputs");
    wcHover_t hover;
    wcHoverReset(&hover);
    for (int i = 0; i < 300; i++) {   // 3 s of flight at 0.55
        wcHoverUpdate(&hover, 0.55f, 0.01f);
    }
    CHECK(wcHoverValid(&hover));
    CHECK_NEAR(hover.value, 0.55f, 0.001f);

    const wcSetpoint_t climbing = {.roll = 10, .pitch = -5, .yawRate = 90, .thrust = 0.8f};
    const wcSetpoint_t sinking = {.roll = 0, .pitch = 0, .yawRate = 0, .thrust = 0.4f};
    wcSetpoint_t out;

    wcFailsafeOutput(&cfg, WC_STAGE_HOLD, &climbing, &hover, NULL, &out);
    CHECK(memcmp(&out, &climbing, sizeof(out)) == 0);

    // level: attitude zero, climbing pilot is limited to hover thrust
    wcFailsafeOutput(&cfg, WC_STAGE_LEVEL, &climbing, &hover, NULL, &out);
    CHECK(out.roll == 0 && out.pitch == 0 && out.yawRate == 0);
    CHECK_NEAR(out.thrust, 0.55f, 0.001f);
    // ... a sinking pilot keeps sinking
    wcFailsafeOutput(&cfg, WC_STAGE_LEVEL, &sinking, &hover, NULL, &out);
    CHECK_NEAR(out.thrust, 0.4f, 0.001f);

    // descend: below hover
    wcFailsafeOutput(&cfg, WC_STAGE_DESCEND, &climbing, &hover, NULL, &out);
    CHECK_NEAR(out.thrust, 0.55f * 0.85f, 0.001f);
    CHECK(out.thrust < hover.value);
    wcFailsafeOutput(&cfg, WC_STAGE_DESCEND, &sinking, &hover, NULL, &out);
    CHECK_NEAR(out.thrust, 0.4f, 0.001f);

    // off
    wcFailsafeOutput(&cfg, WC_STAGE_OFF, &climbing, &hover, NULL, &out);
    CHECK(out.thrust == 0 && out.roll == 0 && out.yawRate == 0);

    // without a valid hover estimate the last thrust is the reference
    wcHover_t fresh;
    wcHoverReset(&fresh);
    wcFailsafeOutput(&cfg, WC_STAGE_LEVEL, &climbing, &fresh, NULL, &out);
    CHECK_NEAR(out.thrust, 0.8f, 0.001f);
    wcFailsafeOutput(&cfg, WC_STAGE_DESCEND, &climbing, &fresh, NULL, &out);
    CHECK_NEAR(out.thrust, 0.8f * 0.85f, 0.001f);
}

static void testHover(void)
{
    puts("hover estimate");
    wcHover_t h;
    wcHoverReset(&h);
    CHECK(!wcHoverValid(&h));
    for (int i = 0; i < 100; i++) {
        wcHoverUpdate(&h, 0.05f, 0.01f);   // on the ground: ignored
    }
    CHECK(!wcHoverValid(&h) && h.flyingS == 0);
    for (int i = 0; i < 150; i++) {
        wcHoverUpdate(&h, 0.6f, 0.01f);
    }
    CHECK(!wcHoverValid(&h));              // only 1.5 s
    for (int i = 0; i < 100; i++) {
        wcHoverUpdate(&h, 0.6f, 0.01f);
    }
    CHECK(wcHoverValid(&h));
    // follows a slow change with the 3 s time constant
    for (int i = 0; i < 300; i++) {
        wcHoverUpdate(&h, 0.5f, 0.01f);
    }
    CHECK(h.value > 0.5f && h.value < 0.54f);  // ~63 % of the step after 3 s
}

static void testDelayEstimator(void)
{
    puts("delay estimation");
    wcDelayEst_t d;
    wcDelayReset(&d);

    // client clock 1e9 ms ahead of the drone: offsets are negative, no problem
    uint32_t clientBase = 1000000000u;
    uint32_t extra = 0;
    for (int i = 0; i < 100; i++) {   // 2 s at 50 Hz, 5 ms network delay
        extra = wcDelayUpdate(&d, 10000 + i * 20 + 5, clientBase + i * 20);
        CHECK(extra == 0);
    }
    // jitter of 3 ms
    extra = wcDelayUpdate(&d, 10000 + 100 * 20 + 8, clientBase + 100 * 20);
    CHECK(extra == 3);
    // a frame delayed by 180 ms more (TCP keeps the order, so the next frame
    // arrives right behind it and is late as well)
    extra = wcDelayUpdate(&d, 10000 + 101 * 20 + 185, clientBase + 101 * 20);
    CHECK(extra == 180);
    extra = wcDelayUpdate(&d, 10000 + 101 * 20 + 186, clientBase + 102 * 20);
    CHECK(extra == 161);
    // back to normal: the minimum survived the delay
    extra = wcDelayUpdate(&d, 10000 + 103 * 20 + 5, clientBase + 103 * 20);
    CHECK(extra == 0);
    // a time stamp from the past does not wipe the history
    extra = wcDelayUpdate(&d, 10000 + 90 * 20 + 5, clientBase + 90 * 20);
    CHECK(extra == 0);
    extra = wcDelayUpdate(&d, 10000 + 104 * 20 + 50, clientBase + 104 * 20);
    CHECK(extra == 45);

    // clock drift: the client runs 0.1 % slow; old minima expire after 8 s
    wcDelayReset(&d);
    uint32_t maxExtra = 0;
    for (int i = 0; i < 1500; i++) {   // 30 s
        uint32_t drone = 50000 + i * 20;
        uint32_t client = (uint32_t)(i * 20 * 0.999);
        uint32_t e = wcDelayUpdate(&d, drone + 5, client);
        if (i > 500 && e > maxExtra) {
            maxExtra = e;
        }
    }
    CHECK(maxExtra <= 10);   // drift inside the 8 s window: 8 ms at most

    // wrap of the drone clock (u32 ms after 49.7 days)
    wcDelayReset(&d);
    for (int i = 0; i < 50; i++) {
        extra = wcDelayUpdate(&d, 0xFFFFFF00u + i * 20, 777 + i * 20);
        CHECK(extra == 0);
    }
}

static void testLinkBurstAfterStall(void)
{
    puts("link: TCP stall of 400 ms, then a burst");
    wcLink_t l;
    wcLinkReset(&l);
    l.connected = true;

    // 1 s of normal traffic at 50 Hz, 5 ms one-way delay
    uint32_t t = 0;
    uint16_t seq = 0;
    for (; t < 1000; t += 20) {
        wcControl_t c = ctrl(seq++, t, 500);
        CHECK(wcLinkOnControl(&l, &cfg, t + 5, &c) == WC_FRAME_ACCEPTED);
        CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, t + 6)) == WC_STAGE_OK);
    }

    // stall: frames sent from t=1000..1380 are held back by TCP. The drone
    // sees no new frame; the stage moves from OK to HOLD to LEVEL.
    uint32_t lastArrival = 1000 - 20 + 5;
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, lastArrival + 150)) == WC_STAGE_HOLD);
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, lastArrival + 350)) == WC_STAGE_LEVEL);

    // at t=1405 the retransmission lands and all 20 queued frames arrive at once
    int accepted = 0, stale = 0;
    uint16_t acceptedSeq = 0;
    for (uint32_t s = 1000; s <= 1380; s += 20) {
        wcControl_t c = ctrl(seq++, s, 500);
        wcFrameResult_t r = wcLinkOnControl(&l, &cfg, 1405, &c);
        if (r == WC_FRAME_ACCEPTED) {
            accepted++;
            acceptedSeq = c.seq;
        } else if (r == WC_FRAME_STALE) {
            stale++;
        }
    }
    // frames older than 250 ms are dropped, the recent ones are taken in order
    CHECK(stale == 8);       // sent at 1000..1140: 260..400 ms late
    CHECK(accepted == 12);   // sent at 1160..1380
    CHECK(acceptedSeq == (uint16_t)(seq - 1));   // newest frame wins
    CHECK(l.staleDrops == 8);
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 1406)) == WC_STAGE_OK);

    // a duplicate / older sequence number is ignored
    wcControl_t old = ctrl((uint16_t)(seq - 5), 1300, 999);
    CHECK(wcLinkOnControl(&l, &cfg, 1410, &old) == WC_FRAME_OLD_SEQ);
    CHECK(l.last.thrust == 500);
}

static void testLinkDisconnectAndPause(void)
{
    puts("link: disconnect and hidden page");
    wcLink_t l;
    wcLinkReset(&l);
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 12345)) == WC_STAGE_OFF);   // never had a frame

    l.connected = true;
    wcControl_t c = ctrl(1, 100, 600);
    CHECK(wcLinkOnControl(&l, &cfg, 200, &c) == WC_FRAME_ACCEPTED);
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 210)) == WC_STAGE_OK);

    // socket closed 10 ms after the last frame: level out at once
    l.connected = false;
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 210)) == WC_STAGE_LEVEL);
    // the timeline still counts from the last fresh frame
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 1250)) == WC_STAGE_DESCEND);
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 4200)) == WC_STAGE_OFF);

    // reconnect within the descent: fresh frames give control back at once
    l.connected = true;
    c = ctrl(2, 2100, 550);
    CHECK(wcLinkOnControl(&l, &cfg, 2200, &c) == WC_FRAME_ACCEPTED);
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 2210)) == WC_STAGE_OK);

    // page hidden: the client sends PAUSE, the drone levels out right away
    c = ctrl(3, 2120, 550);
    c.flags = WC_FLAG_PAUSE;
    CHECK(wcLinkOnControl(&l, &cfg, 2220, &c) == WC_FRAME_ACCEPTED);
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 2225)) == WC_STAGE_LEVEL);
    c = ctrl(4, 2140, 550);
    CHECK(wcLinkOnControl(&l, &cfg, 2240, &c) == WC_FRAME_ACCEPTED);
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 2245)) == WC_STAGE_OK);
}

static void testLinkResume(void)
{
    puts("link: resume on a new connection");
    wcLink_t l;
    wcLinkReset(&l);
    l.connected = true;
    wcControl_t c = ctrl(5000, 900000, 600);
    CHECK(wcLinkOnControl(&l, &cfg, 1000, &c) == WC_FRAME_ACCEPTED);

    // connection lost, the page reloads: seq and client clock start over
    l.connected = false;
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 1500)) == WC_STAGE_LEVEL);
    wcLinkResume(&l);
    l.connected = true;
    // still in failsafe until a fresh frame arrives
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 1600)) == WC_STAGE_LEVEL);
    CHECK(l.last.thrust == 600);
    c = ctrl(0, 12, 580);   // seq restarted at 0, clock at 12 ms
    CHECK(wcLinkOnControl(&l, &cfg, 1700, &c) == WC_FRAME_ACCEPTED);
    CHECK(wcStageForAge(&cfg, wcLinkAge(&l, &cfg, 1710)) == WC_STAGE_OK);
    c = ctrl(1, 32, 570);
    CHECK(wcLinkOnControl(&l, &cfg, 1720, &c) == WC_FRAME_ACCEPTED);
    CHECK(l.lastDelayMs == 0);
}

static void testArmCheck(void)
{
    puts("arm check");
    wcArmInputs_t in = {.systemReady = true, .lowBattery = false, .tumbled = false,
                        .otherPilot = false, .releasedSinceStop = true};
    wcControl_t c = ctrl(0, 0, 0);
    c.flags = WC_FLAG_ARM;

    CHECK(wcArmCheck(&in, &c) == WC_ARM_OK);
    c.thrust = WC_ARM_THRUST_MAX;
    CHECK(wcArmCheck(&in, &c) == WC_ARM_OK);
    c.thrust = WC_ARM_THRUST_MAX + 1;
    CHECK(wcArmCheck(&in, &c) == WC_ARM_THRUST_NOT_ZERO);
    c.thrust = 0;

    wcArmInputs_t x = in;
    x.systemReady = false;
    CHECK(wcArmCheck(&x, &c) == WC_ARM_NOT_READY);
    x = in;
    x.lowBattery = true;
    CHECK(wcArmCheck(&x, &c) == WC_ARM_LOW_BATTERY);
    x = in;
    x.tumbled = true;
    CHECK(wcArmCheck(&x, &c) == WC_ARM_TUMBLED);
    x = in;
    x.otherPilot = true;
    CHECK(wcArmCheck(&x, &c) == WC_ARM_OTHER_PILOT);
    x = in;
    x.releasedSinceStop = false;
    x.otherPilot = true;   // "release first" is reported before anything else
    CHECK(wcArmCheck(&x, &c) == WC_ARM_RELEASE_FIRST);
}

static void testControlToSetpoint(void)
{
    puts("stick to setpoint");
    wcControl_t c = {.roll = 1000, .pitch = -500, .yaw = 250, .thrust = 1000};
    wcSetpoint_t sp;
    wcControlToSetpoint(&cfg, &c, NULL, &sp);
    CHECK_NEAR(sp.roll, 15.0f, 0.001f);
    CHECK_NEAR(sp.pitch, -7.5f, 0.001f);
    CHECK_NEAR(sp.yawRate, 30.0f, 0.001f);
    CHECK_NEAR(sp.thrust, 1.0f, 0.001f);
}

static void testTrim(void)
{
    puts("trim");
    const wcTrim_t in = {.roll = -50, .pitch = 7};
    uint8_t buf[WC_TRIM_LEN];
    wcTrim_t out;
    CHECK(wcEncodeTrim(&in, buf, sizeof(buf)) == WC_TRIM_LEN);
    CHECK(wcEncodeTrim(&in, buf, WC_TRIM_LEN - 1) == 0);
    CHECK(buf[0] == WC_MSG_TRIM && buf[3] == 0);
    CHECK(wcDecodeTrim(buf, sizeof(buf), &out));
    CHECK(out.roll == -50 && out.pitch == 7);

    // wrong length or type, values beyond +-5 deg
    CHECK(!wcDecodeTrim(buf, WC_TRIM_LEN - 1, &out));
    uint8_t bad[WC_TRIM_LEN] = {WC_MSG_TRIM, (uint8_t)(int8_t)(WC_TRIM_MAX + 1), 0, 0};
    CHECK(!wcDecodeTrim(bad, sizeof(bad), &out));
    bad[1] = (uint8_t)(int8_t)(-WC_TRIM_MAX - 1);
    CHECK(!wcDecodeTrim(bad, sizeof(bad), &out));
    bad[1] = 0;
    bad[2] = (uint8_t)(int8_t)-128;
    CHECK(!wcDecodeTrim(bad, sizeof(bad), &out));
    bad[2] = 0;
    bad[0] = WC_MSG_HELLO;
    CHECK(!wcDecodeTrim(bad, sizeof(bad), &out));
    CHECK(!wcDecodeTrim(NULL, WC_TRIM_LEN, &out));

    // the trim adds to the stick angle; the sum stays within the max angle
    const wcTrim_t t = {.roll = 20, .pitch = -15};   // 2.0 deg right, 1.5 deg backward
    wcControl_t c = {.roll = 0, .pitch = 0, .yaw = 500, .thrust = 500};
    wcSetpoint_t sp;
    wcControlToSetpoint(&cfg, &c, &t, &sp);
    CHECK_NEAR(sp.roll, 2.0f, 0.001f);
    CHECK_NEAR(sp.pitch, -1.5f, 0.001f);
    CHECK_NEAR(sp.yawRate, 60.0f, 0.001f);   // yaw and thrust are not trimmed
    CHECK_NEAR(sp.thrust, 0.5f, 0.001f);
    c.roll = 1000;
    c.pitch = -1000;
    wcControlToSetpoint(&cfg, &c, &t, &sp);
    CHECK_NEAR(sp.roll, 15.0f, 0.001f);      // 17 limited to the max angle
    CHECK_NEAR(sp.pitch, -15.0f, 0.001f);    // -16.5 limited
    c.roll = -1000;
    c.pitch = 1000;
    wcControlToSetpoint(&cfg, &c, &t, &sp);
    CHECK_NEAR(sp.roll, -13.0f, 0.001f);
    CHECK_NEAR(sp.pitch, 13.5f, 0.001f);

    // the failsafe levels out to the trim; OFF is plain zero
    wcHover_t hover;
    wcHoverReset(&hover);
    const wcSetpoint_t last = {.roll = 10, .pitch = 5, .yawRate = 30, .thrust = 0.5f};
    wcSetpoint_t fs;
    wcFailsafeOutput(&cfg, WC_STAGE_LEVEL, &last, &hover, &t, &fs);
    CHECK_NEAR(fs.roll, 2.0f, 0.001f);
    CHECK_NEAR(fs.pitch, -1.5f, 0.001f);
    CHECK(fs.yawRate == 0);
    wcFailsafeOutput(&cfg, WC_STAGE_DESCEND, &last, &hover, &t, &fs);
    CHECK_NEAR(fs.roll, 2.0f, 0.001f);
    CHECK_NEAR(fs.pitch, -1.5f, 0.001f);
    wcFailsafeOutput(&cfg, WC_STAGE_OFF, &last, &hover, &t, &fs);
    CHECK(fs.roll == 0 && fs.pitch == 0 && fs.thrust == 0);
}

/* Feed a constant voltage for ms milliseconds in 10 ms steps. Returns the first
 * event (and its time in *eventAt), WC_BAT_NONE if there was none. */
static wcBatEvent_t runBattery(wcBattery_t *b, const wcConfig_t *c, uint32_t *t, uint32_t ms,
                               float volts, bool session, bool flying, uint32_t *eventAt)
{
    wcBatEvent_t first = WC_BAT_NONE;
    for (uint32_t i = 0; i < ms; i += 10) {
        *t += 10;
        const wcBatEvent_t e = wcBatteryUpdate(b, c, *t, 0.01f, volts, session, flying, 0.5f);
        if (first == WC_BAT_NONE && e != WC_BAT_NONE) {
            first = e;
            if (eventAt != NULL) {
                *eventAt = *t;
            }
        }
    }
    return first;
}

static void testBattery(void)
{
    puts("battery protection");
    wcBattery_t b;
    wcConfig_t off = cfg;
    off.batLandMv = 0;
    off.batArmMinMv = 0;
    uint32_t t = 0, at = 0;

    // filter: starts at the first reading, then a 0.5 s time constant
    wcBatteryReset(&b);
    CHECK(wcBatteryUpdate(&b, &cfg, 0, 0.01f, 3.90f, false, false, 0) == WC_BAT_NONE);
    CHECK_NEAR(b.volts, 3.90f, 0.001f);
    runBattery(&b, &cfg, &t, 500, 3.70f, false, false, NULL);
    CHECK(b.volts > 3.75f && b.volts < 3.80f);   // ~63 % of the step

    // no battery (USB power on the bench): no reading, no checks
    wcBatteryUpdate(&b, &cfg, t, 0.01f, 0.1f, false, false, 0);
    CHECK(b.volts == 0);
    CHECK(!wcBatteryArmBlocked(&b, &cfg));
    CHECK(runBattery(&b, &cfg, &t, 5000, 0.0f, true, true, NULL) == WC_BAT_NONE);
    CHECK(!b.landing);

    // arming needs batArmMinMv
    wcBatteryReset(&b);
    wcBatteryUpdate(&b, &cfg, t, 0.01f, 3.35f, false, false, 0);
    CHECK(wcBatteryArmBlocked(&b, &cfg));
    CHECK(!wcBatteryArmBlocked(&b, &off));
    wcBatteryReset(&b);
    wcBatteryUpdate(&b, &cfg, t, 0.01f, 3.45f, false, false, 0);
    CHECK(!wcBatteryArmBlocked(&b, &cfg));

    // in flight a short dip (0.6 s, e.g. a climb) does not start the landing
    wcBatteryReset(&b);
    t = 1000;
    CHECK(runBattery(&b, &cfg, &t, 2000, 3.30f, true, true, NULL) == WC_BAT_NONE);
    CHECK(runBattery(&b, &cfg, &t, 600, 2.80f, true, true, NULL) == WC_BAT_NONE);
    CHECK(runBattery(&b, &cfg, &t, 1000, 3.30f, true, true, NULL) == WC_BAT_NONE);
    CHECK(!b.landing);

    // a lasting drop does: filter (~0.7 s to cross 3.0 V) plus 1 s delay
    const uint32_t start = t;
    CHECK(runBattery(&b, &cfg, &t, 3000, 2.90f, true, true, &at) == WC_BAT_LAND);
    CHECK(at - start >= 1600 && at - start <= 1800);
    CHECK(b.landing);
    CHECK_NEAR(b.landFrom, 0.5f, 0.001f);

    // the thrust cap ramps from landFrom to zero within batLandTimeMs
    CHECK_NEAR(wcBatteryThrustCap(&b, &cfg, at), 0.5f, 0.001f);
    CHECK_NEAR(wcBatteryThrustCap(&b, &cfg, at + 5000), 0.25f, 0.001f);
    const float nearEnd = wcBatteryThrustCap(&b, &cfg, at + 9990);
    CHECK(nearEnd > 0.0f && nearEnd < 0.001f);
    CHECK(wcBatteryThrustCap(&b, &cfg, at + 10000) == 0.0f);
    CHECK(wcBatteryThrustCap(&b, &cfg, at + 60000) == 0.0f);

    // latched: a recovering voltage does not end it, only the session end does
    CHECK(runBattery(&b, &cfg, &t, 2000, 3.70f, true, true, NULL) == WC_BAT_NONE);
    CHECK(b.landing);
    wcBatteryUpdate(&b, &cfg, t, 0.01f, 3.70f, false, false, 0);
    CHECK(!b.landing);
    CHECK(wcBatteryThrustCap(&b, &cfg, t) == 1.0f);

    // armed on the ground with an empty battery: disarm, no landing
    wcBatteryReset(&b);
    t = 0;
    CHECK(runBattery(&b, &cfg, &t, 3000, 2.90f, true, false, NULL) == WC_BAT_EMPTY);
    CHECK(!b.landing);

    // switched off in menuconfig
    wcBatteryReset(&b);
    t = 0;
    CHECK(runBattery(&b, &off, &t, 5000, 2.50f, true, true, NULL) == WC_BAT_NONE);
    CHECK(!b.landing && wcBatteryThrustCap(&b, &off, t) == 1.0f);

    // a new session starts without landing; the filtered voltage stays
    wcBatteryReset(&b);
    t = 0;
    CHECK(runBattery(&b, &cfg, &t, 3000, 2.90f, true, true, NULL) == WC_BAT_LAND);
    const float v = b.volts;
    wcBatteryNewSession(&b);
    CHECK(!b.landing && b.volts == v);
}

static void testParamCodec(void)
{
    puts("param encode/decode");
    CHECK(wcParamNameValid("pid_rate.roll_kp"));
    CHECK(wcParamNameValid("a.b"));
    CHECK(wcParamNameValid("motorPowerSet.m1"));
    CHECK(!wcParamNameValid(NULL));
    CHECK(!wcParamNameValid(""));
    CHECK(!wcParamNameValid("pid_rate"));           // no dot
    CHECK(!wcParamNameValid(".roll_kp"));           // empty group
    CHECK(!wcParamNameValid("pid_rate."));          // empty name
    CHECK(!wcParamNameValid("a.b.c"));              // two dots
    CHECK(!wcParamNameValid("pid rate.kp"));        // space
    CHECK(!wcParamNameValid("pid-rate.kp"));
    CHECK(!wcParamNameValid("pid_rate.roll_kp\n"));
    CHECK(wcParamNameValid("abcdefghijklmno.abcdefghijklmno"));    // 31 characters
    CHECK(!wcParamNameValid("abcdefghijklmnop.abcdefghijklmno"));  // 32

    uint8_t buf[64];
    wcParamReq_t in = {.op = WC_PARAM_WRITE, .reqId = 200, .value = 137.25f};
    strcpy(in.name, "pid_rate.pitch_kd");
    size_t n = wcEncodeParamReq(&in, buf, sizeof(buf));
    CHECK(n == WC_PARAM_HDR_LEN + strlen(in.name));
    CHECK(buf[0] == WC_MSG_PARAM && buf[1] == WC_PARAM_WRITE && buf[2] == 200);
    wcParamReq_t out;
    CHECK(wcDecodeParamReq(buf, n, &out));
    CHECK(out.op == WC_PARAM_WRITE && out.reqId == 200 && out.value == 137.25f);
    CHECK(strcmp(out.name, "pid_rate.pitch_kd") == 0);

    // the name ends with the frame, no terminator on the wire
    CHECK(!wcDecodeParamReq(buf, n - 1, &out) || strcmp(out.name, "pid_rate.pitch_k") == 0);
    CHECK(!wcDecodeParamReq(buf, WC_PARAM_HDR_LEN, &out));         // READ/WRITE need a name
    CHECK(!wcDecodeParamReq(buf, WC_PARAM_HDR_LEN - 1, &out));
    buf[1] = WC_PARAM_RESET + 1;
    CHECK(!wcDecodeParamReq(buf, n, &out));                        // unknown op
    buf[1] = WC_PARAM_SAVE;
    CHECK(!wcDecodeParamReq(buf, n, &out));                        // SAVE without name only
    CHECK(wcDecodeParamReq(buf, WC_PARAM_HDR_LEN, &out) && out.op == WC_PARAM_SAVE && out.name[0] == '\0');
    buf[1] = WC_PARAM_READ;
    buf[0] = WC_MSG_TRIM;
    CHECK(!wcDecodeParamReq(buf, n, &out));
    CHECK(!wcDecodeParamReq(NULL, n, &out));

    // a frame longer than header + 31 characters is rejected
    uint8_t big[WC_PARAM_HDR_LEN + WC_PARAM_NAME_MAX + 1];
    memset(big, 'a', sizeof(big));
    big[0] = WC_MSG_PARAM;
    big[1] = WC_PARAM_READ;
    big[WC_PARAM_HDR_LEN + 3] = '.';
    CHECK(!wcDecodeParamReq(big, sizeof(big), &out));
    CHECK(wcDecodeParamReq(big, sizeof(big) - 1, &out));
    CHECK(strlen(out.name) == WC_PARAM_NAME_MAX);

    // encoder refuses a buffer that is too small
    CHECK(wcEncodeParamReq(&in, buf, WC_PARAM_HDR_LEN + 3) == 0);

    wcParamReply_t rep = {.op = WC_PARAM_READ, .reqId = 7, .status = WC_PARAM_OK,
                          .flags = WC_PARAMF_WRITABLE | WC_PARAMF_STORED, .value = 5.9f, .dflt = NAN};
    strcpy(rep.name, "pid_attitude.roll_kp");
    n = wcEncodeParamReply(&rep, buf, sizeof(buf));
    CHECK(n == WC_PARAM_REPLY_HDR_LEN + strlen(rep.name));
    wcParamReply_t rback;
    CHECK(wcDecodeParamReply(buf, n, &rback));
    CHECK(rback.op == WC_PARAM_READ && rback.reqId == 7 && rback.status == WC_PARAM_OK);
    CHECK(rback.flags == (WC_PARAMF_WRITABLE | WC_PARAMF_STORED));
    CHECK(rback.value == 5.9f && isnan(rback.dflt));
    CHECK(strcmp(rback.name, rep.name) == 0);
    CHECK(!wcDecodeParamReply(buf, WC_PARAM_REPLY_HDR_LEN - 1, &rback));
    CHECK(n <= WC_PARAM_MAX_LEN);
}

static void testGains(void)
{
    puts("gains");
    for (int i = 0; i < WC_GAIN_COUNT; i++) {
        CHECK(wcParamNameValid(wcGainNames[i]));
        CHECK(wcGainIndex(wcGainNames[i]) == i);
    }
    CHECK(wcGainIndex("pid_rate.roll") == -1);
    CHECK(wcGainIndex("motorPowerSet.m1") == -1);
    CHECK(wcGainIndex(NULL) == -1);

    CHECK(wcGainValueValid(0.0f));
    CHECK(wcGainValueValid(137.0f));
    CHECK(wcGainValueValid(WC_GAIN_MAX));
    CHECK(!wcGainValueValid(-0.1f));
    CHECK(!wcGainValueValid(WC_GAIN_MAX + 1.0f));
    CHECK(!wcGainValueValid(NAN));
    CHECK(!wcGainValueValid(INFINITY));

    float v[WC_GAIN_COUNT], back[WC_GAIN_COUNT];
    bool valid[WC_GAIN_COUNT];
    for (int i = 0; i < WC_GAIN_COUNT; i++) {
        v[i] = 0.5f * (float)i;
    }
    v[4] = NAN;
    v[5] = -3.0f;
    uint8_t blob[WC_GAINS_BLOB_LEN];
    CHECK(wcEncodeGainsBlob(v, blob, sizeof(blob)) == WC_GAINS_BLOB_LEN);
    CHECK(wcEncodeGainsBlob(v, blob, sizeof(blob) - 1) == 0);
    CHECK(wcDecodeGainsBlob(blob, sizeof(blob), back, valid));
    int okCount = 0;
    for (int i = 0; i < WC_GAIN_COUNT; i++) {
        okCount += valid[i] ? 1 : 0;
        if (valid[i]) {
            CHECK(back[i] == v[i]);
        }
    }
    CHECK(okCount == WC_GAIN_COUNT - 2 && !valid[4] && !valid[5]);
    CHECK(!wcDecodeGainsBlob(blob, sizeof(blob) - 1, back, valid));
    blob[0] = WC_GAINS_BLOB_VERSION + 1;
    CHECK(!wcDecodeGainsBlob(blob, sizeof(blob), back, valid));
    blob[0] = WC_GAINS_BLOB_VERSION;
    blob[1] = WC_GAIN_COUNT - 1;
    CHECK(!wcDecodeGainsBlob(blob, sizeof(blob), back, valid));
}

static void testStreamAndFast(void)
{
    puts("stream and fast samples");
    uint8_t buf[WC_FAST_MAX_LEN + 8];
    uint8_t flags = 0xFF;
    CHECK(wcEncodeStream(WC_STREAM_FAST, buf, sizeof(buf)) == WC_STREAM_LEN);
    CHECK(wcDecodeStream(buf, WC_STREAM_LEN, &flags) && flags == WC_STREAM_FAST);
    buf[1] = 0xFE;                                   // unknown bits are ignored
    CHECK(wcDecodeStream(buf, WC_STREAM_LEN, &flags) && flags == 0);
    CHECK(!wcDecodeStream(buf, WC_STREAM_LEN + 1, &flags));
    buf[0] = WC_MSG_TRIM;
    CHECK(!wcDecodeStream(buf, WC_STREAM_LEN, &flags));

    wcFast_t f;
    memset(&f, 0, sizeof(f));
    f.seq = 65534;
    f.t0Ms = 123456789;
    f.periodMs = WC_FAST_PERIOD_MS;
    f.count = WC_FAST_SAMPLES;
    for (int i = 0; i < WC_FAST_SAMPLES; i++) {
        wcFastSample_t *s = &f.s[i];
        s->rollCdeg = (int16_t)(100 * i - 250);
        s->pitchCdeg = (int16_t)(-32767 + i);
        s->yawCdeg = 18000;
        s->rollSpCdeg = (int16_t)(-1500 + i);
        s->pitchSpCdeg = 1500;
        s->gyroDdps[0] = -20000;
        s->gyroDdps[1] = 3;
        s->gyroDdps[2] = (int16_t)i;
        s->rateSpDdps[0] = 32767;
        s->rateSpDdps[1] = -1;
        s->rateSpDdps[2] = 0;
        s->motor[0] = 65535;
        s->motor[1] = 0;
        s->motor[2] = (uint16_t)(1000 * i);
        s->motor[3] = 30000;
        s->thrust = 42000;
    }
    size_t n = wcEncodeFast(&f, buf, sizeof(buf));
    CHECK(n == WC_FAST_MAX_LEN);
    CHECK(buf[0] == WC_MSG_FAST && buf[1] == WC_FAST_SAMPLES);
    wcFast_t g;
    CHECK(wcDecodeFast(buf, n, &g));
    CHECK(g.seq == f.seq && g.t0Ms == f.t0Ms && g.periodMs == f.periodMs && g.count == f.count);
    CHECK(memcmp(g.s, f.s, sizeof(f.s)) == 0);
    CHECK(!wcDecodeFast(buf, n - 1, &g));
    CHECK(!wcDecodeFast(buf, n + 1, &g));
    buf[1] = 2;                                      // count does not match the length
    CHECK(!wcDecodeFast(buf, n, &g));
    buf[1] = 0;
    CHECK(!wcDecodeFast(buf, WC_FAST_HDR_LEN, &g));

    f.count = 1;
    n = wcEncodeFast(&f, buf, sizeof(buf));
    CHECK(n == WC_FAST_HDR_LEN + WC_FAST_SAMPLE_LEN);
    CHECK(wcDecodeFast(buf, n, &g) && g.count == 1 && memcmp(&g.s[0], &f.s[0], sizeof(f.s[0])) == 0);
    f.count = 0;
    CHECK(wcEncodeFast(&f, buf, sizeof(buf)) == 0);
    f.count = WC_FAST_SAMPLES + 1;
    CHECK(wcEncodeFast(&f, buf, sizeof(buf)) == 0);
    f.count = WC_FAST_SAMPLES;
    CHECK(wcEncodeFast(&f, buf, WC_FAST_MAX_LEN - 1) == 0);

    CHECK(wcCentiDeg(12.345f) == 1235 || wcCentiDeg(12.345f) == 1234);
    CHECK(wcCentiDeg(-1.0f) == -100);
    CHECK(wcCentiDeg(400.0f) == 32767);
    CHECK(wcCentiDeg(-400.0f) == -32767);
    CHECK(wcCentiDeg(NAN) == 0);
    CHECK(wcDeciDeg(-123.4f) == -1234);
    CHECK(wcDeciDeg(5000.0f) == 32767);
    CHECK(wcU16Sat(-5.0f) == 0);
    CHECK(wcU16Sat(70000.0f) == 65535);
    CHECK(wcU16Sat(NAN) == 0);
    CHECK(wcU16Sat(42000.4f) == 42000);
}

static void testMotorTest(void)
{
    puts("motor test");
    uint8_t buf[WC_MOTOR_TEST_LEN];
    wcMotorTest_t m = {.mask = 0x05, .percent = 20, .durationMs = 2000}, out;
    CHECK(wcEncodeMotorTest(&m, buf, sizeof(buf)) == WC_MOTOR_TEST_LEN);
    CHECK(wcDecodeMotorTest(buf, sizeof(buf), &out));
    CHECK(out.mask == 0x05 && out.percent == 20 && out.durationMs == 2000);
    CHECK(!wcDecodeMotorTest(buf, sizeof(buf) - 1, &out));

    const wcMotorTest_t bad[] = {
        {.mask = 0x10, .percent = 20, .durationMs = 1000},   // no motor 5
        {.mask = 0x01, .percent = 101, .durationMs = 1000},
        {.mask = 0x01, .percent = 0, .durationMs = 1000},    // start without power
        {.mask = 0x01, .percent = 10, .durationMs = 0},      // start without time
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        wcEncodeMotorTest(&bad[i], buf, sizeof(buf));
        CHECK(!wcDecodeMotorTest(buf, sizeof(buf), &out));
    }
    const wcMotorTest_t stop = {0};
    wcEncodeMotorTest(&stop, buf, sizeof(buf));
    CHECK(wcDecodeMotorTest(buf, sizeof(buf), &out) && out.mask == 0);

    wcMotorTestInputs_t in = {.enabled = true, .systemReady = true, .busy = false};
    wcMotorTest_t applied = {0};
    const wcMotorTest_t req = {.mask = 0x0F, .percent = 80, .durationMs = 10000};
    CHECK(wcMotorTestCheck(&in, &req, 30, &applied) == WC_MOTOR_OK);
    CHECK(applied.mask == 0x0F && applied.percent == 30 && applied.durationMs == WC_MOTOR_TEST_MAX_MS);
    const wcMotorTest_t small = {.mask = 0x02, .percent = 10, .durationMs = 1500};
    CHECK(wcMotorTestCheck(&in, &small, 30, &applied) == WC_MOTOR_OK);
    CHECK(applied.mask == 0x02 && applied.percent == 10 && applied.durationMs == 1500);
    CHECK(wcMotorTestCheck(&in, &small, 0, &applied) == WC_MOTOR_DISABLED);
    in.busy = true;
    in.systemReady = false;
    CHECK(wcMotorTestCheck(&in, &small, 30, &applied) == WC_MOTOR_BUSY);
    in.busy = false;
    CHECK(wcMotorTestCheck(&in, &small, 30, &applied) == WC_MOTOR_NOT_READY);
    in.enabled = false;
    CHECK(wcMotorTestCheck(&in, &small, 30, &applied) == WC_MOTOR_DISABLED);

    CHECK(wcMotorRatio(0) == 0);
    CHECK(wcMotorRatio(100) == 65535);
    CHECK(wcMotorRatio(50) == 32767);
    CHECK(wcMotorRatio(200) == 65535);

    const wcMotorReply_t rep = {.mask = 0x08, .percent = 15, .status = WC_MOTOR_BUSY, .durationMs = 2500};
    wcMotorReply_t rback;
    uint8_t rb[WC_MOTOR_REPLY_LEN];
    CHECK(wcEncodeMotorReply(&rep, rb, sizeof(rb)) == WC_MOTOR_REPLY_LEN);
    CHECK(wcDecodeMotorReply(rb, sizeof(rb), &rback));
    CHECK(rback.mask == 0x08 && rback.percent == 15 && rback.status == WC_MOTOR_BUSY && rback.durationMs == 2500);
}

static const wcFlipConfig_t flipCfg = {
    .rateDps = 800, .boostMs = 250, .boostPct = 170, .spinPct = 50,
    .exitDeg = 70, .recoverMs = 350, .recoverPct = 140, .timeoutMs = 900,
};

/* Euler roll and pitch of a drone turned by angleDeg about one axis (as the
 * estimator reports them: pitch stays within +-90, roll jumps to 180). */
static void flipAttitude(uint8_t dir, float angleDeg, float *roll, float *pitch)
{
    float a = fmodf(angleDeg, 360.0f);
    if (dir == WC_FLIP_LEFT || dir == WC_FLIP_RIGHT) {
        a = dir == WC_FLIP_RIGHT ? a : -a;
        *roll = a > 180.0f ? a - 360.0f : (a < -180.0f ? a + 360.0f : a);
        *pitch = 0.0f;
        return;
    }
    const float phi = dir == WC_FLIP_BACK ? a : -a;     /* + = nose up */
    if (fabsf(phi) <= 90.0f || fabsf(phi) >= 270.0f) {
        *roll = 0.0f;
        *pitch = fabsf(phi) <= 90.0f ? phi : (phi > 0 ? phi - 360.0f : phi + 360.0f);
    } else {
        *roll = 180.0f;
        *pitch = phi > 0 ? 180.0f - phi : -180.0f - phi;
    }
}

/* Runs a flip with the drone turning at the commanded rate; returns the
 * duration of SPIN in ms (or -1 if it never left SPIN) and the angle turned. */
static int runFlip(uint8_t dir, const wcFlipConfig_t *c, float hover, bool turns, float *turned)
{
    wcFlip_t f;
    wcFlipOut_t out;
    uint32_t t = 5000;
    float angle = 0.0f;
    int spinStart = -1, spinEnd = -1;
    wcFlipStart(&f, dir, t, hover);
    for (int i = 0; i < 400; i++, t += 10) {
        float roll, pitch;
        flipAttitude(dir, angle, &roll, &pitch);
        if (!wcFlipUpdate(&f, c, t, roll, pitch, &out)) {
            break;
        }
        if (out.spin && spinStart < 0) {
            spinStart = (int)t;
        }
        if (!out.spin && spinStart >= 0 && spinEnd < 0) {
            spinEnd = (int)t;
        }
        if (out.spin && turns) {
            const float rate = fabsf(out.rollRateDps) + fabsf(out.pitchRateDps);
            angle += rate * 0.01f;
        }
    }
    *turned = angle;
    return spinStart >= 0 && spinEnd >= 0 ? spinEnd - spinStart : -1;
}

static void testFlip(void)
{
    puts("flip");
    uint8_t buf[8], dir = 0xFF, status = 0xFF;
    for (uint8_t d = 0; d <= WC_FLIP_RIGHT; d++) {
        CHECK(wcEncodeFlip(d, buf, sizeof(buf)) == WC_FLIP_LEN);
        CHECK(wcDecodeFlip(buf, WC_FLIP_LEN, &dir) && dir == d);
    }
    buf[1] = WC_FLIP_RIGHT + 1;
    CHECK(!wcDecodeFlip(buf, WC_FLIP_LEN, &dir));
    CHECK(!wcDecodeFlip(buf, WC_FLIP_LEN + 1, &dir));
    CHECK(wcEncodeFlip(0, buf, 3) == 0);
    CHECK(wcEncodeFlipReply(WC_FLIP_LEFT, WC_FLIP_BUSY, buf, sizeof(buf)) == WC_FLIP_REPLY_LEN);
    CHECK(wcDecodeFlipReply(buf, WC_FLIP_REPLY_LEN, &dir, &status) && dir == WC_FLIP_LEFT && status == WC_FLIP_BUSY);
    CHECK(!wcDecodeFlipReply(buf, WC_FLIP_REPLY_LEN - 1, &dir, &status));

    // start conditions, in this order
    wcFlipInputs_t in = {.enabled = true, .pilot = true, .hoverKnown = true, .thrustUp = true, .busy = false};
    CHECK(wcFlipCheck(&in) == WC_FLIP_OK);
    in.thrustUp = false;
    CHECK(wcFlipCheck(&in) == WC_FLIP_NOT_FLYING);
    in.thrustUp = true;
    in.hoverKnown = false;
    CHECK(wcFlipCheck(&in) == WC_FLIP_NOT_FLYING);
    in.busy = true;
    CHECK(wcFlipCheck(&in) == WC_FLIP_BUSY);
    in.pilot = false;
    CHECK(wcFlipCheck(&in) == WC_FLIP_NOT_PILOT);
    in.enabled = false;
    CHECK(wcFlipCheck(&in) == WC_FLIP_DISABLED);

    // phases of a right flip: boost, spin at +rate, recover, done
    wcFlip_t f;
    wcFlipOut_t out;
    wcFlipStart(&f, WC_FLIP_RIGHT, 1000, 0.5f);
    CHECK(wcFlipUpdate(&f, &flipCfg, 1000, 0, 0, &out) && !out.spin);
    CHECK_NEAR(out.thrust, 0.85f, 1e-4f);                     // 170 % of hover
    CHECK(wcFlipUpdate(&f, &flipCfg, 1249, 0, 0, &out) && !out.spin && f.phase == WC_FLIP_BOOST);
    CHECK(wcFlipUpdate(&f, &flipCfg, 1250, 0, 0, &out) && out.spin && f.phase == WC_FLIP_SPIN);
    CHECK(out.rollRateDps == 800.0f && out.pitchRateDps == 0.0f);
    CHECK_NEAR(out.thrust, 0.25f, 1e-4f);                     // 50 % of hover
    // upright again but never upside down: keeps turning
    CHECK(wcFlipUpdate(&f, &flipCfg, 1300, 10, 0, &out) && out.spin);
    CHECK(wcFlipUpdate(&f, &flipCfg, 1400, 170, 0, &out) && out.spin && f.inverted);
    CHECK(wcFlipUpdate(&f, &flipCfg, 1450, -100, 0, &out) && out.spin);
    CHECK(wcFlipUpdate(&f, &flipCfg, 1460, -71, 0, &out) && out.spin);      // 71 deg from upright
    CHECK(wcFlipUpdate(&f, &flipCfg, 1470, -65, 0, &out) && !out.spin);     // within 70: recover
    CHECK(f.phase == WC_FLIP_RECOVER && !f.timedOut);
    CHECK_NEAR(out.thrust, 0.7f, 1e-4f);                      // 140 % of hover
    CHECK(wcFlipUpdate(&f, &flipCfg, 1819, 0, 0, &out) && !out.spin);
    CHECK(!wcFlipUpdate(&f, &flipCfg, 1820, 0, 0, &out) && f.phase == WC_FLIP_IDLE);
    CHECK(!wcFlipUpdate(&f, &flipCfg, 1830, 0, 0, &out));

    // directions and signs
    const struct { uint8_t dir; float roll, pitch; } signs[] = {
        {WC_FLIP_FORWARD, 0, -800}, {WC_FLIP_BACK, 0, 800}, {WC_FLIP_LEFT, -800, 0}, {WC_FLIP_RIGHT, 800, 0},
    };
    for (size_t i = 0; i < 4; i++) {
        wcFlipStart(&f, signs[i].dir, 0, 0.5f);
        CHECK(wcFlipUpdate(&f, &flipCfg, 300, 0, 0, &out) && out.spin);
        CHECK(out.rollRateDps == signs[i].roll && out.pitchRateDps == signs[i].pitch);
    }

    // a full turn in every direction, including the Euler jump of pitch flips:
    // the spin ends 70 deg before upright, so after 290 deg
    for (uint8_t d = 0; d <= WC_FLIP_RIGHT; d++) {
        float turned = 0.0f;
        const int spinMs = runFlip(d, &flipCfg, 0.5f, true, &turned);
        CHECK(spinMs > 0 && spinMs < (int)flipCfg.timeoutMs);
        CHECK(turned > 285.0f && turned < 305.0f);
    }

    // a drone that does not turn: the timeout ends the spin
    float turned = 1.0f;
    const int spinMs = runFlip(WC_FLIP_LEFT, &flipCfg, 0.5f, false, &turned);
    CHECK(spinMs == (int)flipCfg.timeoutMs && turned == 0.0f);

    // thrust limits: at least WC_FLIP_MIN_THRUST while turning, at most 1
    wcFlipConfig_t c = flipCfg;
    c.spinPct = 10;
    c.boostPct = 250;
    c.boostMs = 0;
    wcFlipStart(&f, WC_FLIP_BACK, 0, 0.5f);
    CHECK(wcFlipUpdate(&f, &c, 0, 0, 0, &out) && out.spin);   // no boost: spin at once
    CHECK_NEAR(out.thrust, WC_FLIP_MIN_THRUST, 1e-6f);
    c.boostMs = 100;
    wcFlipStart(&f, WC_FLIP_BACK, 0, 0.5f);
    CHECK(wcFlipUpdate(&f, &c, 0, 0, 0, &out) && out.thrust == 1.0f);

    // changeable parameters
    for (int i = 0; i < WC_GAIN_COUNT; i++) {
        CHECK(strcmp(wcTunables[i].name, wcGainNames[i]) == 0);
    }
    for (int i = 0; i < WC_TUNABLE_COUNT; i++) {
        CHECK(wcParamNameValid(wcTunables[i].name));
        CHECK(wcTunableIndex(wcTunables[i].name) == i);
        CHECK(wcTunableValid(i, wcTunables[i].min) && wcTunableValid(i, wcTunables[i].max));
        CHECK(!wcTunableValid(i, wcTunables[i].max + 1.0f) && !wcTunableValid(i, NAN));
    }
    CHECK(wcTunableIndex("flip.rate") == WC_GAIN_COUNT);
    CHECK(wcTunableIndex("flip.timeoutMs") == WC_TUNABLE_COUNT - 1);
    CHECK(wcTunableIndex("motorPowerSet.m1") == -1);
    CHECK(!wcTunableValid(-1, 1.0f) && !wcTunableValid(WC_TUNABLE_COUNT, 1.0f));
    // the defaults above are valid
    const uint16_t *v = &flipCfg.rateDps;
    for (int i = 0; i < WC_FLIP_PARAM_COUNT; i++) {
        CHECK(wcTunableValid(WC_GAIN_COUNT + i, (float)v[i]));
    }

    float vals[WC_FLIP_PARAM_COUNT], back[WC_FLIP_PARAM_COUNT];
    bool valid[WC_FLIP_PARAM_COUNT];
    for (int i = 0; i < WC_FLIP_PARAM_COUNT; i++) {
        vals[i] = (float)v[i];
    }
    vals[3] = 500.0f;                                          // spin: out of range
    uint8_t blob[WC_FLIP_BLOB_LEN];
    CHECK(wcEncodeFlipBlob(vals, blob, sizeof(blob)) == WC_FLIP_BLOB_LEN);
    CHECK(wcDecodeFlipBlob(blob, sizeof(blob), back, valid));
    CHECK(valid[0] && back[0] == 800.0f && !valid[3] && valid[7]);
    CHECK(!wcDecodeFlipBlob(blob, sizeof(blob) - 1, back, valid));
    blob[1] = WC_FLIP_PARAM_COUNT + 1;
    CHECK(!wcDecodeFlipBlob(blob, sizeof(blob), back, valid));
}

int main(void)
{
    testControlRoundTrip();
    testHelloAndTelemetry();
    testSeq();
    testStages();
    testFailsafeOutput();
    testHover();
    testDelayEstimator();
    testLinkBurstAfterStall();
    testLinkDisconnectAndPause();
    testLinkResume();
    testArmCheck();
    testControlToSetpoint();
    testTrim();
    testBattery();
    testParamCodec();
    testGains();
    testStreamAndFast();
    testMotorTest();
    testFlip();

    printf("\n%d checks, %d failed\n", checks, failures);
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
