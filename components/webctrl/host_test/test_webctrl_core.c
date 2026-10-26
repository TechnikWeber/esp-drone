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
    const uint8_t hello[WC_HELLO_LEN] = {WC_MSG_HELLO, 1, 0, 0, 0x78, 0x56, 0x34, 0x12};
    wcHello_t h;
    CHECK(wcDecodeHello(hello, sizeof(hello), &h));
    CHECK(h.version == 1 && h.sessionToken == 0x12345678u);
    CHECK(!wcDecodeHello(hello, sizeof(hello) - 1, &h));

    wcTelemetry_t t = {.state = WC_STATE_ARMED | WC_STATE_YOU_PILOT, .stage = WC_STAGE_LEVEL,
                       .armResult = WC_ARM_OK, .echoSeq = 4711, .rxRate = 50,
                       .echoClientMs = 123456789, .vbatMv = 3912, .rollCdeg = -1234,
                       .pitchCdeg = 567, .yawCdeg = -17999, .thrustOut = 512,
                       .staleDrops = 3, .delayMs = 42, .uptimeMs = 987654, .sessionToken = 0xCAFEF00D};
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
    CHECK(buf[26] == 0 && buf[27] == 0);   // reserved bytes are zero
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

    wcFailsafeOutput(&cfg, WC_STAGE_HOLD, &climbing, &hover, &out);
    CHECK(memcmp(&out, &climbing, sizeof(out)) == 0);

    // level: attitude zero, climbing pilot is limited to hover thrust
    wcFailsafeOutput(&cfg, WC_STAGE_LEVEL, &climbing, &hover, &out);
    CHECK(out.roll == 0 && out.pitch == 0 && out.yawRate == 0);
    CHECK_NEAR(out.thrust, 0.55f, 0.001f);
    // ... a sinking pilot keeps sinking
    wcFailsafeOutput(&cfg, WC_STAGE_LEVEL, &sinking, &hover, &out);
    CHECK_NEAR(out.thrust, 0.4f, 0.001f);

    // descend: below hover
    wcFailsafeOutput(&cfg, WC_STAGE_DESCEND, &climbing, &hover, &out);
    CHECK_NEAR(out.thrust, 0.55f * 0.85f, 0.001f);
    CHECK(out.thrust < hover.value);
    wcFailsafeOutput(&cfg, WC_STAGE_DESCEND, &sinking, &hover, &out);
    CHECK_NEAR(out.thrust, 0.4f, 0.001f);

    // off
    wcFailsafeOutput(&cfg, WC_STAGE_OFF, &climbing, &hover, &out);
    CHECK(out.thrust == 0 && out.roll == 0 && out.yawRate == 0);

    // without a valid hover estimate the last thrust is the reference
    wcHover_t fresh;
    wcHoverReset(&fresh);
    wcFailsafeOutput(&cfg, WC_STAGE_LEVEL, &climbing, &fresh, &out);
    CHECK_NEAR(out.thrust, 0.8f, 0.001f);
    wcFailsafeOutput(&cfg, WC_STAGE_DESCEND, &climbing, &fresh, &out);
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
    wcControlToSetpoint(&cfg, &c, &sp);
    CHECK_NEAR(sp.roll, 15.0f, 0.001f);
    CHECK_NEAR(sp.pitch, -7.5f, 0.001f);
    CHECK_NEAR(sp.yawRate, 30.0f, 0.001f);
    CHECK_NEAR(sp.thrust, 1.0f, 0.001f);
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

    printf("\n%d checks, %d failed\n", checks, failures);
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
