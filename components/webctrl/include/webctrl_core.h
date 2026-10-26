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
 * webctrl_core.h - platform independent part of the browser control:
 * wire protocol, link supervision (failsafe stages), delay and hover
 * estimation, arm checks. No ESP-IDF includes, so all of it is unit tested
 * on the host (components/webctrl/host_test).
 *
 * The protocol is documented in docs/webcontrol.md.
 */

#ifndef WEBCTRL_CORE_H_
#define WEBCTRL_CORE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------------ */
/* Wire protocol: binary WebSocket frames, little endian                     */
/* ------------------------------------------------------------------------ */

/* 2: TRIM message; telemetry 40 bytes with state2, stop reason and trim */
#define WC_PROTOCOL_VERSION   2

#define WC_MSG_CONTROL        0x01  /* client -> drone */
#define WC_MSG_HELLO          0x02  /* client -> drone, first frame after connect */
#define WC_MSG_TRIM           0x03  /* client -> drone, when the pilot trims */
#define WC_MSG_TELEMETRY      0x81  /* drone -> client */

#define WC_CONTROL_LEN        16
#define WC_HELLO_LEN          8
#define WC_TRIM_LEN           4
#define WC_TELEMETRY_LEN      40

/* CONTROL flags */
#define WC_FLAG_ARM           0x01  /* pilot wants the motors armed (level, not edge) */
#define WC_FLAG_KILL          0x02  /* emergency stop: motors off immediately */
#define WC_FLAG_PAUSE         0x04  /* page hidden: start the failsafe right away */
#define WC_FLAGS_KNOWN        (WC_FLAG_ARM | WC_FLAG_KILL | WC_FLAG_PAUSE)

#define WC_AXIS_MAX           1000  /* roll / pitch / yaw: -1000..1000 */
#define WC_THRUST_MAX         1000  /* thrust: 0..1000 */
#define WC_ARM_THRUST_MAX     20    /* throttle has to be (almost) down to arm */

typedef struct {
    uint8_t  flags;
    uint16_t seq;       /* increments with every frame, wraps */
    uint32_t clientMs;  /* sender clock (performance.now()), for delay estimation */
    int16_t  roll;      /* + = roll right */
    int16_t  pitch;     /* + = forward (nose down) */
    int16_t  yaw;       /* + = turn right (clockwise seen from above) */
    uint16_t thrust;    /* 0..1000 */
} wcControl_t;

typedef struct {
    uint8_t  version;
    uint32_t sessionToken;  /* 0 = new session, else the token from telemetry */
} wcHello_t;

/* Roll / pitch trim, added to the stick angles and used as "level" by the
 * failsafe. The drone stores it; every page shows it from the telemetry. */
#define WC_TRIM_MAX           50    /* in 1/10 deg: +-5.0 deg */

typedef struct {
    int8_t roll;    /* 1/10 deg, + = right */
    int8_t pitch;   /* 1/10 deg, + = forward */
} wcTrim_t;

/* TELEMETRY state bits */
#define WC_STATE_ARMED        0x01  /* motors armed by the web pilot */
#define WC_STATE_YOU_PILOT    0x02  /* this connection is the pilot */
#define WC_STATE_OTHER_PILOT  0x04  /* someone else (app, cfclient, other browser) flies */
#define WC_STATE_FAILSAFE     0x08  /* link failsafe active */
#define WC_STATE_LOW_BATTERY  0x10
#define WC_STATE_ARM_BLOCKED  0x20  /* release ARM before arming again */
#define WC_STATE_READY        0x40  /* self test passed, sensors calibrated */
#define WC_STATE_TUMBLED      0x80  /* upside down detected */

/* TELEMETRY state2 bits */
#define WC_STATE2_BAT_LANDING 0x01  /* battery empty in flight: forced landing */
#define WC_STATE2_BAT_NO_ARM  0x02  /* battery too low to arm (only sent while nobody flies) */

/* Why the last web pilot session ended */
typedef enum {
    WC_STOP_NONE = 0,
    WC_STOP_RELEASED,   /* the pilot disarmed */
    WC_STOP_KILL,       /* emergency stop from a browser */
    WC_STOP_LINK,       /* link failsafe ran to the end */
    WC_STOP_TUMBLED,
    WC_STOP_BATTERY,    /* battery empty */
    WC_STOP_ARM_LOST,   /* arm taken away elsewhere (system.forceArm, KILL by CRTP) */
} wcStopReason_t;

typedef enum {
    WC_ARM_OK = 0,
    WC_ARM_NOT_READY,       /* self test failed or sensors still calibrating */
    WC_ARM_THRUST_NOT_ZERO, /* throttle not down */
    WC_ARM_LOW_BATTERY,
    WC_ARM_OTHER_PILOT,     /* another source holds the arm */
    WC_ARM_TUMBLED,
    WC_ARM_RELEASE_FIRST,   /* ARM flag has to go off once (after kill/failsafe) */
} wcArmResult_t;

typedef struct {
    uint8_t  state;         /* WC_STATE_* */
    uint8_t  stage;         /* wcStage_t of the pilot link */
    uint8_t  armResult;     /* wcArmResult_t of the last arm attempt */
    uint16_t echoSeq;       /* last accepted seq of this connection */
    uint16_t rxRate;        /* accepted control frames per second */
    uint32_t echoClientMs;  /* clientMs of that frame: round trip time on the client */
    uint16_t vbatMv;
    int16_t  rollCdeg;      /* attitude in 1/100 deg */
    int16_t  pitchCdeg;
    int16_t  yawCdeg;
    uint16_t thrustOut;     /* thrust sent to the controller, 0..1000 */
    uint16_t staleDrops;    /* frames dropped because they arrived too late (wraps) */
    uint16_t delayMs;       /* extra delay of the last frame vs. the best seen */
    uint8_t  state2;        /* WC_STATE2_* */
    uint8_t  stopReason;    /* wcStopReason_t of the last session end */
    uint32_t uptimeMs;
    uint32_t sessionToken;  /* token of this connection's pilot session, 0 if none */
    wcTrim_t trim;          /* current trim of the drone */
} wcTelemetry_t;

/* Decoders validate type, length and value ranges; they return false for
 * anything malformed (the frame is then ignored). */
bool wcDecodeControl(const uint8_t *buf, size_t len, wcControl_t *out);
bool wcDecodeHello(const uint8_t *buf, size_t len, wcHello_t *out);
bool wcDecodeTrim(const uint8_t *buf, size_t len, wcTrim_t *out);
size_t wcEncodeTelemetry(const wcTelemetry_t *t, uint8_t *buf, size_t cap);

/* Encoders used by tests and tools */
size_t wcEncodeControl(const wcControl_t *c, uint8_t *buf, size_t cap);
size_t wcEncodeTrim(const wcTrim_t *t, uint8_t *buf, size_t cap);
bool wcDecodeTelemetry(const uint8_t *buf, size_t len, wcTelemetry_t *out);

/* true if sequence number a is newer than b (wrap safe) */
static inline bool wcSeqNewer(uint16_t a, uint16_t b)
{
    return (int16_t)(uint16_t)(a - b) > 0;
}

/* ------------------------------------------------------------------------ */
/* Link supervision                                                          */
/* ------------------------------------------------------------------------ */

typedef enum {
    WC_STAGE_OK = 0,   /* fresh commands */
    WC_STAGE_HOLD,     /* short gap: the last command is held */
    WC_STAGE_LEVEL,    /* level out, thrust limited to the hover estimate */
    WC_STAGE_DESCEND,  /* thrust below hover: sink */
    WC_STAGE_OFF,      /* motors off, disarmed */
} wcStage_t;

typedef struct {
    uint32_t okMs;          /* age up to which a command counts as fresh */
    uint32_t holdMs;        /* hold the last command up to this age */
    uint32_t descendMs;     /* level out up to this age, then descend */
    uint32_t offMs;         /* motors off at this age */
    uint32_t staleMs;       /* drop commands delayed more than this */
    float    descendFactor; /* descent thrust = hover thrust * factor */
    float    maxAngleDeg;   /* full stick roll / pitch */
    float    maxYawRateDeg; /* full stick yaw rate */
    uint16_t batLandMv;     /* forced landing below this voltage in flight, 0 = off */
    uint16_t batArmMinMv;   /* no arming below this voltage, 0 = off */
    uint32_t batLandDelayMs; /* the voltage has to stay low this long */
    uint32_t batLandTimeMs; /* forced landing: thrust ramps down to zero in this time */
} wcConfig_t;

/* Pilot command in physical units */
typedef struct {
    float roll;     /* deg, + right */
    float pitch;    /* deg, + forward */
    float yawRate;  /* deg/s, + right */
    float thrust;   /* 0..1 */
} wcSetpoint_t;

wcStage_t wcStageForAge(const wcConfig_t *cfg, uint32_t ageMs);

/* Stick values (protocol units) plus trim to physical units. Roll and pitch
 * stay within +-maxAngleDeg. trim may be NULL. */
void wcControlToSetpoint(const wcConfig_t *cfg, const wcControl_t *c, const wcTrim_t *trim,
                         wcSetpoint_t *out);

/* ------------------------------------------------------------------------ */
/* Hover thrust estimate: low pass of the thrust while flying                */
/* ------------------------------------------------------------------------ */

#define WC_HOVER_TAU_S         3.0f   /* filter time constant */
#define WC_HOVER_MIN_FLYING_S  2.0f   /* estimate valid after this flight time */
#define WC_HOVER_FLYING_THRUST 0.15f  /* thrust above this counts as flying */

typedef struct {
    float value;    /* estimated hover thrust, 0..1 */
    float flyingS;  /* seconds of flight in the estimate */
} wcHover_t;

void wcHoverReset(wcHover_t *h);
/* Call periodically with the pilot's thrust while armed and in WC_STAGE_OK. */
void wcHoverUpdate(wcHover_t *h, float thrust, float dtS);
bool wcHoverValid(const wcHover_t *h);

/* Setpoint for a stage, based on the pilot's last fresh command. "Level" means
 * the trim angles (trim may be NULL). */
void wcFailsafeOutput(const wcConfig_t *cfg, wcStage_t stage, const wcSetpoint_t *last,
                      const wcHover_t *hover, const wcTrim_t *trim, wcSetpoint_t *out);

/* ------------------------------------------------------------------------ */
/* Battery protection                                                        */
/* ------------------------------------------------------------------------ */
/*
 * Forced landing: in flight, a battery voltage below batLandMv for
 * batLandDelayMs starts it. The pilot keeps steering, but the thrust is capped:
 * the cap starts at the hover thrust and ramps down to zero within
 * batLandTimeMs, then the drone disarms. Armed on the ground with an empty
 * battery, the drone disarms at once. Readings below WC_BAT_ABSENT_V mean "no
 * battery" (USB power on the bench): no checks.
 */
#define WC_BAT_FILTER_TAU_S   0.5f  /* low pass of the voltage readings */
#define WC_BAT_ABSENT_V       0.5f

typedef enum {
    WC_BAT_NONE = 0,
    WC_BAT_LAND,        /* start the forced landing now */
    WC_BAT_EMPTY,       /* armed but not flying with an empty battery: disarm */
} wcBatEvent_t;

typedef struct {
    float    volts;         /* filtered voltage, 0 = no reading */
    bool     lowTiming;     /* below batLandMv since lowSinceMs */
    uint32_t lowSinceMs;
    bool     landing;       /* forced landing active */
    uint32_t landStartMs;
    float    landFrom;      /* thrust cap at the start of the landing */
} wcBattery_t;

void wcBatteryReset(wcBattery_t *b);
/* A new pilot session: no landing, no timer. The filtered voltage stays. */
void wcBatteryNewSession(wcBattery_t *b);
/*
 * Call every cycle. volts: latest reading. session: a web pilot holds the arm.
 * flying: the thrust is above WC_HOVER_FLYING_THRUST. landFrom: thrust the
 * landing ramp starts from (hover estimate, or the current thrust if lower).
 */
wcBatEvent_t wcBatteryUpdate(wcBattery_t *b, const wcConfig_t *cfg, uint32_t nowMs, float dtS,
                             float volts, bool session, bool flying, float landFrom);
/* Upper limit for the thrust: 1 normally, the landing ramp while landing */
float wcBatteryThrustCap(const wcBattery_t *b, const wcConfig_t *cfg, uint32_t nowMs);
bool wcBatteryArmBlocked(const wcBattery_t *b, const wcConfig_t *cfg);

/* ------------------------------------------------------------------------ */
/* One-way delay estimation without synchronized clocks                      */
/* ------------------------------------------------------------------------ */
/*
 * offset = arrival time (drone clock) - send time (client clock)
 *        = one-way delay + unknown clock offset.
 * The smallest offset seen in the last few seconds stands for the fastest
 * path. offset - minimum is the extra delay of a frame (queueing, TCP
 * retransmission, a stalled browser). Clock drift is covered because old
 * minima expire.
 */
#define WC_DELAY_BUCKETS   8
#define WC_DELAY_BUCKET_MS 1000

typedef struct {
    int32_t  bucketMin[WC_DELAY_BUCKETS];
    bool     bucketUsed[WC_DELAY_BUCKETS];
    uint32_t bucketStartMs;
    uint8_t  current;
    bool     started;
} wcDelayEst_t;

void wcDelayReset(wcDelayEst_t *d);
/* Returns the extra delay in ms of a frame sent at clientMs and received at droneMs. */
uint32_t wcDelayUpdate(wcDelayEst_t *d, uint32_t droneMs, uint32_t clientMs);

/* ------------------------------------------------------------------------ */
/* Pilot link: freshness of the commands of one connection                   */
/* ------------------------------------------------------------------------ */

typedef enum {
    WC_FRAME_ACCEPTED = 0,  /* fresh: becomes the active command */
    WC_FRAME_STALE,         /* arrived too late (TCP stall, burst): dropped */
    WC_FRAME_OLD_SEQ,       /* not newer than the last accepted frame: dropped */
} wcFrameResult_t;

typedef struct {
    wcDelayEst_t delay;
    bool        haveFrame;     /* at least one accepted frame in this session */
    bool        seqValid;      /* lastSeq belongs to the current connection */
    bool        connected;     /* WebSocket open */
    bool        paused;        /* client reported a hidden page */
    uint16_t    lastSeq;
    uint32_t    lastFreshMs;   /* drone time of the last accepted frame */
    uint32_t    lastDelayMs;   /* extra delay of the last frame */
    uint16_t    staleDrops;
    wcControl_t last;          /* last accepted frame */
} wcLink_t;

/* New pilot session: forget everything. */
void wcLinkReset(wcLink_t *l);

/*
 * The pilot came back on a new connection: sequence numbers and the client
 * clock may have restarted, so forget them. The last command and its time
 * stay, the failsafe timeline continues until fresh frames arrive.
 */
void wcLinkResume(wcLink_t *l);

/* Feed a decoded CONTROL frame received at droneMs. */
wcFrameResult_t wcLinkOnControl(wcLink_t *l, const wcConfig_t *cfg, uint32_t droneMs,
                                const wcControl_t *c);

/*
 * Age of the active command as seen by the failsafe. A closed connection or a
 * hidden page counts as at least holdMs old, so the drone levels out at once
 * instead of holding the last stick input. No frame at all counts as offMs.
 */
uint32_t wcLinkAge(const wcLink_t *l, const wcConfig_t *cfg, uint32_t nowMs);

/* ------------------------------------------------------------------------ */
/* Arm check                                                                 */
/* ------------------------------------------------------------------------ */

typedef struct {
    bool systemReady;        /* self test passed and sensors calibrated */
    bool lowBattery;
    bool tumbled;
    bool otherPilot;         /* another source holds the arm */
    bool releasedSinceStop;  /* ARM flag was off since the last disarm/kill */
} wcArmInputs_t;

wcArmResult_t wcArmCheck(const wcArmInputs_t *in, const wcControl_t *cmd);

#endif /* WEBCTRL_CORE_H_ */
