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

/*
 * 2: TRIM message; telemetry 40 bytes with state2, stop reason and trim
 * 3: PARAM, STREAM and MOTOR_TEST for the ground station, FAST samples
 *    (all optional: a version 2 page works unchanged)
 * 4: FLIP (experimental), flip parameters writable with PARAM
 */
#define WC_PROTOCOL_VERSION   4

#define WC_MSG_CONTROL        0x01  /* client -> drone */
#define WC_MSG_HELLO          0x02  /* client -> drone, first frame after connect */
#define WC_MSG_TRIM           0x03  /* client -> drone, when the pilot trims */
#define WC_MSG_PARAM          0x04  /* client -> drone: read, write, save or reset a gain */
#define WC_MSG_STREAM         0x05  /* client -> drone: FAST samples on or off */
#define WC_MSG_MOTOR_TEST     0x06  /* client -> drone: spin motors on the bench */
#define WC_MSG_FLIP           0x07  /* client -> drone: flip in flight (experimental) */
#define WC_MSG_TELEMETRY      0x81  /* drone -> client */
#define WC_MSG_FAST           0x82  /* drone -> client that asked for it, 20 frames/s */
#define WC_MSG_PARAM_REPLY    0x84  /* drone -> client, answer to PARAM */
#define WC_MSG_MOTOR_REPLY    0x86  /* drone -> client, answer to MOTOR_TEST */
#define WC_MSG_FLIP_REPLY     0x87  /* drone -> client, answer to FLIP */

#define WC_CONTROL_LEN        16
#define WC_HELLO_LEN          8
#define WC_TRIM_LEN           4
#define WC_TELEMETRY_LEN      40
#define WC_STREAM_LEN         4
#define WC_MOTOR_TEST_LEN     8
#define WC_MOTOR_REPLY_LEN    8
#define WC_FLIP_LEN           4
#define WC_FLIP_REPLY_LEN     4

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
#define WC_STATE2_MOTOR_TEST  0x04  /* a motor test is running */
#define WC_STATE2_FLIP        0x08  /* a flip is running */

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
    WC_ARM_MOTOR_TEST,      /* a motor test is running */
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

/* ------------------------------------------------------------------------ */
/* Ground station messages (protocol 3)                                      */
/* ------------------------------------------------------------------------ */

/*
 * PARAM: read any parameter of the firmware by name ("group.name"); write,
 * save and reset only the controller gains (wcGainNames). Every request gets
 * a PARAM_REPLY on the same connection.
 */
#define WC_PARAM_HDR_LEN        8
#define WC_PARAM_REPLY_HDR_LEN  14
#define WC_PARAM_NAME_MAX       31
#define WC_PARAM_MAX_LEN        (WC_PARAM_REPLY_HDR_LEN + WC_PARAM_NAME_MAX)

typedef enum {
    WC_PARAM_READ = 0,
    WC_PARAM_WRITE,     /* a gain, takes effect at once (RAM only) */
    WC_PARAM_SAVE,      /* store all gains on the drone; they are loaded at boot */
    WC_PARAM_RESET,     /* all gains back to the firmware defaults, stored ones erased */
} wcParamOp_t;

typedef enum {
    WC_PARAM_OK = 0,
    WC_PARAM_UNKNOWN,   /* no parameter of that name */
    WC_PARAM_DENIED,    /* not a gain: read only over this channel */
    WC_PARAM_BUSY,      /* someone else flies, or saving in flight, or still starting */
    WC_PARAM_INVALID,   /* value out of range */
    WC_PARAM_FAILED,    /* flash error */
} wcParamStatus_t;

#define WC_PARAMF_WRITABLE      0x01  /* the parameter is a gain */
#define WC_PARAMF_STORED        0x02  /* gains stored on the drone are active */

typedef struct {
    uint8_t op;         /* wcParamOp_t */
    uint8_t reqId;      /* echoed in the reply */
    float   value;      /* WRITE */
    char    name[WC_PARAM_NAME_MAX + 1];   /* READ / WRITE, empty for SAVE / RESET */
} wcParamReq_t;

typedef struct {
    uint8_t op;
    uint8_t reqId;
    uint8_t status;     /* wcParamStatus_t */
    uint8_t flags;      /* WC_PARAMF_* */
    float   value;      /* current value */
    float   dflt;       /* firmware default of a gain, NaN otherwise */
    char    name[WC_PARAM_NAME_MAX + 1];
} wcParamReply_t;

/* Names: "group.name", letters, digits and '_', one dot, at most WC_PARAM_NAME_MAX */
bool wcParamNameValid(const char *name);
bool wcDecodeParamReq(const uint8_t *buf, size_t len, wcParamReq_t *out);
size_t wcEncodeParamReq(const wcParamReq_t *r, uint8_t *buf, size_t cap);
size_t wcEncodeParamReply(const wcParamReply_t *r, uint8_t *buf, size_t cap);
bool wcDecodeParamReply(const uint8_t *buf, size_t len, wcParamReply_t *out);

/* Controller gains that may be changed over the network: rate and attitude
 * PID of roll, pitch and yaw. */
#define WC_GAIN_COUNT           18
#define WC_GAIN_MAX             5000.0f
extern const char *const wcGainNames[WC_GAIN_COUNT];
int wcGainIndex(const char *name);      /* -1 if not a gain */
bool wcGainValueValid(float v);         /* finite, 0..WC_GAIN_MAX */

/* Stored gains (NVS blob): version, count, reserved (2), WC_GAIN_COUNT floats */
#define WC_GAINS_BLOB_VERSION   1
#define WC_GAINS_BLOB_LEN       (4 + 4 * WC_GAIN_COUNT)
size_t wcEncodeGainsBlob(const float *values, uint8_t *buf, size_t cap);
/* valid[i] tells which values passed the checks; false if the blob is unusable */
bool wcDecodeGainsBlob(const uint8_t *buf, size_t len, float *values, bool *valid);

/* STREAM: flags of what the client wants besides TELEMETRY */
#define WC_STREAM_FAST          0x01

bool wcDecodeStream(const uint8_t *buf, size_t len, uint8_t *flags);
size_t wcEncodeStream(uint8_t flags, uint8_t *buf, size_t cap);

/*
 * FAST: samples of the flight controller every WC_FAST_PERIOD_MS, sent in
 * frames of WC_FAST_SAMPLES samples. Angles as in the telemetry (roll + =
 * right side down, pitch + = nose up). Rates in the convention of the rate
 * controller: roll + = to the right, pitch + = nose up, yaw + = counter-
 * clockwise seen from above.
 */
#define WC_FAST_PERIOD_MS       10
#define WC_FAST_SAMPLES         5
#define WC_FAST_HDR_LEN         12
#define WC_FAST_SAMPLE_LEN      32
#define WC_FAST_MAX_LEN         (WC_FAST_HDR_LEN + WC_FAST_SAMPLES * WC_FAST_SAMPLE_LEN)

typedef struct {
    int16_t  rollCdeg, pitchCdeg, yawCdeg;     /* attitude estimate */
    int16_t  rollSpCdeg, pitchSpCdeg;          /* attitude setpoint of the controller */
    int16_t  gyroDdps[3];                      /* roll, pitch, yaw rate, 1/10 deg/s */
    int16_t  rateSpDdps[3];                    /* rate setpoints, 1/10 deg/s */
    uint16_t motor[4];                         /* M1..M4, 0..65535 */
    uint16_t thrust;                           /* thrust to the mixer, 0..65535 */
} wcFastSample_t;

typedef struct {
    uint16_t       seq;       /* number of the first sample (wraps) */
    uint32_t       t0Ms;      /* drone time of the first sample */
    uint8_t        periodMs;
    uint8_t        count;     /* 1..WC_FAST_SAMPLES */
    wcFastSample_t s[WC_FAST_SAMPLES];
} wcFast_t;

size_t wcEncodeFast(const wcFast_t *f, uint8_t *buf, size_t cap);
bool wcDecodeFast(const uint8_t *buf, size_t len, wcFast_t *out);
/* float to fixed point with saturation */
int16_t wcCentiDeg(float deg);
int16_t wcDeciDeg(float deg);
uint16_t wcU16Sat(float v);

/*
 * MOTOR_TEST: spin single motors on the bench, propellers off. Only while
 * nobody flies; the drone caps power and duration and stops the motors when
 * the time is up, the client leaves, or anyone sends KILL.
 */
#define WC_MOTOR_TEST_MAX_MS    3000

typedef struct {
    uint8_t  mask;        /* bit 0 = M1 ... bit 3 = M4; 0 = stop */
    uint8_t  percent;     /* power 1..100 */
    uint16_t durationMs;
} wcMotorTest_t;

typedef enum {
    WC_MOTOR_OK = 0,
    WC_MOTOR_NOT_READY,   /* self test or calibration not done */
    WC_MOTOR_BUSY,        /* armed or someone flies */
    WC_MOTOR_DISABLED,    /* switched off in the firmware configuration */
} wcMotorStatus_t;

typedef struct {
    uint8_t  mask;
    uint8_t  percent;     /* as applied */
    uint8_t  status;      /* wcMotorStatus_t */
    uint16_t durationMs;  /* as applied */
} wcMotorReply_t;

typedef struct {
    bool enabled;         /* compiled in (max percent > 0) and the motors can be set */
    bool systemReady;
    bool busy;            /* a pilot session or another source holds the arm */
} wcMotorTestInputs_t;

bool wcDecodeMotorTest(const uint8_t *buf, size_t len, wcMotorTest_t *out);
size_t wcEncodeMotorTest(const wcMotorTest_t *m, uint8_t *buf, size_t cap);
size_t wcEncodeMotorReply(const wcMotorReply_t *r, uint8_t *buf, size_t cap);
bool wcDecodeMotorReply(const uint8_t *buf, size_t len, wcMotorReply_t *out);
/* Checks a start request (mask != 0) and caps power and duration into applied. */
wcMotorStatus_t wcMotorTestCheck(const wcMotorTestInputs_t *in, const wcMotorTest_t *req,
                                 uint8_t maxPercent, wcMotorTest_t *applied);
/* Motor ratio (0..65535) for a power in percent */
uint16_t wcMotorRatio(uint8_t percent);

/* ------------------------------------------------------------------------ */
/* Flips (experimental, protocol 4)                                          */
/* ------------------------------------------------------------------------ */
/*
 * The pilot asks for a flip, the drone flies it by itself, so a stalled link
 * cannot cut it in half: climb with more thrust (BOOST), turn about the roll
 * or pitch axis at a fixed rate with little thrust (SPIN), and once it has
 * been upside down and is back within exitDeg of upright, level out with
 * extra thrust to catch the fall (RECOVER). Then the pilot's sticks count
 * again. Without an altitude sensor this is open loop: it needs height.
 *
 * During SPIN both tilt axes are rate controlled (the other one at 0 deg/s),
 * because Euler angles jump by 180 deg when a pitch flip passes the vertical.
 * Whether the drone is upright is judged from cos(roll) * cos(pitch), the
 * vertical part of its up axis, which does not jump.
 */
typedef enum {
    WC_FLIP_FORWARD = 0,  /* nose down first */
    WC_FLIP_BACK,
    WC_FLIP_LEFT,
    WC_FLIP_RIGHT,
} wcFlipDir_t;

typedef enum {
    WC_FLIP_OK = 0,       /* flip started */
    WC_FLIP_DISABLED,     /* switched off in the firmware configuration */
    WC_FLIP_NOT_PILOT,    /* only the web pilot can flip */
    WC_FLIP_NOT_FLYING,   /* hover thrust not learned yet, or throttle too low */
    WC_FLIP_BUSY,         /* flip running, failsafe, forced landing */
} wcFlipStatus_t;

typedef enum {
    WC_FLIP_IDLE = 0,
    WC_FLIP_BOOST,
    WC_FLIP_SPIN,
    WC_FLIP_RECOVER,
} wcFlipPhase_t;

/* Firmware parameters (group "flip"), so the ground station can tune them */
typedef struct {
    uint16_t rateDps;     /* rotation rate */
    uint16_t boostMs;     /* climb before the rotation */
    uint16_t boostPct;    /* thrust while climbing, % of the hover thrust */
    uint16_t spinPct;     /* thrust while rotating, % of the hover thrust */
    uint16_t exitDeg;     /* levelling takes over this far before upright */
    uint16_t recoverMs;   /* catching after the rotation */
    uint16_t recoverPct;  /* thrust while catching, % of the hover thrust */
    uint16_t timeoutMs;   /* longest rotation, then the drone levels out anyway */
} wcFlipConfig_t;

#define WC_FLIP_MIN_THRUST    0.1f  /* the controller needs some thrust to turn */

typedef struct {
    bool enabled;         /* compiled in */
    bool pilot;           /* the request comes from the web pilot */
    bool hoverKnown;      /* hover thrust learned */
    bool thrustUp;        /* the pilot's thrust is near the hover thrust */
    bool busy;            /* flip running, failsafe stage, forced landing, motor test */
} wcFlipInputs_t;

typedef struct {
    uint8_t  phase;       /* wcFlipPhase_t */
    uint8_t  dir;         /* wcFlipDir_t */
    bool     inverted;    /* has been upside down */
    bool     timedOut;    /* SPIN ended by the timeout */
    uint32_t phaseMs;     /* start of the current phase */
    float    hover;       /* hover thrust at the start */
} wcFlip_t;

typedef struct {
    bool  spin;           /* SPIN: both tilt axes rate controlled, yaw follows */
    float rollRateDps;    /* + = right side down */
    float pitchRateDps;   /* + = nose up */
    float thrust;         /* 0..1; outside SPIN the attitude is level (trim) */
} wcFlipOut_t;

bool wcDecodeFlip(const uint8_t *buf, size_t len, uint8_t *dir);
size_t wcEncodeFlip(uint8_t dir, uint8_t *buf, size_t cap);
size_t wcEncodeFlipReply(uint8_t dir, uint8_t status, uint8_t *buf, size_t cap);
bool wcDecodeFlipReply(const uint8_t *buf, size_t len, uint8_t *dir, uint8_t *status);
wcFlipStatus_t wcFlipCheck(const wcFlipInputs_t *in);
void wcFlipStart(wcFlip_t *f, uint8_t dir, uint32_t nowMs, float hover);
/* One control cycle with the current attitude. False once the flip is over
 * (or was not running); out is valid only while it returns true. */
bool wcFlipUpdate(wcFlip_t *f, const wcFlipConfig_t *cfg, uint32_t nowMs, float rollDeg, float pitchDeg,
                  wcFlipOut_t *out);

/* ------------------------------------------------------------------------ */
/* Parameters the network may change: the gains and the flip parameters      */
/* ------------------------------------------------------------------------ */

#define WC_FLIP_PARAM_COUNT   8
#define WC_TUNABLE_COUNT      (WC_GAIN_COUNT + WC_FLIP_PARAM_COUNT)

typedef struct {
    const char *name;
    float min;
    float max;
} wcTunable_t;

/* The gains first (same order as wcGainNames), then the flip parameters in
 * the order of wcFlipConfig_t. */
extern const wcTunable_t wcTunables[WC_TUNABLE_COUNT];
int wcTunableIndex(const char *name);   /* -1 if not changeable */
bool wcTunableValid(int index, float v);

/* Stored flip parameters (NVS blob "flip"): version, count, reserved (2),
 * WC_FLIP_PARAM_COUNT floats */
#define WC_FLIP_BLOB_VERSION  1
#define WC_FLIP_BLOB_LEN      (4 + 4 * WC_FLIP_PARAM_COUNT)
size_t wcEncodeFlipBlob(const float *values, uint8_t *buf, size_t cap);
bool wcDecodeFlipBlob(const uint8_t *buf, size_t len, float *values, bool *valid);

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
