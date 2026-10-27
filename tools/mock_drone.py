#!/usr/bin/env python3
"""
Simulated drone for developing and testing the web control page and the
desktop ground station without hardware. Python standard library only.

    python3 tools/mock_drone.py                      # page on :8080, control on :8081
    python3 tools/mock_drone.py --stall-every 10 --stall-ms 700

Control page: http://127.0.0.1:8080/?host=127.0.0.1:8081 (or on a phone in the
same network with your PC's address). Ground station:
python3 tools/groundstation/groundstation.py --drone 127.0.0.1:8081

The mock speaks the same WebSocket protocol as the firmware (docs/webcontrol.md)
and imitates its link failsafe stages with the default thresholds. Attitude and
rates come from a small simulation of the Crazyflie PID controller and the
airframe, so changed gains in the ground station change the step response. It
is a UI tool; the real logic is tested in components/webctrl/host_test.

--stall-every N --stall-ms M  hold back incoming frames for M ms every N s,
                              like a TCP retransmission stall, then deliver
                              them as a burst
--delay-ms D                  add D ms to every incoming frame
--vbat V --drain D            battery at rest V volts, losing D volts per
                              second in flight. Under load it sags by up to
                              0.4 V. Try --vbat 3.45 --drain 0.02 to see the
                              forced landing after about 10 s of hovering.
--imu-offset R,P              the sensor reads roll R and pitch P degrees while
                              the drone stands level (try "Level on the ground")
--turbulence T                strength of the simulated gusts (default 1, 0 = calm)
--legacy                      behave like firmware v0.2 (protocol 2: no gains,
                              no fast samples, no motor test, no flips)

Copyright 2026 Philipp Weber
SPDX-License-Identifier: GPL-3.0-only
"""

import argparse
import asyncio
import base64
import hashlib
import json
import math
import os
import random
import struct
import sys
import time

PAGE = os.path.join(os.path.dirname(__file__), "..", "components", "webctrl", "web", "index.html")

# protocol (webctrl_core.h)
PROTO = 4
MSG_CONTROL, MSG_HELLO, MSG_TRIM, MSG_PARAM, MSG_STREAM, MSG_MOTOR_TEST, MSG_FLIP = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
MSG_TELEMETRY, MSG_FAST, MSG_PARAM_REPLY, MSG_MOTOR_REPLY, MSG_FLIP_REPLY = 0x81, 0x82, 0x84, 0x86, 0x87
F_ARM, F_KILL, F_PAUSE = 1, 2, 4
S_ARMED, S_YOU, S_OTHER, S_FAILSAFE, S_LOWBAT, S_BLOCKED, S_READY, S_TUMBLED = 1, 2, 4, 8, 16, 32, 64, 128
S2_BAT_LAND, S2_BAT_NOARM, S2_MOTOR_TEST, S2_FLIP = 1, 2, 4, 8
FLIP_OK, FLIP_DISABLED, FLIP_NOT_PILOT, FLIP_NOT_FLYING, FLIP_BUSY = range(5)
STOP_RELEASED, STOP_KILL, STOP_LINK, STOP_TUMBLED, STOP_BATTERY = 1, 2, 3, 4, 5
ARM_OK, ARM_NOT_READY, ARM_THRUST, ARM_BATTERY, ARM_OTHER, ARM_TUMBLED, ARM_RELEASE, ARM_MOTOR_TEST = range(8)
OK_MS, HOLD_MS, DESCEND_MS, OFF_MS, STALE_MS = 100, 300, 1000, 4000, 250
BAT_LAND_V, BAT_ARM_V, BAT_LOW_V, BAT_DELAY_S, BAT_LAND_S, SAG_V = 3.0, 3.4, 3.2, 1.0, 10.0, 0.4
TRIM_MAX = 50
MAX_ANGLE, MAX_YAW_RATE = 15, 120
PARAM_READ, PARAM_WRITE, PARAM_SAVE, PARAM_RESET = range(4)
P_OK, P_UNKNOWN, P_DENIED, P_BUSY, P_INVALID = range(5)
PF_WRITABLE, PF_STORED = 1, 2
MOTOR_OK, MOTOR_NOT_READY, MOTOR_BUSY, MOTOR_DISABLED = range(4)
MOTOR_MAX_PERCENT, MOTOR_MAX_MS = 30, 3000
FAST_PERIOD_MS, FAST_SAMPLES = 10, 5
WEB_MAX_THRUST = 60000.0
HOVER = 0.55                    # thrust that lifts the simulated drone

# Controller gains of the E1 Mini (pid.h)
GAINS = {
    "pid_rate.roll_kp": 137.0, "pid_rate.roll_ki": 229.0, "pid_rate.roll_kd": 2.3,
    "pid_rate.pitch_kp": 137.0, "pid_rate.pitch_ki": 229.0, "pid_rate.pitch_kd": 2.3,
    "pid_rate.yaw_kp": 120.0, "pid_rate.yaw_ki": 16.7, "pid_rate.yaw_kd": 0.0,
    "pid_attitude.roll_kp": 5.9, "pid_attitude.roll_ki": 2.9, "pid_attitude.roll_kd": 0.0,
    "pid_attitude.pitch_kp": 5.9, "pid_attitude.pitch_ki": 2.9, "pid_attitude.pitch_kd": 0.0,
    "pid_attitude.yaw_kp": 6.0, "pid_attitude.yaw_ki": 1.0, "pid_attitude.yaw_kd": 0.35,
}
# Flip parameters (group "flip") with their ranges, as in the firmware
FLIP_PARAMS = {"flip.rate": (800.0, 360, 2000), "flip.boostMs": (250.0, 0, 600), "flip.boost": (170.0, 100, 250),
               "flip.spin": (50.0, 10, 150), "flip.exitDeg": (70.0, 10, 120), "flip.recoverMs": (350.0, 0, 1000),
               "flip.recover": (140.0, 50, 250), "flip.timeoutMs": (900.0, 300, 1500)}
# everything the network may change, with the defaults
TUNABLES = dict(GAINS, **{k: v[0] for k, v in FLIP_PARAMS.items()})
RANGES = dict({k: (0, 5000) for k in GAINS}, **{k: (v[1], v[2]) for k, v in FLIP_PARAMS.items()})
OTHER_PARAMS = {"stabilizer.controller": 1.0, "stabilizer.estimator": 1.0, "motorPowerSet.enable": 0.0,
                "powerDist.idleThrust": 0.0}


def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Pid:
    """Crazyflie pid.c: P, I with a limit, D on the error."""

    def __init__(self, ilimit):
        self.ilimit, self.integ, self.prev = ilimit, 0.0, 0.0

    def reset(self):
        self.integ, self.prev = 0.0, 0.0

    def update(self, err, kp, ki, kd, dt):
        self.integ = clamp(self.integ + err * dt, -self.ilimit, self.ilimit)
        d = (err - self.prev) / dt
        self.prev = err
        return kp * err + ki * self.integ + kd * d


class Axis:
    """One rotation axis: cascaded attitude and rate PID, sensor and motor lag,
    inertia. With 8 ms of delay in the loop, too much rate P makes it
    oscillate like a real airframe."""

    DELAY_STEPS = 4             # 4 x 2 ms: sensor filter and processing

    def __init__(self, name, gain, damping, ilimit_att, ilimit_rate):
        self.name, self.gain, self.damping = name, gain, damping
        self.att, self.rate_pid = Pid(ilimit_att), Pid(ilimit_rate)
        self.angle = self.rate = self.torque = self.gust = 0.0
        self.angle_sp = self.rate_sp = self.out = self.gyro = self.seen = 0.0
        self.rate_override = None   # rate mode (flip): deg/s instead of the attitude PID
        self.delayed = [(0.0, 0.0)] * self.DELAY_STEPS

    def reset_controller(self):
        self.att.reset()
        self.rate_pid.reset()
        self.out = 0.0

    def g(self, kind, term):
        return drone.gains[f"pid_{kind}.{self.name}_{term}"]

    def step(self, dt, flying, turbulence):
        self.delayed.append((self.angle, self.rate))
        self.seen, rate = self.delayed.pop(0)
        self.gyro = rate + random.gauss(0, 0.6)               # vibration noise
        err = (self.angle_sp - self.seen + 180) % 360 - 180
        if self.rate_override is not None:
            self.att.reset()
            self.rate_sp = self.rate_override
        else:
            self.rate_sp = self.att.update(err, self.g("attitude", "kp"), self.g("attitude", "ki"),
                                           self.g("attitude", "kd"), dt)
        self.out = clamp(self.rate_pid.update(self.rate_sp - self.gyro, self.g("rate", "kp"),
                                              self.g("rate", "ki"), self.g("rate", "kd"), dt), -32767, 32767)
        if not flying:                       # on the ground: nothing turns
            self.angle = self.rate = self.torque = self.gust = 0.0
            return
        self.torque += (self.out - self.torque) * dt / 0.02                # motor lag
        self.gust += (-self.gust / 0.4 + random.gauss(0, 900) * turbulence) * dt
        accel = self.gain * (self.torque + self.gust) - self.damping * self.rate
        self.rate += accel * dt
        self.angle = (self.angle + self.rate * dt + 180) % 360 - 180


class Drone:
    def __init__(self):
        self.clients = {}           # id -> Client
        self.pilot = None           # Client
        self.token = 0
        self.last = None            # last accepted control tuple
        self.last_fresh = 0
        self.connected = False
        self.paused = False
        self.stale = 0
        self.min_offset = None
        self.stage = 4
        self.thrust = 0.0
        self.vbat_rest = 4.10
        self.trim = (0, 0)          # 1/10 deg, stored on the drone
        self.stop_reason = 0
        self.landing = False        # forced landing (battery empty)
        self.land_t0 = 0
        self.land_from = 0.0
        self.low_since = None
        self.boot = time.monotonic()
        self.gains = dict(TUNABLES)
        self.stored = None          # gains "in flash"
        self.roll = Axis("roll", 0.40, 2.0, 20.0, 33.3)
        self.pitch = Axis("pitch", 0.40, 2.0, 20.0, 33.3)
        self.yaw = Axis("yaw", 0.08, 1.5, 360.0, 166.7)
        self.motors = [0, 0, 0, 0]
        self.motor_test = None      # (client, mask, percent, until_ms)
        self.flip = None            # running flip (dict)
        self.hover = 0.0            # learned hover thrust
        self.hover_s = 0.0          # seconds of flight in it
        self.fast = []              # samples of the current FAST frame
        self.fast_seq = 0
        self.fast_t0 = 0

    def ready(self):
        return time.monotonic() - self.boot > 3.0      # "calibration" takes 3 s

    def vbat(self):
        return max(0.0, self.vbat_rest - SAG_V * self.thrust)   # sags under load

    def busy(self):
        return bool(self.token) or self.motor_test is not None

    def flip_update(self, t):
        """The firmware's flip (wcFlipUpdate): returns (spin, roll rate, pitch rate, thrust) or None when over."""
        f, cfg = self.flip, self.gains
        hover = f["hover"]
        if f["phase"] == "boost":
            if (t - f["t0"]) & 0xFFFFFFFF < cfg["flip.boostMs"]:
                return False, 0, 0, min(1.0, hover * cfg["flip.boost"] / 100)
            f["phase"], f["t0"] = "spin", t
        if f["phase"] == "spin":
            up = math.cos(math.radians(self.roll.angle)) * math.cos(math.radians(self.pitch.angle))
            f["inverted"] = f["inverted"] or up < -0.5
            done = f["inverted"] and up > math.cos(math.radians(cfg["flip.exitDeg"]))
            timeout = not done and (t - f["t0"]) & 0xFFFFFFFF >= cfg["flip.timeoutMs"]
            if not done and not timeout:
                rate, d = cfg["flip.rate"], f["dir"]
                return (True, rate if d == 3 else -rate if d == 2 else 0, -rate if d == 0 else rate if d == 1 else 0,
                        max(0.1, min(1.0, hover * cfg["flip.spin"] / 100)))
            print(f"[drone] flip {'timed out' if timeout else 'done'}")
            f["phase"], f["t0"] = "recover", t
        if (t - f["t0"]) & 0xFFFFFFFF < cfg["flip.recoverMs"]:
            return False, 0, 0, min(1.0, hover * cfg["flip.recover"] / 100)
        self.flip = None
        return None

    def age(self, t):
        if self.last is None:
            return OFF_MS
        a = (t - self.last_fresh) & 0xFFFFFFFF
        if (not self.connected or self.paused) and a < HOLD_MS:
            a = HOLD_MS
        return a

    def end_session(self, why, reason, block=True):
        print(f"[drone] disarmed: {why}")
        self.flip, self.hover, self.hover_s = None, 0.0, 0.0
        self.roll.rate_override = self.pitch.rate_override = None
        if self.pilot and block:
            self.pilot.released = False
        self.pilot, self.token, self.last, self.stage, self.thrust = None, 0, None, 4, 0.0
        self.stop_reason, self.landing, self.low_since = reason, False, None

    def stop_motor_test(self, why):
        if self.motor_test:
            print(f"[drone] motor test stopped: {why}")
            self.motor_test = None

    def on_control(self, c, frame, t):
        flags, seq, cms, roll, pitch, yaw, thr = struct.unpack_from("<BHIhhhH", frame, 1)
        if flags & F_KILL:
            print("[drone] KILL")
            self.stop_motor_test("KILL")
            if self.token:
                self.end_session("KILL", STOP_KILL)
            self.stop_reason = STOP_KILL
            c.released = False
            return
        offset = (t - cms) & 0xFFFFFFFF
        offset = offset - (1 << 32) if offset >= (1 << 31) else offset
        if c is self.pilot:
            if self.min_offset is None or offset < self.min_offset:
                self.min_offset = offset
            if offset - self.min_offset > STALE_MS:
                self.stale += 1
                return
            c.echo = (seq, cms)
            c.rx += 1
            self.last, self.last_fresh = (roll, pitch, yaw, thr), t
            self.paused = bool(flags & F_PAUSE)
            if not flags & F_ARM:
                c.released = True
                self.end_session("pilot released ARM", STOP_RELEASED, block=False)
            return
        c.echo = (seq, cms)
        c.rx += 1
        if not flags & F_ARM:
            c.released, c.arm_result = True, ARM_OK
            return
        if self.token:
            c.arm_result = ARM_OTHER
            return
        if self.motor_test:
            c.arm_result = ARM_MOTOR_TEST
        elif not c.released:
            c.arm_result = ARM_RELEASE
        elif not self.ready():
            c.arm_result = ARM_NOT_READY
        elif 0.5 <= self.vbat() < BAT_ARM_V:
            c.arm_result = ARM_BATTERY
        elif thr > 20:
            c.arm_result = ARM_THRUST
        else:
            c.arm_result = ARM_OK
            self.pilot, self.token = c, random.getrandbits(31) | 1
            self.last, self.last_fresh, self.connected, self.stale = (roll, pitch, yaw, thr), t, True, 0
            self.min_offset = offset
            print(f"[drone] armed by client {c.id}")

    def battery(self, t, thrust, pilot_thr):
        """Forced landing as in the firmware. Returns the capped thrust, None if disarmed."""
        v = self.vbat()
        if not self.landing:
            if 0.5 <= v < BAT_LAND_V:
                self.low_since = t if self.low_since is None else self.low_since
                if (t - self.low_since) / 1000 >= BAT_DELAY_S:
                    self.low_since = None
                    if self.thrust < 0.15:
                        self.end_session("battery empty", STOP_BATTERY)
                        return None
                    self.landing, self.land_t0, self.land_from = True, t, min(self.thrust, HOVER)
                    print(f"[drone] battery {v:.2f} V: forced landing")
            else:
                self.low_since = None
        if self.landing:
            thrust = min(thrust, self.land_from * max(0.0, 1 - (t - self.land_t0) / 1000 / BAT_LAND_S))
            if thrust <= 0 or pilot_thr == 0:
                self.end_session("battery empty, landed", STOP_BATTERY)
                return None
        return thrust

    def step(self, dt):
        t = now_ms()
        tr, tp = self.trim[0] / 10, self.trim[1] / 10
        roll_sp, pitch_sp, yaw_rate = 0.0, 0.0, 0.0       # pitch + = forward, yaw + = to the right
        if self.token:
            age = self.age(t)
            stage = 0 if age < OK_MS else 1 if age < HOLD_MS else 2 if age < DESCEND_MS else 3 if age < OFF_MS else 4
            if stage != self.stage:
                print(f"[drone] stage {self.stage} -> {stage}")
            self.stage = stage
            roll, pitch, yaw, thr = self.last
            thrust = None
            if stage <= 1:
                roll_sp = clamp(roll / 1000 * MAX_ANGLE + tr, -MAX_ANGLE, MAX_ANGLE)
                pitch_sp = clamp(pitch / 1000 * MAX_ANGLE + tp, -MAX_ANGLE, MAX_ANGLE)
                yaw_rate, thrust = yaw / 1000 * MAX_YAW_RATE, thr / 1000
            elif stage == 2:
                roll_sp, pitch_sp, thrust = tr, tp, min(thr / 1000, HOVER)
            elif stage == 3:
                roll_sp, pitch_sp, thrust = tr, tp, min(thr / 1000, HOVER * 0.85)
            else:
                self.end_session("link lost, failsafe ended", STOP_LINK)
            rates = None
            if self.flip and thrust is not None:
                res = self.flip_update(t)
                if res:
                    spin, rr, pr, thrust = res
                    roll_sp, pitch_sp, yaw_rate = tr, tp, 0.0
                    rates = (rr, pr) if spin else None
            elif stage == 0 and thrust is not None and thrust > 0.15:
                self.hover = thrust if not self.hover_s else self.hover + (thrust - self.hover) * min(1.0, dt / 3)
                self.hover_s += dt
            self.roll.rate_override, self.pitch.rate_override = rates if rates else (None, None)
            if thrust is not None:
                thrust = self.battery(t, thrust, thr)
            if thrust is not None:
                self.thrust = thrust
        if self.motor_test and ((t - self.motor_test[3]) & 0xFFFFFFFF) < 0x80000000:
            self.stop_motor_test("time is up")
        self.simulate(dt, roll_sp, pitch_sp, yaw_rate)
        self.vbat_rest = max(2.8, self.vbat_rest - dt * (args.drain if self.thrust > 0.1 else 0.0002))

    def simulate(self, dt, roll_sp, pitch_sp, yaw_rate):
        """Crazyflie conventions inside: pitch + = nose up, yaw + = counter-clockwise."""
        armed = bool(self.token)
        flying = armed and (self.thrust > HOVER * 0.75 or self.flip is not None or
                            abs(self.roll.angle) > 1 or abs(self.pitch.angle) > 1)
        steps = max(1, min(50, round(dt / 0.002)))
        h = dt / steps
        for _ in range(steps):
            self.roll.angle_sp = roll_sp
            self.pitch.angle_sp = -pitch_sp
            if armed and self.thrust > 0:
                self.yaw.angle_sp = (self.yaw.angle_sp - yaw_rate * h + 180) % 360 - 180
            else:                                           # thrust 0: the controller resets
                self.yaw.angle_sp = self.yaw.angle
                for ax in (self.roll, self.pitch, self.yaw):
                    ax.reset_controller()
            for ax in (self.roll, self.pitch, self.yaw):
                ax.step(h, flying, args.turbulence)
        # mixer (power_distribution_stock.c, X frame)
        if self.motor_test:
            _, mask, percent, _ = self.motor_test
            ratio = int(percent * 65535 / 100)
            self.motors = [ratio if mask & (1 << m) else 0 for m in range(4)]
        elif armed and self.thrust > 0:
            thrust = self.thrust * WEB_MAX_THRUST
            r, p, y = self.roll.out / 2, self.pitch.out / 2, -self.yaw.out
            self.motors = [int(clamp(v, 0, 65535)) for v in
                           (thrust - r + p + y, thrust - r - p - y, thrust + r - p + y, thrust + r + p - y)]
        else:
            self.motors = [0, 0, 0, 0]

    def sample(self):
        """One FAST sample (webctrl_core.h, wcFastSample_t)."""
        off_r, off_p = (0.0, 0.0) if self.token else args.imu_offset

        def cdeg(v):
            return int(clamp(round(v * 100), -32767, 32767))

        def ddeg(v):
            return int(clamp(round(v * 10), -32767, 32767))
        r, p, y = self.roll, self.pitch, self.yaw
        on = bool(self.token)
        return struct.pack("<11h5H", cdeg(r.angle + off_r), cdeg(p.angle + off_p), cdeg(y.angle),
                           cdeg(r.angle_sp if on else 0), cdeg(p.angle_sp if on else 0),
                           ddeg(r.gyro), ddeg(p.gyro), ddeg(y.gyro),
                           ddeg(r.rate_sp if on else 0), ddeg(p.rate_sp if on else 0), ddeg(y.rate_sp if on else 0),
                           *self.motors, int(clamp(self.thrust * WEB_MAX_THRUST, 0, 65535)))

    def telemetry(self, c):
        t = now_ms()
        s = 0
        if self.token:
            s |= S_ARMED
            if c is self.pilot:
                s |= S_YOU
            else:
                s |= S_OTHER
            if self.stage >= 2:
                s |= S_FAILSAFE
        # On the ground a tilted sensor shows its offset; in flight the controller
        # holds the reading at the setpoint, so the offset is not visible.
        off_r, off_p = (0.0, 0.0) if self.token else args.imu_offset
        v = self.vbat()
        if 0.5 <= v < BAT_LOW_V:
            s |= S_LOWBAT
        if not c.released:
            s |= S_BLOCKED
        if self.ready():
            s |= S_READY
        s2 = S2_BAT_LAND if self.token and self.landing else 0
        if not self.token and 0.5 <= v < BAT_ARM_V:
            s2 |= S2_BAT_NOARM
        if self.motor_test:
            s2 |= S2_MOTOR_TEST
        if self.flip:
            s2 |= S2_FLIP
        seq, cms = c.echo
        mine = c is self.pilot
        return struct.pack("<BBBBHHIHhhhHHHBBIIbbH", MSG_TELEMETRY, s, self.stage if self.token else 4, c.arm_result,
                           seq, c.rate, cms, int(v * 1000), int((self.roll.angle + off_r) * 100),
                           int((self.pitch.angle + off_p) * 100),
                           int(self.yaw.angle * 100), int(self.thrust * 1000), self.stale if mine else 0, 0,
                           s2, self.stop_reason, t, self.token if mine else 0, self.trim[0], self.trim[1], 0)


class Client:
    def __init__(self, cid, writer):
        self.id, self.writer = cid, writer
        self.released = False
        self.arm_result = 0
        self.echo = (0, 0)
        self.rx = 0
        self.rate = 0
        self.hello = False      # telemetry only after HELLO, as in the firmware
        self.fast = False       # wants FAST samples


drone = Drone()
args = None


def ws_frame(opcode, payload):
    n = len(payload)
    if n < 126:
        head = struct.pack("!BB", 0x80 | opcode, n)
    elif n < 65536:
        head = struct.pack("!BBH", 0x80 | opcode, 126, n)
    else:
        head = struct.pack("!BBQ", 0x80 | opcode, 127, n)
    return head + payload


async def read_frame(reader):
    b0, b1 = await reader.readexactly(2)
    opcode, n = b0 & 0x0F, b1 & 0x7F
    if n == 126:
        n = struct.unpack("!H", await reader.readexactly(2))[0]
    elif n == 127:
        n = struct.unpack("!Q", await reader.readexactly(8))[0]
    mask = await reader.readexactly(4) if b1 & 0x80 else b"\0\0\0\0"
    data = bytearray(await reader.readexactly(n))
    for i in range(n):
        data[i] ^= mask[i % 4]
    return opcode, bytes(data)


async def handle_ws(reader, writer):
    request = await reader.readuntil(b"\r\n\r\n")
    key = None
    for line in request.decode("latin1").split("\r\n"):
        if line.lower().startswith("sec-websocket-key:"):
            key = line.split(":", 1)[1].strip()
    if key is None:
        writer.close()
        return
    accept = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
    writer.write(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                  f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode())
    await writer.drain()

    c = Client(id(writer) & 0xFFFF, writer)
    drone.clients[c.id] = c
    print(f"[ws] client {c.id} connected")
    pending = []          # frames held back during a simulated stall
    try:
        while True:
            opcode, data = await read_frame(reader)
            if opcode == 8:
                break
            if opcode == 9:
                writer.write(ws_frame(10, data))
                continue
            if opcode != 2 or not data:
                continue
            if args.delay_ms:
                await asyncio.sleep(args.delay_ms / 1000)
            if stalled():
                pending.append(data)
                continue
            for frame in pending + [data]:
                process(c, frame)
            pending = []
    except (asyncio.IncompleteReadError, ConnectionError):
        pass
    finally:
        print(f"[ws] client {c.id} closed")
        drone.clients.pop(c.id, None)
        if drone.pilot is c:
            drone.pilot, drone.connected = None, False
        if drone.motor_test and drone.motor_test[0] is c:
            drone.stop_motor_test("ground station left")
        writer.close()


def stalled():
    if not args.stall_every:
        return False
    return (time.monotonic() % args.stall_every) * 1000 < args.stall_ms


def param_reply(c, op, req, status, name="", value=0.0, default=float("nan"), flags=0):
    if drone.stored is not None:
        flags |= PF_STORED
    c.writer.write(ws_frame(2, struct.pack("<BBBBffBB", MSG_PARAM_REPLY, op, req, status, value, default, flags, 0)
                            + name.encode()))


def on_param(c, frame):
    if len(frame) < 8 or frame[1] > PARAM_RESET:
        return
    op, req = frame[1], frame[2]
    value = struct.unpack_from("<f", frame, 4)[0]
    name = frame[8:].decode("ascii", "replace")
    if op in (PARAM_SAVE, PARAM_RESET):
        if drone.busy():
            param_reply(c, op, req, P_BUSY)
            return
        if op == PARAM_RESET:
            drone.gains = dict(TUNABLES)
        drone.stored = None if drone.gains == TUNABLES else dict(drone.gains)
        print(f"[drone] gains {'reset' if op == PARAM_RESET else 'stored'}")
        param_reply(c, op, req, P_OK)
        return
    if name in drone.gains and not drone.ready():
        # like the firmware: the gains are set up once the system has started
        param_reply(c, op, req, P_BUSY if op == PARAM_WRITE else P_OK, name, drone.gains[name])
    elif name in drone.gains:
        status = P_OK
        if op == PARAM_WRITE:
            lo, hi = RANGES[name]
            if drone.motor_test or (drone.token and c is not drone.pilot) or (drone.flip and name in FLIP_PARAMS):
                status = P_BUSY
            elif not (math.isfinite(value) and lo <= value <= hi):
                status = P_INVALID
            else:
                drone.gains[name] = round(value) if name in FLIP_PARAMS else value
                print(f"[drone] {name} = {value:.4g}")
        param_reply(c, op, req, status, name, drone.gains[name], TUNABLES[name], PF_WRITABLE)
    elif name in OTHER_PARAMS:
        param_reply(c, op, req, P_DENIED if op == PARAM_WRITE else P_OK, name, OTHER_PARAMS[name])
    else:
        param_reply(c, op, req, P_UNKNOWN, name)


def on_motor_test(c, frame):
    mask, percent, _, duration = struct.unpack_from("<BBBH", frame, 1)
    if mask > 15 or percent > 100 or (mask and (not percent or not duration)):
        return
    status, applied = MOTOR_OK, (0, 0, 0)
    if mask == 0:
        drone.stop_motor_test("stop request")
    elif drone.token:
        status = MOTOR_BUSY
    elif not drone.ready():
        status = MOTOR_NOT_READY
    else:
        applied = (mask, min(percent, MOTOR_MAX_PERCENT), min(duration, MOTOR_MAX_MS))
        drone.motor_test = (c, applied[0], applied[1], (now_ms() + applied[2]) & 0xFFFFFFFF)
        print(f"[drone] motor test: motors {mask:#x} at {applied[1]} % for {applied[2]} ms")
    c.writer.write(ws_frame(2, struct.pack("<BBBBHH", MSG_MOTOR_REPLY, applied[0], applied[1], status, applied[2], 0)))


def on_flip(c, frame):
    d = frame[1]
    if d > 3:
        return
    if c is not drone.pilot or not drone.token:
        status = FLIP_NOT_PILOT
    elif drone.flip or drone.stage != 0 or drone.landing or drone.motor_test:
        status = FLIP_BUSY
    elif drone.hover_s < 2 or drone.thrust < 0.7 * drone.hover:
        status = FLIP_NOT_FLYING
    else:
        status = FLIP_OK
        drone.flip = {"phase": "boost", "dir": d, "t0": now_ms(), "inverted": False, "hover": drone.hover}
        print(f"[drone] flip {['forward', 'back', 'left', 'right'][d]} (hover {drone.hover:.2f})")
    c.writer.write(ws_frame(2, struct.pack("<BBBB", MSG_FLIP_REPLY, d, status, 0)))


def process(c, frame):
    t = now_ms()
    if frame[0] == MSG_HELLO and len(frame) == 8:
        token = struct.unpack_from("<I", frame, 4)[0]
        resumed = bool(drone.token and token == drone.token)
        if resumed:
            drone.pilot, drone.connected, c.released, drone.min_offset = c, True, True, None
            print(f"[drone] pilot resumed on client {c.id}")
        c.hello = True
        info = {"t": "info", "proto": 2 if args.legacy else PROTO, "fw": "mock", "device": "Mock drone",
                "resumed": resumed, "maxAngle": MAX_ANGLE, "maxYawRate": MAX_YAW_RATE, "telemetryHz": 20,
                "failsafe": {"ok": OK_MS, "hold": HOLD_MS, "descend": DESCEND_MS, "off": OFF_MS, "stale": STALE_MS},
                "battery": {"land": int(BAT_LAND_V * 1000), "arm": int(BAT_ARM_V * 1000),
                            "landTime": int(BAT_LAND_S * 1000)}, "trimMax": TRIM_MAX}
        if not args.legacy:
            info.update({"fastHz": 1000 // FAST_PERIOD_MS,
                         "motorTest": {"maxPercent": MOTOR_MAX_PERCENT, "maxMs": MOTOR_MAX_MS}, "flip": True})
        c.writer.write(ws_frame(1, json.dumps(info).encode()))
    elif frame[0] == MSG_CONTROL and len(frame) == 16:
        drone.on_control(c, frame, t)
    elif frame[0] == MSG_TRIM and len(frame) == 4:
        r, p = struct.unpack_from("<bb", frame, 1)
        # on the ground every page may trim, in flight only the pilot
        if abs(r) <= TRIM_MAX and abs(p) <= TRIM_MAX and (not drone.token or c is drone.pilot) \
                and (r, p) != drone.trim:
            drone.trim = (r, p)
            print(f"[drone] trim roll {r / 10:.1f}, pitch {p / 10:.1f} deg")
    elif args.legacy:
        return                          # firmware v0.2 ignores the rest
    elif frame[0] == MSG_STREAM and len(frame) == 4:
        c.fast = bool(frame[1] & 1)
    elif frame[0] == MSG_PARAM:
        on_param(c, frame)
    elif frame[0] == MSG_MOTOR_TEST and len(frame) == 8:
        on_motor_test(c, frame)
    elif frame[0] == MSG_FLIP and len(frame) == 4:
        on_flip(c, frame)


async def ticker():
    last = time.monotonic()
    tel = 0.0
    rate_t = time.monotonic()
    while True:
        await asyncio.sleep(0.01)
        now = time.monotonic()
        drone.step(now - last)
        last = now
        if now - rate_t >= 1:
            rate_t = now
            for c in drone.clients.values():
                c.rate, c.rx = c.rx, 0
        fast_clients = [c for c in drone.clients.values() if c.fast and c.hello]
        if fast_clients:
            if not drone.fast:
                drone.fast_t0 = now_ms()
            drone.fast.append(drone.sample())
            if len(drone.fast) == FAST_SAMPLES:
                frame = struct.pack("<BBHIB3x", MSG_FAST, FAST_SAMPLES, drone.fast_seq & 0xFFFF, drone.fast_t0,
                                    FAST_PERIOD_MS) + b"".join(drone.fast)
                drone.fast_seq += FAST_SAMPLES
                drone.fast = []
                for c in fast_clients:
                    try:
                        c.writer.write(ws_frame(2, frame))
                    except ConnectionError:
                        pass
        else:
            drone.fast = []
        if now - tel >= 0.05:
            tel = now
            for c in list(drone.clients.values()):
                if not c.hello:
                    continue
                try:
                    c.writer.write(ws_frame(2, drone.telemetry(c)))
                except ConnectionError:
                    pass


async def handle_http(reader, writer):
    try:
        await reader.readuntil(b"\r\n\r\n")
        with open(PAGE, "rb") as f:
            body = f.read()
        writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-store\r\n"
                     + f"Content-Length: {len(body)}\r\nConnection: close\r\n\r\n".encode() + body)
        await writer.drain()
    finally:
        writer.close()


async def main():
    global args
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bind", default="127.0.0.1")
    p.add_argument("--http-port", type=int, default=8080)
    p.add_argument("--ws-port", type=int, default=8081)
    p.add_argument("--delay-ms", type=int, default=0)
    p.add_argument("--stall-every", type=float, default=0)
    p.add_argument("--stall-ms", type=int, default=0)
    p.add_argument("--vbat", type=float, default=4.10)
    p.add_argument("--drain", type=float, default=0.004)
    p.add_argument("--imu-offset", default="0,0", type=lambda v: tuple(float(x) for x in v.split(",")),
                   help="roll,pitch in degrees")
    p.add_argument("--turbulence", type=float, default=1.0)
    p.add_argument("--legacy", action="store_true")
    args = p.parse_args()
    drone.vbat_rest = args.vbat
    sys.stdout.reconfigure(line_buffering=True)
    http = await asyncio.start_server(handle_http, args.bind, args.http_port)
    ws = await asyncio.start_server(handle_ws, args.bind, args.ws_port)
    print(f"page:    http://{args.bind}:{args.http_port}/?host={args.bind}:{args.ws_port}")
    asyncio.create_task(ticker())
    async with http, ws:
        await asyncio.gather(http.serve_forever(), ws.serve_forever())


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
