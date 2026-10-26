#!/usr/bin/env python3
"""
Simulated drone for developing and testing the web control page without
hardware. Python standard library only.

    python3 tools/mock_drone.py                      # page on :8080, control on :8081
    python3 tools/mock_drone.py --stall-every 10 --stall-ms 700

Open http://127.0.0.1:8080/?host=127.0.0.1:8081 in a browser (or on a phone
in the same network with your PC's address).

The mock speaks the same WebSocket protocol as the firmware (docs/webcontrol.md)
and imitates its link failsafe stages with the default thresholds. It is a
UI tool, the real logic is tested in components/webctrl/host_test.

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
PROTO = 2
MSG_CONTROL, MSG_HELLO, MSG_TRIM, MSG_TELEMETRY = 0x01, 0x02, 0x03, 0x81
F_ARM, F_KILL, F_PAUSE = 1, 2, 4
S_ARMED, S_YOU, S_OTHER, S_FAILSAFE, S_LOWBAT, S_BLOCKED, S_READY, S_TUMBLED = 1, 2, 4, 8, 16, 32, 64, 128
S2_BAT_LAND, S2_BAT_NOARM = 1, 2
STOP_RELEASED, STOP_KILL, STOP_LINK, STOP_TUMBLED, STOP_BATTERY = 1, 2, 3, 4, 5
OK_MS, HOLD_MS, DESCEND_MS, OFF_MS, STALE_MS = 100, 300, 1000, 4000, 250
BAT_LAND_V, BAT_ARM_V, BAT_LOW_V, BAT_DELAY_S, BAT_LAND_S, SAG_V = 3.0, 3.4, 3.2, 1.0, 10.0, 0.4
TRIM_MAX = 50


def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


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
        self.roll = self.pitch = self.yaw = 0.0
        self.thrust = 0.0
        self.vbat_rest = 4.10
        self.trim = (0, 0)          # 1/10 deg, stored on the drone
        self.stop_reason = 0
        self.landing = False        # forced landing (battery empty)
        self.land_t0 = 0
        self.land_from = 0.0
        self.low_since = None
        self.boot = time.monotonic()

    def ready(self):
        return time.monotonic() - self.boot > 3.0      # "calibration" takes 3 s

    def vbat(self):
        return max(0.0, self.vbat_rest - SAG_V * self.thrust)   # sags under load

    def age(self, t):
        if self.last is None:
            return OFF_MS
        a = (t - self.last_fresh) & 0xFFFFFFFF
        if (not self.connected or self.paused) and a < HOLD_MS:
            a = HOLD_MS
        return a

    def end_session(self, why, reason, block=True):
        print(f"[drone] disarmed: {why}")
        if self.pilot and block:
            self.pilot.released = False
        self.pilot, self.token, self.last, self.stage, self.thrust = None, 0, None, 4, 0.0
        self.stop_reason, self.landing, self.low_since = reason, False, None

    def on_control(self, c, frame, t):
        flags, seq, cms, roll, pitch, yaw, thr = struct.unpack_from("<BHIhhhH", frame, 1)
        if flags & F_KILL:
            print("[drone] KILL")
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
            c.released, c.arm_result = True, 0
            return
        if self.token:
            c.arm_result = 4
            return
        if not c.released:
            c.arm_result = 6
        elif not self.ready():
            c.arm_result = 1
        elif 0.5 <= self.vbat() < BAT_ARM_V:
            c.arm_result = 3
        elif thr > 20:
            c.arm_result = 2
        else:
            c.arm_result = 0
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
                    self.landing, self.land_t0, self.land_from = True, t, min(self.thrust, 0.55)
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
        target = (0.0, 0.0, 0.0)
        if self.token:
            age = self.age(t)
            stage = 0 if age < OK_MS else 1 if age < HOLD_MS else 2 if age < DESCEND_MS else 3 if age < OFF_MS else 4
            if stage != self.stage:
                print(f"[drone] stage {self.stage} -> {stage}")
            self.stage = stage
            roll, pitch, yaw, thr = self.last
            thrust = None
            if stage <= 1:
                target, thrust = (roll * 0.015 + tr, pitch * 0.015 + tp, yaw * 0.12), thr / 1000
            elif stage == 2:
                target, thrust = (tr, tp, 0.0), min(thr / 1000, 0.55)
            elif stage == 3:
                target, thrust = (tr, tp, 0.0), min(thr / 1000, 0.55 * 0.85)
            else:
                self.end_session("link lost, failsafe ended", STOP_LINK)
            if thrust is not None:
                thrust = self.battery(t, thrust, thr)
            if thrust is not None:
                self.thrust = thrust
        k = min(1.0, dt * 6)
        self.roll += (target[0] - self.roll) * k
        self.pitch += (-target[1] - self.pitch) * k
        self.yaw = (self.yaw + target[2] * dt + 180) % 360 - 180
        self.vbat_rest = max(2.8, self.vbat_rest - dt * (args.drain if self.thrust > 0.1 else 0.0002))

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
        seq, cms = c.echo
        mine = c is self.pilot
        return struct.pack("<BBBBHHIHhhhHHHBBIIbbH", MSG_TELEMETRY, s, self.stage if self.token else 4, c.arm_result,
                           seq, c.rate, cms, int(v * 1000), int((self.roll + off_r) * 100),
                           int((self.pitch + off_p) * 100),
                           int(self.yaw * 100), int(self.thrust * 1000), self.stale if mine else 0, 0,
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
        writer.close()


def stalled():
    if not args.stall_every:
        return False
    return (time.monotonic() % args.stall_every) * 1000 < args.stall_ms


def process(c, frame):
    t = now_ms()
    if frame[0] == MSG_HELLO and len(frame) == 8:
        token = struct.unpack_from("<I", frame, 4)[0]
        resumed = bool(drone.token and token == drone.token)
        if resumed:
            drone.pilot, drone.connected, c.released, drone.min_offset = c, True, True, None
            print(f"[drone] pilot resumed on client {c.id}")
        c.hello = True
        info = {"t": "info", "proto": PROTO, "fw": "mock", "device": "Mock drone", "resumed": resumed,
                "maxAngle": 15, "maxYawRate": 120, "telemetryHz": 20,
                "failsafe": {"ok": OK_MS, "hold": HOLD_MS, "descend": DESCEND_MS, "off": OFF_MS, "stale": STALE_MS},
                "battery": {"land": int(BAT_LAND_V * 1000), "arm": int(BAT_ARM_V * 1000),
                            "landTime": int(BAT_LAND_S * 1000)}, "trimMax": TRIM_MAX}
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
