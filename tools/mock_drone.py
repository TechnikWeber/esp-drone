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
MSG_CONTROL, MSG_HELLO, MSG_TELEMETRY = 0x01, 0x02, 0x81
F_ARM, F_KILL, F_PAUSE = 1, 2, 4
S_ARMED, S_YOU, S_OTHER, S_FAILSAFE, S_LOWBAT, S_BLOCKED, S_READY, S_TUMBLED = 1, 2, 4, 8, 16, 32, 64, 128
OK_MS, HOLD_MS, DESCEND_MS, OFF_MS, STALE_MS = 100, 300, 1000, 4000, 250


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
        self.vbat = 4.10
        self.boot = time.monotonic()

    def ready(self):
        return time.monotonic() - self.boot > 3.0      # "calibration" takes 3 s

    def age(self, t):
        if self.last is None:
            return OFF_MS
        a = (t - self.last_fresh) & 0xFFFFFFFF
        if (not self.connected or self.paused) and a < HOLD_MS:
            a = HOLD_MS
        return a

    def end_session(self, why, block=True):
        print(f"[drone] disarmed: {why}")
        if self.pilot and block:
            self.pilot.released = False
        self.pilot, self.token, self.last, self.stage, self.thrust = None, 0, None, 4, 0.0

    def on_control(self, c, frame, t):
        flags, seq, cms, roll, pitch, yaw, thr = struct.unpack_from("<BHIhhhH", frame, 1)
        if flags & F_KILL:
            print("[drone] KILL")
            if self.token:
                self.end_session("KILL")
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
                self.end_session("pilot released ARM", block=False)
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
        elif thr > 20:
            c.arm_result = 2
        else:
            c.arm_result = 0
            self.pilot, self.token = c, random.getrandbits(31) | 1
            self.last, self.last_fresh, self.connected, self.stale = (roll, pitch, yaw, thr), t, True, 0
            self.min_offset = offset
            print(f"[drone] armed by client {c.id}")

    def step(self, dt):
        t = now_ms()
        target = (0.0, 0.0, 0.0)
        if self.token:
            age = self.age(t)
            stage = 0 if age < OK_MS else 1 if age < HOLD_MS else 2 if age < DESCEND_MS else 3 if age < OFF_MS else 4
            if stage != self.stage:
                print(f"[drone] stage {self.stage} -> {stage}")
            self.stage = stage
            roll, pitch, yaw, thr = self.last
            if stage <= 1:
                target, self.thrust = (roll * 0.015, pitch * 0.015, yaw * 0.12), thr / 1000
            elif stage == 2:
                self.thrust = min(thr / 1000, 0.55)
            elif stage == 3:
                self.thrust = min(thr / 1000, 0.55 * 0.85)
            else:
                self.end_session("link lost, failsafe ended")
        k = min(1.0, dt * 6)
        self.roll += (target[0] - self.roll) * k
        self.pitch += (-target[1] - self.pitch) * k
        self.yaw = (self.yaw + target[2] * dt + 180) % 360 - 180
        self.vbat = max(3.3, self.vbat - dt * (0.004 if self.thrust > 0.1 else 0.0002))

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
        if self.vbat < 3.5:
            s |= S_LOWBAT
        if not c.released:
            s |= S_BLOCKED
        if self.ready():
            s |= S_READY
        seq, cms = c.echo
        mine = c is self.pilot
        return struct.pack("<BBBBHHIHhhhHHHHII", MSG_TELEMETRY, s, self.stage if self.token else 4, c.arm_result,
                           seq, c.rate, cms, int(self.vbat * 1000), int(self.roll * 100), int(self.pitch * 100),
                           int(self.yaw * 100), int(self.thrust * 1000), self.stale if mine else 0, 0, 0, t,
                           self.token if mine else 0)


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
        info = {"t": "info", "proto": 1, "fw": "mock", "device": "Mock drone", "resumed": resumed,
                "maxAngle": 15, "maxYawRate": 120, "telemetryHz": 20,
                "failsafe": {"ok": OK_MS, "hold": HOLD_MS, "descend": DESCEND_MS, "off": OFF_MS, "stale": STALE_MS}}
        c.writer.write(ws_frame(1, json.dumps(info).encode()))
    elif frame[0] == MSG_CONTROL and len(frame) == 16:
        drone.on_control(c, frame, t)


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
    args = p.parse_args()
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
