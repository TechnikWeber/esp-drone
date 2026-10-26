#!/usr/bin/env python3
"""
Screenshots of the web control page for the README (docs/images).

    python3 tools/ui_screenshots.py            # writes docs/images/webcontrol-*.png

Starts tools/mock_drone.py and a headless Google Chrome, drives the page over
the DevTools protocol like a phone in landscape (844 x 390, 2x) and saves four
scenes: flying, trim, link lost, forced landing. With ImageMagick (`magick`)
installed, the PNGs are reduced to 192 colours (about 40 KB each). Python
standard library only; set CHROME=/path/to/chrome if it is not google-chrome.

Copyright 2026 Philipp Weber
SPDX-License-Identifier: GPL-3.0-only
"""

import base64
import json
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "docs", "images")
HTTP, WSP, DEVTOOLS = 8180, 8181, 9333
W, H, DPR = 844, 390, 2
URL = f"http://127.0.0.1:{HTTP}/?host=127.0.0.1:{WSP}&lang=en"


class WebSocket:
    """Just enough of a WebSocket client for the DevTools protocol."""

    def __init__(self, url):
        hostport, path = url[len("ws://"):].split("/", 1)
        host, port = hostport.split(":")
        self.sock = socket.create_connection((host, int(port)))
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((f"GET /{path} HTTP/1.1\r\nHost: {hostport}\r\nUpgrade: websocket\r\n"
                           f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
                           "Sec-WebSocket-Version: 13\r\n\r\n").encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            buf += self.sock.recv(4096)
        if b" 101 " not in buf.split(b"\r\n")[0]:
            raise ConnectionError(buf.split(b"\r\n")[0].decode())
        self.buf = buf.split(b"\r\n\r\n", 1)[1]

    def send(self, text):
        data = text.encode()
        n = len(data)
        if n < 126:
            head = bytes([0x81, 0x80 | n])
        elif n < 65536:
            head = bytes([0x81, 0x80 | 126]) + struct.pack("!H", n)
        else:
            head = bytes([0x81, 0x80 | 127]) + struct.pack("!Q", n)
        mask = os.urandom(4)
        self.sock.sendall(head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def _need(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(1 << 20)
            if not chunk:
                raise ConnectionError("DevTools connection closed")
            self.buf += chunk

    def recv(self):
        self._need(2)
        n, off = self.buf[1] & 0x7F, 2
        if n == 126:
            self._need(4)
            n, off = struct.unpack("!H", self.buf[2:4])[0], 4
        elif n == 127:
            self._need(10)
            n, off = struct.unpack("!Q", self.buf[2:10])[0], 10
        self._need(off + n)
        payload, self.buf = self.buf[off:off + n], self.buf[off + n:]
        return payload.decode()


class Page:
    """The control page in Chrome, driven over the DevTools protocol."""

    def __init__(self, ws):
        self.ws, self.next_id = ws, 0

    def call(self, method, **params):
        self.next_id += 1
        self.ws.send(json.dumps({"id": self.next_id, "method": method, "params": params}))
        while True:
            msg = json.loads(self.ws.recv())
            if msg.get("id") == self.next_id:
                if "error" in msg:
                    raise RuntimeError(f"{method}: {msg['error']}")
                return msg.get("result", {})

    def js(self, body):
        r = self.call("Runtime.evaluate", expression=f"(async () => {{ {body} }})()",
                      awaitPromise=True, returnByValue=True)
        if "exceptionDetails" in r:
            raise RuntimeError(r["exceptionDetails"])
        return r["result"].get("value")

    def mouse(self, kind, x, y):
        self.call("Input.dispatchMouseEvent", type=kind, x=x, y=y, button="left", clickCount=1)

    def tap(self, x, y):
        self.mouse("mousePressed", x, y)
        self.mouse("mouseReleased", x, y)

    def rect(self, selector):
        return self.js(f"const r = document.querySelector('{selector}').getBoundingClientRect();"
                       "return {x: r.left, y: r.top, w: r.width, h: r.height};")

    def tap_on(self, selector):
        r = self.rect(selector)
        self.tap(r["x"] + r["w"] / 2, r["y"] + r["h"] / 2)

    def stick_point(self, selector, x, y):
        """Screen point of stick position x, y in -1..1 (y up), see makeStick() in the page."""
        r = self.rect(selector)
        return r["x"] + r["w"] / 2 + x * 0.74 * r["w"] / 2, r["y"] + r["h"] / 2 - y * 0.74 * r["h"] / 2

    def throttle(self, value):
        self.tap(*self.stick_point("#left", 0, 2 * value - 1))

    def arm(self):
        return self.js("const el = document.getElementById('arm');"
                       "el.dispatchEvent(new PointerEvent('pointerdown', {bubbles: true}));"
                       "await new Promise(r => setTimeout(r, 1200));"
                       "el.dispatchEvent(new PointerEvent('pointerup', {bubbles: true}));"
                       "await new Promise(r => setTimeout(r, 500));"
                       "return document.getElementById('state').textContent;")

    def wait_ready(self, seconds=12):
        end = time.time() + seconds
        while time.time() < end:
            # a fresh connection (HELLO answered) with recent telemetry from a calibrated drone
            if self.js("return !!(wsOpen && infoSeen && tel && performance.now() - lastTel < 300 && (tel.state & 64));"):
                return
            time.sleep(0.25)
        raise RuntimeError("the simulated drone did not get ready")

    def shot(self, path):
        png = base64.b64decode(self.call("Page.captureScreenshot", format="png")["data"])
        with open(path, "wb") as f:
            f.write(png)


def start_mock(*extra):
    return subprocess.Popen([sys.executable, os.path.join(REPO, "tools", "mock_drone.py"),
                             "--http-port", str(HTTP), "--ws-port", str(WSP), *extra],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop(proc):
    proc.send_signal(signal.SIGTERM)
    proc.wait()


def take(page, raw):
    """The four scenes; PNGs go to the directory raw."""
    page.wait_ready()
    page.arm()
    page.throttle(0.56)
    stick = page.stick_point("#right", 0.42, 0.30)       # forward and to the right
    page.mouse("mousePressed", *stick)
    time.sleep(1.6)
    page.shot(os.path.join(raw, "webcontrol-flying.png"))
    page.mouse("mouseReleased", *stick)

    page.tap_on("#trimBtn")
    time.sleep(0.3)
    for selector, taps in (('#trim button[data-ax="r"][data-d="-1"]', 2), ('#trim button[data-ax="p"][data-d="1"]', 1)):
        for _ in range(taps):
            page.tap_on(selector)
            time.sleep(0.15)
    time.sleep(1.4)                                       # past the confirmation: values from the drone
    page.shot(os.path.join(raw, "webcontrol-trim.png"))
    page.tap_on("#trimBtn")

    page.js("paused = true; return 0;")                   # the page stops sending: link failsafe
    time.sleep(1.5)
    page.shot(os.path.join(raw, "webcontrol-failsafe.png"))
    page.js("paused = false; return 0;")
    time.sleep(1.0)


def take_battery(page, raw):
    page.throttle(0.0)
    time.sleep(1.5)                                       # the page reconnects to the new mock
    page.wait_ready()
    page.arm()
    page.throttle(0.56)
    end = time.time() + 30
    while time.time() < end and not page.js("return !!(tel && (tel.state2 & 1));"):
        time.sleep(0.3)
    time.sleep(2.5)
    page.shot(os.path.join(raw, "webcontrol-battery.png"))
    page.throttle(0.0)


def main():
    chrome_bin = os.environ.get("CHROME", "google-chrome")
    if not shutil.which(chrome_bin):
        sys.exit(f"{chrome_bin} not found, set CHROME=/path/to/chrome")
    os.makedirs(OUT, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix="ui-screenshots-")
    raw = os.path.join(tmp, "raw")
    os.makedirs(raw)
    mock = start_mock("--vbat", "4.20", "--drain", "0.002")
    chrome = subprocess.Popen([chrome_bin, "--headless=new", f"--remote-debugging-port={DEVTOOLS}",
                               f"--user-data-dir={os.path.join(tmp, 'profile')}", "--no-first-run",
                               "--no-default-browser-check", "--disable-extensions", "--hide-scrollbars",
                               "--mute-audio", "about:blank"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        target = None
        for _ in range(50):
            try:
                pages = json.load(urllib.request.urlopen(f"http://127.0.0.1:{DEVTOOLS}/json/list"))
                target = next(p for p in pages if p["type"] == "page")
                break
            except (OSError, StopIteration, ValueError):
                time.sleep(0.2)
        if target is None:
            sys.exit("Chrome did not start")
        page = Page(WebSocket(target["webSocketDebuggerUrl"]))
        page.call("Emulation.setDeviceMetricsOverride", width=W, height=H, deviceScaleFactor=DPR, mobile=True,
                  screenOrientation={"type": "landscapePrimary", "angle": 90})
        page.call("Page.enable")
        page.call("Page.addScriptToEvaluateOnNewDocument",
                  source="window.__errors = []; addEventListener('error', e => __errors.push(String(e.message)));")
        page.call("Page.navigate", url=URL)
        time.sleep(1.5)
        missing = page.js("const en = Object.keys(TEXT.en), de = Object.keys(TEXT.de);"
                          "return en.filter(k => !de.includes(k)).concat(de.filter(k => !en.includes(k)));")
        if missing:
            print("warning: texts missing in one language:", ", ".join(missing))

        take(page, raw)
        stop(mock)                                        # forced landing: a nearly empty battery
        mock = start_mock("--vbat", "3.45", "--drain", "0.03")
        take_battery(page, raw)

        errors = page.js("return window.__errors;")
        if errors:
            print("warning: errors on the page:", errors)
    finally:
        chrome.terminate()
        chrome.wait()
        stop(mock)

    magick = shutil.which("magick")
    for name in sorted(os.listdir(raw)):
        src, dst = os.path.join(raw, name), os.path.join(OUT, name)
        if magick:
            subprocess.run([magick, src, "-strip", "-dither", "None", "-colors", "192", f"PNG8:{dst}"], check=True)
        else:
            shutil.copyfile(src, dst)
        print(f"{os.path.relpath(dst, REPO)}: {os.path.getsize(dst) // 1024} KB")
    shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
