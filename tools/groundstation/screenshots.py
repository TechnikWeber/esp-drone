#!/usr/bin/env python3
"""
Screenshots of the desktop ground station for the README (docs/images).

    python3 tools/groundstation/screenshots.py     # writes docs/images/groundstation-*.png

Starts tools/mock_drone.py, the ground station (groundstation.py) and a headless
Google Chrome with a simulated DualSense gamepad, flies a short pattern and
saves four scenes: flight, step test, motor test, recorder. With ImageMagick
(`magick`) installed, the PNGs are reduced to 256 colours. Python standard
library only; set CHROME=/path/to/chrome if it is not google-chrome.

Copyright 2026 Philipp Weber
SPDX-License-Identifier: GPL-3.0-only
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "tools"))
sys.dont_write_bytecode = True                # no __pycache__ in the repository
from ui_screenshots import Page, WebSocket  # noqa: E402  (shared DevTools client)

OUT = os.path.join(REPO, "docs", "images")
HTTP, WSP, GS, DEVTOOLS = 8280, 8281, 8290, 9433
W, H = 1440, 900

# A DualSense as Chrome reports it; the page polls navigator.getGamepads().
FAKE_PAD = r"""
window.__errors = []; addEventListener('error', e => __errors.push(String(e.message)));
window.__pad = { id: "DualSense Wireless Controller (STANDARD GAMEPAD Vendor: 054c Product: 0ce6)", index: 0,
  connected: true, mapping: "standard", timestamp: 0, axes: [0, 0, 0, 0],
  buttons: Array.from({length: 18}, () => ({pressed: false, touched: false, value: 0})),
  vibrationActuator: { type: "dual-rumble", playEffect() { return Promise.resolve("complete"); } } };
Object.defineProperty(Navigator.prototype, "getGamepads", { value: () => [window.__pad, null, null, null] });
window.__btn = (i, v) => { __pad.buttons[i] = {pressed: v > 0.5, touched: v > 0, value: v}; };
window.__ax = (i, v) => { __pad.axes[i] = v; };
"""


class Pad:
    def __init__(self, page):
        self.page = page

    def axes(self, **kw):
        names = {"lx": 0, "ly": 1, "rx": 2, "ry": 3}
        self.page.js("".join(f"__ax({names[k]}, {v});" for k, v in kw.items()) + "return 0;")

    def press(self, button, hold=0.12):
        self.page.js(f"__btn({button}, 1); return 0;")
        time.sleep(hold)
        self.page.js(f"__btn({button}, 0); return 0;")


def wait_for(page, expr, seconds=15):
    end = time.time() + seconds
    while time.time() < end:
        if page.js(f"return !!({expr});"):
            return
        time.sleep(0.25)
    raise RuntimeError(f"timeout waiting for {expr}")


def scenes(page, raw):
    pad = Pad(page)
    wait_for(page, "link.tel && (link.tel.state & 64) && Object.keys(gains).length === 18")
    page.js("showPanel('pad'); settings.plotSet = 'flight'; buildPlots(); return 0;")

    # 1. flight: arm with Options, climb, fly a little pattern
    pad.press(9, 1.25)
    pad.axes(ly=-0.62)
    time.sleep(1.5)
    for rx, ry, lx, dur in ((0.55, 0, 0, 0.9), (0, 0, 0, 0.5), (0, -0.5, 0, 0.9), (0, 0, 0.6, 0.6),
                            (-0.45, 0.3, 0, 0.9), (0, 0, 0, 0.5)):
        pad.axes(rx=rx, ry=ry, lx=lx)
        time.sleep(dur)
    pad.axes(rx=0.35, ry=-0.25)
    time.sleep(0.7)
    page.shot(os.path.join(raw, "groundstation-flight.png"))
    pad.axes(rx=0, ry=0, lx=0)
    time.sleep(1.0)

    # 2. tuning: a step test with a soft outer loop, a stiffer one, compare
    page.js("showPanel('tune'); return 0;")
    page.js("sendParam(P_WRITE, 'pid_attitude.roll_kp', 4.0); return 0;")
    time.sleep(0.6)
    pad.press(5)                                          # R1: roll step test
    time.sleep(2.4)
    page.js("plotView.range = null; plotView.frozen = null; renderFreeze(); return 0;")
    page.js("const el = document.querySelector('[data-gain=\"pid_attitude.roll_kp\"]'); el.value = '7.5';"
            "el.dispatchEvent(new Event('change')); applyGains(); return 0;")
    time.sleep(0.8)
    pad.press(5)
    time.sleep(2.4)
    page.shot(os.path.join(raw, "groundstation-tuning.png"))
    page.js("plotView.range = null; plotView.frozen = null; renderFreeze(); return 0;")

    # 3. land, disarm, motor test
    pad.axes(ly=0)
    time.sleep(0.5)
    pad.press(9)
    time.sleep(3.6)                                       # the recording ends 3 s after landing
    page.js("showPanel('motors'); settings.plotSet = 'flight'; buildPlots();"
            "document.getElementById('propsOff').checked = true; renderMotors(); return 0;")
    time.sleep(0.3)
    page.js("startMotorTest(2); return 0;")               # M2, rear right
    time.sleep(0.8)
    page.shot(os.path.join(raw, "groundstation-motors.png"))
    time.sleep(2.5)

    # 4. a second short flight, then the recorder with a minute of plots
    pad.press(9, 1.25)
    pad.axes(ly=-0.6)
    for rx, ry, dur in ((0, 0, 1.2), (-0.5, 0.4, 0.8), (0, 0, 0.6), (0.6, 0, 0.7), (0, 0, 0.8)):
        pad.axes(rx=rx, ry=ry)
        time.sleep(dur)
    pad.press(0)                                          # ✕: a marker in the recording
    pad.axes(ly=0, rx=0, ry=0)
    time.sleep(0.5)
    pad.press(9)
    time.sleep(3.6)
    page.js("showPanel('rec'); settings.plotWin = 60; document.getElementById('plotWin').value = '60'; return 0;")
    time.sleep(1.0)
    page.shot(os.path.join(raw, "groundstation-recorder.png"))


def main():
    chrome_bin = os.environ.get("CHROME", "google-chrome")
    if not shutil.which(chrome_bin):
        sys.exit(f"{chrome_bin} not found, set CHROME=/path/to/chrome")
    os.makedirs(OUT, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix="gs-screenshots-")
    raw = os.path.join(tmp, "raw")
    os.makedirs(raw)
    procs = [
        subprocess.Popen([sys.executable, os.path.join(REPO, "tools", "mock_drone.py"), "--http-port", str(HTTP),
                          "--ws-port", str(WSP), "--vbat", "4.15", "--drain", "0.002", "--turbulence", "0.6"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        subprocess.Popen([sys.executable, os.path.join(HERE, "groundstation.py"), "--port", str(GS), "--no-browser"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        subprocess.Popen([chrome_bin, "--headless=new", f"--remote-debugging-port={DEVTOOLS}",
                          f"--user-data-dir={os.path.join(tmp, 'profile')}", "--no-first-run",
                          "--no-default-browser-check", "--disable-extensions", "--hide-scrollbars",
                          "--mute-audio", "about:blank"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
    ]
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
        page.call("Emulation.setDeviceMetricsOverride", width=W, height=H, deviceScaleFactor=1, mobile=False)
        page.call("Emulation.setFocusEmulationEnabled", enabled=True)    # the page levels out without focus
        page.call("Page.enable")
        page.call("Page.addScriptToEvaluateOnNewDocument", source=FAKE_PAD)
        page.call("Page.navigate", url=f"http://localhost:{GS}/?host=127.0.0.1:{WSP}&lang=en")
        time.sleep(1.5)
        missing = page.js("const en = Object.keys(TEXT.en), de = Object.keys(TEXT.de);"
                          "return en.filter(k => !de.includes(k)).concat(de.filter(k => !en.includes(k)));")
        if missing:
            print("warning: texts missing in one language:", ", ".join(missing))
        scenes(page, raw)
        errors = page.js("return window.__errors;")
        if errors:
            print("warning: errors on the page:", errors)
    finally:
        for p in reversed(procs):
            p.terminate()
            p.wait()

    magick = shutil.which("magick")
    for name in sorted(os.listdir(raw)):
        src, dst = os.path.join(raw, name), os.path.join(OUT, name)
        if magick:
            subprocess.run([magick, src, "-strip", "-dither", "None", "-colors", "256", f"PNG8:{dst}"], check=True)
        else:
            shutil.copyfile(src, dst)
        print(f"{os.path.relpath(dst, REPO)}: {os.path.getsize(dst) // 1024} KB")
    shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
