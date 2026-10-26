#!/usr/bin/env python3
"""
Start the ESP-Drone desktop ground station.

    python3 tools/groundstation/groundstation.py
    python3 tools/groundstation/groundstation.py --drone 127.0.0.1:8081   # tools/mock_drone.py

Serves index.html on http://localhost (only to this computer) and opens it in
the browser. Join the drone's Wi-Fi (ESP-DRONE_..., password 12345678) first;
the page connects to the drone by itself and keeps retrying.

Why a local web server: browsers allow gamepads only on secure pages, and
http://localhost counts as secure while the drone's own page
(http://192.168.43.42) does not. Opening index.html directly from disk works
too in Chrome and Edge.

Python standard library only. Copyright 2026 Philipp Weber
SPDX-License-Identifier: GPL-3.0-only
"""

import argparse
import functools
import http.server
import os
import sys
import threading
import urllib.parse
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))


class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass                                   # quiet: the page has its own log


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--drone", default="192.168.43.42", help="drone address, HOST or HOST:PORT (default %(default)s)")
    p.add_argument("--port", type=int, default=8090, help="local web server port (default %(default)s)")
    p.add_argument("--lang", choices=["en", "de"], help="language of the page (default: the system's)")
    p.add_argument("--no-browser", action="store_true", help="do not open a browser")
    args = p.parse_args()

    handler = functools.partial(Handler, directory=HERE)
    try:
        server = http.server.ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    except OSError as e:
        sys.exit(f"port {args.port} is busy ({e.strerror}); try --port {args.port + 1}")
    query = {"host": args.drone}
    if args.lang:
        query["lang"] = args.lang
    url = f"http://localhost:{args.port}/?{urllib.parse.urlencode(query)}"
    print(f"Ground station: {url}")
    print("Join the drone's Wi-Fi first. Stop with Ctrl+C.")
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
