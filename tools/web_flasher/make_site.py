#!/usr/bin/env python3
"""
Build the web flasher site (GitHub Pages) and the release notes from the
merged firmware images.

    make_site.py --version v0.1.0 --repo owner/name --bins dist --out site --notes notes.md

--bins holds the images built by .github/workflows/release.yml
(espdrone-<board>-<chip>.bin, each written at address 0x0). The site gets
index.html, one ESP Web Tools manifest per board, the images and SHA256SUMS.txt.

Copyright 2026 Philipp Weber
SPDX-License-Identifier: GPL-3.0-only
"""

import argparse
import hashlib
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

# board -> manifest name and (ESP Web Tools chip family, image) per chip
BOARDS = {
    "e1-mini": ("ESP-Drone web control for the E1 Mini",
                [("ESP32", "espdrone-e1-mini-esp32.bin")]),
    "s2-drone": ("ESP-Drone web control for the ESP32-S2-Drone",
                 [("ESP32-S2", "espdrone-s2-drone-esp32s2.bin"),
                  ("ESP32-S3", "espdrone-s2-drone-esp32s3.bin")]),
}

PICTURES = {
    "e1-mini.jpg": os.path.join(ROOT, "docs", "images", "e1-mini.jpg"),
    "s2-drone.png": os.path.join(ROOT, "docs", "_static", "espdrone_s2_v1_2_2.png"),
}


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 16), b""):
            h.update(block)
    return h.hexdigest()


def fill(template, values):
    for key, value in values.items():
        template = template.replace("{{" + key + "}}", value)
    return template


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--version", required=True)
    p.add_argument("--repo", required=True, help="owner/name on GitHub")
    p.add_argument("--bins", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--notes", help="write the release notes here")
    args = p.parse_args()

    images = sorted(f for _, builds in BOARDS.values() for _, f in builds)
    missing = [f for f in images if not os.path.isfile(os.path.join(args.bins, f))]
    if missing:
        sys.exit("missing images: " + ", ".join(missing))

    firmware = os.path.join(args.out, "firmware")
    os.makedirs(firmware, exist_ok=True)
    sums = []
    for name in images:
        shutil.copyfile(os.path.join(args.bins, name), os.path.join(firmware, name))
        sums.append(f"{sha256(os.path.join(firmware, name))}  {name}")
    with open(os.path.join(args.out, "SHA256SUMS.txt"), "w") as f:
        f.write("\n".join(sums) + "\n")

    for board, (title, builds) in BOARDS.items():
        manifest = {
            "name": title,
            "version": args.version,
            "new_install_prompt_erase": True,
            "new_install_improv_wait_time": 0,     # the firmware does not speak Improv
            "builds": [{"chipFamily": chip, "parts": [{"path": f"firmware/{image}", "offset": 0}]}
                       for chip, image in builds],
        }
        with open(os.path.join(args.out, f"manifest-{board}.json"), "w") as f:
            json.dump(manifest, f, indent=2)
            f.write("\n")

    for name, src in PICTURES.items():
        shutil.copyfile(src, os.path.join(args.out, name))

    values = {"VERSION": args.version, "REPO": args.repo, "SHA256SUMS": "\n".join(sums)}
    with open(os.path.join(HERE, "index.html")) as f:
        page = fill(f.read(), values)
    with open(os.path.join(args.out, "index.html"), "w") as f:
        f.write(page)
    with open(os.path.join(args.out, ".nojekyll"), "w"):
        pass

    if args.notes:
        with open(os.path.join(HERE, "release-notes.md")) as f:
            notes = fill(f.read(), values)
        with open(args.notes, "w") as f:
            f.write(notes)

    print(f"site in {args.out}: {len(images)} images, {len(BOARDS)} manifests")


if __name__ == "__main__":
    main()
