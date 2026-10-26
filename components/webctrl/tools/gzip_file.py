#!/usr/bin/env python3
# Compress a file with gzip for embedding into the firmware.
# mtime=0 keeps the output identical between builds.
#
# Copyright 2026 Philipp Weber
# SPDX-License-Identifier: GPL-3.0-only
import gzip
import sys

if len(sys.argv) != 3:
    sys.exit("usage: gzip_file.py <input> <output.gz>")

with open(sys.argv[1], "rb") as src:
    data = src.read()

with open(sys.argv[2], "wb") as dst:
    dst.write(gzip.compress(data, compresslevel=9, mtime=0))
