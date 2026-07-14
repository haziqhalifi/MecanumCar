#!/usr/bin/env python3
"""Regenerate the embedded dashboard headers from the dashboard/*.html sources.

Run from the esp32-wifi-bridge directory:
    python3 tools/gen_dashboard_header.py
The ESP32 firmware serves these gzip-compressed pages from PROGMEM:
    dashboard/dashboard-wifi.html -> src/dashboard_html.h  (served at "/")
    dashboard/showcase.html       -> src/showcase_html.h   (served at "/showcase")
"""
import gzip, os

HERE = os.path.dirname(os.path.abspath(__file__))
DASH = os.path.normpath(os.path.join(HERE, "..", "..", "dashboard"))
SRC  = os.path.normpath(os.path.join(HERE, "..", "src"))

# (source html, output header, C symbol prefix)
PAGES = [
    ("dashboard-wifi.html", "dashboard_html.h", "DASHBOARD_HTML"),
    ("showcase.html",       "showcase_html.h",  "SHOWCASE_HTML"),
]

for src_name, out_name, prefix in PAGES:
    src_path = os.path.join(DASH, src_name)
    out_path = os.path.join(SRC, out_name)
    html = open(src_path, "rb").read()
    gz = gzip.compress(html, 9)
    with open(out_path, "w") as f:
        f.write(f"// AUTO-GENERATED from dashboard/{src_name} — do not edit by hand.\n")
        f.write("// Regenerate: python3 tools/gen_dashboard_header.py\n")
        f.write("#pragma once\n")
        f.write("#include <pgmspace.h>\n\n")
        f.write(f"// gzip-compressed page: {len(html)} bytes raw -> {len(gz)} bytes gz\n")
        f.write(f"const unsigned int {prefix}_GZ_LEN = {len(gz)};\n")
        f.write(f"const uint8_t {prefix}_GZ[] PROGMEM = {{\n")
        for i in range(0, len(gz), 16):
            chunk = gz[i:i+16]
            f.write("  " + ",".join(f"0x{b:02x}" for b in chunk) + ",\n")
        f.write("};\n")
    print(f"wrote {out_path}: raw={len(html)} gz={len(gz)}")
