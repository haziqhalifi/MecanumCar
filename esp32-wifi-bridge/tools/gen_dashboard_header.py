#!/usr/bin/env python3
"""Regenerate src/dashboard_html.h from dashboard/dashboard-wifi.html.

Run from the esp32-wifi-bridge directory:
    python3 tools/gen_dashboard_header.py
The ESP32 firmware serves this gzip-compressed HTML from PROGMEM.
"""
import gzip, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_HTML = os.path.normpath(os.path.join(HERE, "..", "..", "dashboard", "dashboard-wifi.html"))
OUT_H = os.path.normpath(os.path.join(HERE, "..", "src", "dashboard_html.h"))

html = open(SRC_HTML, "rb").read()
gz = gzip.compress(html, 9)
with open(OUT_H, "w") as f:
    f.write("// AUTO-GENERATED from dashboard/dashboard-wifi.html — do not edit by hand.\n")
    f.write("// Regenerate: python3 tools/gen_dashboard_header.py\n")
    f.write("#pragma once\n")
    f.write("#include <pgmspace.h>\n\n")
    f.write(f"// gzip-compressed dashboard: {len(html)} bytes raw -> {len(gz)} bytes gz\n")
    f.write(f"const unsigned int DASHBOARD_HTML_GZ_LEN = {len(gz)};\n")
    f.write("const uint8_t DASHBOARD_HTML_GZ[] PROGMEM = {\n")
    for i in range(0, len(gz), 16):
        chunk = gz[i:i+16]
        f.write("  " + ",".join(f"0x{b:02x}" for b in chunk) + ",\n")
    f.write("};\n")
print(f"wrote {OUT_H}: raw={len(html)} gz={len(gz)}")
