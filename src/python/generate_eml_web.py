#!/usr/bin/env python3
"""Generate the single-asset, deterministic EML Bridge WebContent header."""
from pathlib import Path
import gzip

root = Path(__file__).resolve().parents[1]
source = root / "html" / "eml-bridge.html"
target = root / "html" / "headers" / "web-eml-bridge.h"
data = gzip.compress(source.read_bytes(), compresslevel=9, mtime=0)
rows = []
for offset in range(0, len(data), 16):
    rows.append("  " + ", ".join(f"0x{byte:02x}" for byte in data[offset:offset + 16]) + ",")
header = f'''#pragma once
#include <stddef.h>
#include <stdint.h>
#include <pgmspace.h>
typedef struct {{ const char* path; const char* content_type; const unsigned char* data; const size_t size; }} WebAsset;
// Generated from eml-bridge.html; gzip payload {len(data)} bytes.
static const unsigned char eml_bridge_html[] PROGMEM = {{
{chr(10).join(rows)}
}};
static const WebAsset WEB_ASSETS[] PROGMEM = {{
  {{ "/index.html", "text/html", eml_bridge_html, sizeof(eml_bridge_html) }},
}};
static const size_t WEB_ASSETS_COUNT = sizeof(WEB_ASSETS) / sizeof(WEB_ASSETS[0]);
'''
target.write_text(header)
print(f"{target}: {len(data)} gzip bytes")
