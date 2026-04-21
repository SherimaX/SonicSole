#!/usr/bin/env python3
"""Generate static QR code PNGs for each phone group.

The QR codes encode `http://<host>/phone/group<N>`, where <host> defaults to
`172.20.10.2:5001` (the local Wi-Fi router address used at the event). Pass a
different host as the first argument to regenerate the files for a new network,
e.g. `python3 generate_group_qr_codes.py 192.168.1.10:5001`.
"""

from __future__ import annotations

import os
import sys

import qrcode

DEFAULT_HOST = "172.20.10.2:5001"
GROUP_NUMBERS = range(1, 6)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "static", "qr"))


def build_url(host: str, group_number: int) -> str:
    return f"http://{host}/phone/group{group_number}"


def generate_qr_png(url: str, output_path: str) -> None:
    qr = qrcode.QRCode(
        version=None,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=16,
        border=2,
    )
    qr.add_data(url)
    qr.make(fit=True)
    image = qr.make_image(fill_color="black", back_color="white")
    image.save(output_path, format="PNG")


def main() -> int:
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    for group_number in GROUP_NUMBERS:
        url = build_url(host, group_number)
        output_path = os.path.join(OUTPUT_DIR, f"group{group_number}.png")
        generate_qr_png(url, output_path)
        print(f"wrote {output_path} -> {url}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
