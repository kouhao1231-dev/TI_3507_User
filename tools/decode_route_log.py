#!/usr/bin/env python3
"""Decode deferred DCAR four-segment route records."""

from __future__ import annotations

import argparse
import json
import re
import struct
import sys
from pathlib import Path
from typing import Iterable, Optional, TextIO


RECORD_SIZE = 44
HEX_SIZE = RECORD_SIZE * 2
EVENT_COUNT = 4
EVENT_OFFSET = 19
EVENT_SIZE = 6


def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _i16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def _u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def decode_record(hex_text: str) -> dict:
    payload = hex_text.strip()
    if len(payload) != HEX_SIZE:
        raise ValueError(f"record must contain exactly {HEX_SIZE} hex characters")
    try:
        data = bytes.fromhex(payload)
    except ValueError as exc:
        raise ValueError("record contains non-hex characters") from exc
    if len(data) != RECORD_SIZE:
        raise ValueError(f"record must decode to exactly {RECORD_SIZE} bytes")
    expected_crc = crc8(data[:43])
    if data[43] != expected_crc:
        raise ValueError(
            f"CRC mismatch: received {data[43]:02X}, expected {expected_crc:02X}"
        )
    if data[0] != 1:
        raise ValueError(f"unsupported compact record version {data[0]}")
    if data[18] > EVENT_COUNT:
        raise ValueError(f"invalid event count {data[18]}")

    flags = data[2]
    phase_ticks = [_u16(data, 12), _u16(data, 14), _u16(data, 16)]
    events = []
    for event_index in range(EVENT_COUNT):
        offset = EVENT_OFFSET + event_index * EVENT_SIZE
        events.append(
            {
                "x_mm": _i16(data, offset),
                "y_mm": _i16(data, offset + 2),
                "yaw_mrad": _i16(data, offset + 4),
            }
        )

    return {
        "version": data[0],
        "run": data[1],
        "direction": "R" if flags & 0x01 else "L",
        "gray_valid": bool(flags & 0x02),
        "plan_valid": bool(flags & 0x04),
        "status": struct.unpack_from("<b", data, 3)[0],
        "gray_mask": f"{data[4]:02X}",
        "correction_half_cm": struct.unpack_from("<b", data, 5)[0],
        "turn_in_mrad": _i16(data, 6),
        "straight_mm": _u16(data, 8),
        "turn_out_mrad": _i16(data, 10),
        "phase_ticks": phase_ticks,
        "phase_ms": [ticks * 8 for ticks in phase_ticks],
        "event_count": data[18],
        "events": events[: data[18]],
        "crc": f"{data[43]:02X}",
    }


def extract_dc_payload(line: str) -> Optional[str]:
    marker = "DC,"
    marker_index = line.find(marker)
    if marker_index < 0:
        return None
    text = line[marker_index + len(marker) :]
    match = re.match(r"[0-9A-Fa-f]+", text)
    return match.group(0) if match is not None else ""


def _decode_lines(lines: Iterable[str], source: str, output: TextIO) -> int:
    failed = False
    for line_number, line in enumerate(lines, start=1):
        payload = extract_dc_payload(line)
        if payload is None:
            continue
        try:
            record = decode_record(payload)
        except ValueError as exc:
            print(f"{source}:{line_number}: {exc}", file=sys.stderr)
            failed = True
            continue
        print(
            json.dumps(record, ensure_ascii=False, separators=(",", ":")),
            file=output,
        )
    return 1 if failed else 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Decode DC,<88 hex> records emitted after a route run."
    )
    parser.add_argument(
        "files",
        nargs="*",
        type=Path,
        help="serial log files; read standard input when omitted",
    )
    args = parser.parse_args()

    result = 0
    if not args.files:
        return _decode_lines(sys.stdin, "<stdin>", sys.stdout)
    for path in args.files:
        try:
            with path.open("r", encoding="utf-8", errors="replace") as handle:
                result |= _decode_lines(handle, str(path), sys.stdout)
        except OSError as exc:
            print(f"{path}: {exc}", file=sys.stderr)
            result = 1
    return result


if __name__ == "__main__":
    raise SystemExit(main())
