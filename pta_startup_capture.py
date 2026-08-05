#!/usr/bin/env python3
"""Capture early RS485 startup traffic to a timestamped log file.

Usage:
    python pta_startup_capture.py --port COM5

Typical workflow:
1) Run this script.
2) Power-cycle PTA and click Start in the original software.
3) Press Ctrl+C when done.
"""

from __future__ import annotations

import argparse
import struct
import sys
from datetime import datetime
from pathlib import Path

import serial

from rs485_sniffer import hexdump, read_frames


def _scan_pta_records_in_view(view: bytes, tag: str) -> tuple[list[str], int, int]:
    """Scan a specific byte view for known PTA request/response shapes."""
    hits: list[str] = []
    req_count = 0
    rsp_count = 0

    n = len(view)
    for i in range(n):
        addr = view[i]
        if addr < 1 or addr > 24:
            continue

        # Request candidate.
        if i + 5 <= n and view[i + 1] == 0x49 and view[i + 2] == 0x01:
            req_count += 1
            hits.append(
                f"{tag} REQ@{i:03d} ch={addr} cmd=0x{view[i + 3]:02X} chk=0x{view[i + 4]:02X}"
            )

        # Response candidate.
        if i + 9 <= n and view[i + 1] == 0x49:
            rsp_count += 1
            payload = bytes(view[i + 2 : i + 6])
            value = struct.unpack(">f", payload)[0]
            status = view[i + 6]
            hits.append(
                f"{tag} RSP@{i:03d} ch={addr} f32_be={value:.6g} status=0x{status:02X} tail={view[i + 7]:02X} {view[i + 8]:02X}"
            )

    return hits, req_count, rsp_count


def scan_known_pta_records(raw: bytes) -> tuple[list[str], int, int]:
    """Scan a byte stream for known PTA protocol request/response shapes.

    Known patterns from prior reverse engineering:
    - Request:  [addr, 0x49, 0x01, cmd, chk]
    - Response: [addr, 0x49, d0, d1, d2, d3, status, c0, c1]
    """
    raw_hits, raw_req, raw_rsp = _scan_pta_records_in_view(raw, "RAW")
    # Also scan with MSB cleared in case parity/mark bits are leaking into data bytes.
    masked = bytes(b & 0x7F for b in raw)
    masked_hits, masked_req, masked_rsp = _scan_pta_records_in_view(masked, "7BIT")

    return raw_hits + masked_hits, raw_req + masked_req, raw_rsp + masked_rsp


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture PTA startup RS485 frames to a log file.")
    parser.add_argument("--port", default="COM5", help="Serial port (default: COM5)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--parity", choices=["N", "E", "O"], default="N", help="Parity (default: N)")
    parser.add_argument("--stopbits", type=float, choices=[1, 2], default=1, help="Stop bits (default: 1)")
    parser.add_argument("--timeout-ms", type=int, default=30, help="Serial timeout in ms (default: 30)")
    parser.add_argument("--idle-gap-ms", type=int, default=6, help="Frame split idle gap in ms (default: 6)")
    parser.add_argument("--max-frame-ms", type=int, default=40, help="Max frame age in ms (default: 40)")
    parser.add_argument("--max-frame-bytes", type=int, default=256, help="Max frame size (default: 256)")
    parser.add_argument("--seconds", type=float, default=0.0, help="Optional auto-stop after N seconds (0 = run until Ctrl+C)")
    parser.add_argument("--output", help="Optional explicit output file path")
    return parser


def make_output_path(output_arg: str | None) -> Path:
    if output_arg:
        return Path(output_arg)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(f"startup_capture_{ts}.log")


def main() -> int:
    args = build_parser().parse_args()
    out_path = make_output_path(args.output)

    stop_after = args.seconds if args.seconds > 0 else None
    parity_map = {"N": serial.PARITY_NONE, "E": serial.PARITY_EVEN, "O": serial.PARITY_ODD}
    stopbits_value = serial.STOPBITS_ONE if args.stopbits == 1 else serial.STOPBITS_TWO

    print(f"[capture] output: {out_path}", flush=True)
    print(
        f"[capture] listening on {args.port} @ {args.baud} 8{args.parity}{int(args.stopbits)} "
        f"(Ctrl+C to stop)",
        flush=True,
    )
    print("[capture] protocol scan: looking for byte-pattern hits of known PTA request/response", flush=True)

    total_req_hits = 0
    total_rsp_hits = 0
    total_frames = 0
    total_hex49 = 0

    try:
        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=parity_map[args.parity],
            stopbits=stopbits_value,
            timeout=max(0.001, args.timeout_ms / 1000.0),
        ) as ser, out_path.open("w", encoding="utf-8") as out:
            out.write(
                f"# startup capture\n# port={args.port} baud={args.baud} parity={args.parity} stopbits={args.stopbits}\n"
            )
            out.flush()

            for i, frame in enumerate(
                read_frames(
                    ser=ser,
                    idle_gap_s=max(0.001, args.idle_gap_ms / 1000.0),
                    max_frame_s=max(0.001, args.max_frame_ms / 1000.0),
                    max_frame_bytes=max(16, args.max_frame_bytes),
                    stop_after_s=stop_after,
                ),
                start=1,
            ):
                stamp = datetime.fromtimestamp(frame.timestamp).isoformat(timespec="milliseconds")
                line = f"{i:06d} {stamp} len={len(frame.raw):03d} hex={hexdump(frame.raw)}"
                print(line, flush=True)
                out.write(line + "\n")
                total_frames += 1
                total_hex49 += frame.raw.count(0x49)

                hits, req_hits, rsp_hits = scan_known_pta_records(frame.raw)
                total_req_hits += req_hits
                total_rsp_hits += rsp_hits
                for hit in hits:
                    tagged = "         >> " + hit
                    print(tagged, flush=True)
                    out.write(tagged + "\n")

    except KeyboardInterrupt:
        print("\n[capture] stopped by user", flush=True)
    except serial.SerialException as exc:
        print(f"[capture] serial error: {exc}", file=sys.stderr, flush=True)
        print("[capture] close anything using that COM port, then retry", file=sys.stderr, flush=True)
        return 1

    summary = (
        "[capture] protocol-hit summary: "
        f"frames={total_frames} req={total_req_hits} rsp={total_rsp_hits} byte49={total_hex49}"
    )
    print(summary, flush=True)
    try:
        with out_path.open("a", encoding="utf-8") as out:
            out.write(summary + "\n")
    except OSError:
        pass

    print("[capture] complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
