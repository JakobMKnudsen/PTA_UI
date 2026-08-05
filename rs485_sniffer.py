#!/usr/bin/env python3
"""Simple RS485 listener for identifying initialization and read commands.

Features:
- Starts with common RS485 serial settings.
- Optional scan mode to test common profiles and pick the most active one.
- Splits incoming bytes into frames using an idle gap.
- Prints raw hex for each frame.
- Attempts Modbus RTU decode (address/function/CRC) and labels likely command intent.

Requires:
    pip install pyserial
"""

from __future__ import annotations

import argparse
import math
import signal
import struct
import sys
import time
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from typing import Iterable, List, Optional, Sequence, Tuple

import serial
from serial.tools import list_ports


# Common RS485 defaults seen in industrial devices.
COMMON_PROFILES: Sequence[Tuple[int, str, float]] = (
    (9600, "N", 1),
    (19200, "N", 1),
    (38400, "N", 1),
    (57600, "N", 1),
    (115200, "N", 1),
    (9600, "E", 1),
    (19200, "E", 1),
    (9600, "O", 1),
)


def crc16_modbus(data: bytes) -> int:
    """Return Modbus CRC16 over data bytes."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


@dataclass
class FrameInfo:
    raw: bytes
    timestamp: float
    modbus_valid_crc: bool
    modbus_address: Optional[int]
    modbus_function: Optional[int]
    inferred_type: str


@dataclass
class Pattern49Record:
    addr: int
    query_class: int
    query_code: int
    query_checksum: int
    response_type: int
    response_payload: bytes
    response_tail: bytes


def infer_command_type(function_code: Optional[int]) -> str:
    """Heuristic labels: read, write/init, or unknown."""
    if function_code is None:
        return "unknown"

    read_fcs = {0x01, 0x02, 0x03, 0x04}
    write_fcs = {0x05, 0x06, 0x0F, 0x10, 0x16, 0x17}

    if function_code in read_fcs:
        return "read"
    if function_code in write_fcs:
        return "write_or_init"
    if function_code >= 0x80:
        return "exception"
    return "other"


def parse_modbus(frame: bytes) -> Tuple[bool, Optional[int], Optional[int], str]:
    """Try to parse frame as Modbus RTU. Returns validity/address/function/type."""
    if len(frame) < 4:
        return False, None, None, "unknown"

    payload, rx_crc_le = frame[:-2], frame[-2:]
    expected_crc = crc16_modbus(payload)
    expected_crc_le = expected_crc.to_bytes(2, byteorder="little")
    valid_crc = rx_crc_le == expected_crc_le

    address = frame[0] if len(frame) >= 1 else None
    function_code = frame[1] if len(frame) >= 2 else None
    inferred = infer_command_type(function_code)
    return valid_crc, address, function_code, inferred


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def parse_int_auto(value: str) -> int:
    """Parse decimal or 0x-prefixed integer from CLI."""
    return int(value, 0)


def decode_payload_candidates(payload: bytes) -> dict:
    """Return candidate numeric decodes for a 4-byte payload."""
    if len(payload) != 4:
        return {}

    out = {
        "u32_be": int.from_bytes(payload, byteorder="big", signed=False),
        "s32_be": int.from_bytes(payload, byteorder="big", signed=True),
        "u32_le": int.from_bytes(payload, byteorder="little", signed=False),
        "s32_le": int.from_bytes(payload, byteorder="little", signed=True),
        "u16_be_hi": int.from_bytes(payload[0:2], byteorder="big", signed=False),
        "u16_be_lo": int.from_bytes(payload[2:4], byteorder="big", signed=False),
        "u16_le_hi": int.from_bytes(payload[0:2], byteorder="little", signed=False),
        "u16_le_lo": int.from_bytes(payload[2:4], byteorder="little", signed=False),
    }

    f_be = struct.unpack(">f", payload)[0]
    f_le = struct.unpack("<f", payload)[0]
    out["f32_be"] = f_be if math.isfinite(f_be) else float("nan")
    out["f32_le"] = f_le if math.isfinite(f_le) else float("nan")
    return out


def decode_pattern49_value_candidates(rec: Pattern49Record) -> dict:
        """Decode value candidates that include response_type as data MSB.

        Observed traffic often looks like:
            [addr 49 d0 d1 d2 d3 status t0 t1]
        where d0 is currently labeled response_type.
        """
        val_bytes = bytes([rec.response_type]) + rec.response_payload[:3]
        out = {
                "f32_be_full": struct.unpack(">f", val_bytes)[0],
                "u32_be_full": int.from_bytes(val_bytes, byteorder="big", signed=False),
                "s32_be_full": int.from_bytes(val_bytes, byteorder="big", signed=True),
                "status_byte": rec.response_payload[3],
        }
        return out


def fmt_float(value: float) -> str:
    if math.isnan(value):
        return "nan"
    return f"{value:.6g}"


def is_likely_float_status_record(rec: Pattern49Record) -> bool:
    """Heuristic: status is typically the 4th payload byte and is often 0x00."""
    return len(rec.response_payload) == 4 and rec.response_payload[3] == 0x00


def parse_pattern49_records(frame: bytes) -> List[Pattern49Record]:
    """Heuristic parser for repeating records seen as:

    [addr 49 qclass qcode qchk addr 49 rtype p0 p1 p2 p3 t0 t1]
    """
    records: List[Pattern49Record] = []
    i = 0
    while i + 14 <= len(frame):
        chunk = frame[i : i + 14]
        if chunk[0] == chunk[5] and chunk[1] == 0x49 and chunk[6] == 0x49:
            records.append(
                Pattern49Record(
                    addr=chunk[0],
                    query_class=chunk[2],
                    query_code=chunk[3],
                    query_checksum=chunk[4],
                    response_type=chunk[7],
                    response_payload=bytes(chunk[8:12]),
                    response_tail=bytes(chunk[12:14]),
                )
            )
            i += 14
            continue
        i += 1
    return records


def read_frames(
    ser: serial.Serial,
    idle_gap_s: float,
    max_frame_s: Optional[float] = None,
    max_frame_bytes: Optional[int] = None,
    stop_after_s: Optional[float] = None,
) -> Iterable[FrameInfo]:
    """Read bytes and emit frames by idle gap, age limit, or byte count."""
    buffer = bytearray()
    frame_start_time: Optional[float] = None
    last_rx_time: Optional[float] = None
    start = time.time()

    while True:
        if stop_after_s is not None and (time.time() - start) >= stop_after_s:
            break

        chunk = ser.read(256)
        now = time.time()
        if chunk:
            if not buffer:
                frame_start_time = now
            buffer.extend(chunk)
            last_rx_time = now

            # Force split if a frame grows too long in bytes.
            if max_frame_bytes is not None and len(buffer) >= max_frame_bytes:
                frame = bytes(buffer)
                buffer.clear()
                frame_start_time = None
                valid_crc, addr, fc, inferred = parse_modbus(frame)
                yield FrameInfo(
                    raw=frame,
                    timestamp=now,
                    modbus_valid_crc=valid_crc,
                    modbus_address=addr,
                    modbus_function=fc,
                    inferred_type=inferred,
                )
            continue

        # Timeout with no data. Split if frame is too old even without a long idle gap.
        if (
            buffer
            and frame_start_time is not None
            and max_frame_s is not None
            and (now - frame_start_time) >= max_frame_s
        ):
            frame = bytes(buffer)
            buffer.clear()
            frame_start_time = None
            valid_crc, addr, fc, inferred = parse_modbus(frame)
            yield FrameInfo(
                raw=frame,
                timestamp=now,
                modbus_valid_crc=valid_crc,
                modbus_address=addr,
                modbus_function=fc,
                inferred_type=inferred,
            )
            continue

        # Timeout with no data. If buffer exists and line has been idle long enough,
        # flush a frame.
        if buffer and last_rx_time is not None and (now - last_rx_time) >= idle_gap_s:
            frame = bytes(buffer)
            buffer.clear()
            frame_start_time = None
            valid_crc, addr, fc, inferred = parse_modbus(frame)
            yield FrameInfo(
                raw=frame,
                timestamp=now,
                modbus_valid_crc=valid_crc,
                modbus_address=addr,
                modbus_function=fc,
                inferred_type=inferred,
            )

    if buffer:
        now = time.time()
        valid_crc, addr, fc, inferred = parse_modbus(bytes(buffer))
        yield FrameInfo(
            raw=bytes(buffer),
            timestamp=now,
            modbus_valid_crc=valid_crc,
            modbus_address=addr,
            modbus_function=fc,
            inferred_type=inferred,
        )


def detect_port(explicit_port: Optional[str]) -> str:
    if explicit_port:
        return explicit_port

    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Provide --port COMx explicitly.")

    # Pick the first available port as a convenience default.
    return ports[0].device


def open_serial(port: str, baud: int, parity: str, stopbits: float, timeout_s: float) -> serial.Serial:
    parity_map = {
        "N": serial.PARITY_NONE,
        "E": serial.PARITY_EVEN,
        "O": serial.PARITY_ODD,
    }
    stop_map = {
        1: serial.STOPBITS_ONE,
        2: serial.STOPBITS_TWO,
    }

    if parity not in parity_map:
        raise ValueError(f"Unsupported parity: {parity}")
    if stopbits not in stop_map:
        raise ValueError("Unsupported stop bits. Use 1 or 2.")

    return serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=parity_map[parity],
        stopbits=stop_map[stopbits],
        timeout=timeout_s,
    )


def scan_profiles(port: str, idle_gap_s: float, timeout_s: float, dwell_s: float) -> Tuple[int, str, float]:
    """Test common profiles and pick the one with most traffic / valid Modbus frames."""
    best_profile = COMMON_PROFILES[0]
    best_score = -1

    print(f"[scan] Testing {len(COMMON_PROFILES)} common profiles on {port}...", flush=True)

    for baud, parity, stopbits in COMMON_PROFILES:
        try:
            with open_serial(port, baud, parity, stopbits, timeout_s) as ser:
                frame_count = 0
                valid_modbus = 0
                bytes_seen = 0
                for frame in read_frames(
                    ser,
                    idle_gap_s=idle_gap_s,
                    max_frame_s=0.05,
                    max_frame_bytes=256,
                    stop_after_s=dwell_s,
                ):
                    frame_count += 1
                    bytes_seen += len(frame.raw)
                    if frame.modbus_valid_crc:
                        valid_modbus += 1

            score = valid_modbus * 1000 + frame_count * 10 + bytes_seen
            print(
                f"[scan] {baud} 8{parity}{int(stopbits)} -> frames={frame_count}, "
                f"valid_modbus={valid_modbus}, bytes={bytes_seen}",
                flush=True,
            )

            if score > best_score:
                best_score = score
                best_profile = (baud, parity, stopbits)
        except serial.SerialException as exc:
            print(f"[scan] {baud} 8{parity}{int(stopbits)} failed: {exc}", flush=True)

    b, p, s = best_profile
    print(f"[scan] Selected profile: {b} 8{p}{int(s)}", flush=True)
    return best_profile


def print_frame(frame: FrameInfo, hex_only: bool = False) -> None:
    ts = datetime.fromtimestamp(frame.timestamp).strftime("%H:%M:%S.%f")[:-3]
    raw_hex = hexdump(frame.raw)

    if hex_only:
        print(f"[{ts}] {raw_hex}", flush=True)
        return

    if frame.modbus_valid_crc and frame.modbus_address is not None and frame.modbus_function is not None:
        print(
            f"[{ts}] MODBUS addr={frame.modbus_address} fc=0x{frame.modbus_function:02X} "
            f"type={frame.inferred_type} len={len(frame.raw)} data={raw_hex}",
            flush=True,
        )
    else:
        print(f"[{ts}] RAW len={len(frame.raw)} data={raw_hex}", flush=True)


def run_listener(args: argparse.Namespace) -> int:
    port = detect_port(args.port)

    baud = args.baud
    parity = args.parity
    stopbits = args.stopbits

    if args.scan:
        baud, parity, stopbits = scan_profiles(
            port=port,
            idle_gap_s=args.idle_gap_ms / 1000.0,
            timeout_s=args.timeout_ms / 1000.0,
            dwell_s=args.scan_dwell_s,
        )

    print(
        f"[listen] port={port}, serial={baud} 8{parity}{int(stopbits)}, "
        f"idle_gap={args.idle_gap_ms}ms, max_frame={args.max_frame_ms}ms/{args.max_frame_bytes}B",
        flush=True,
    )
    print("[listen] Press Ctrl+C to stop.\n", flush=True)

    counts = Counter()
    byte_hist = Counter()
    modbus_by_addr_fc = Counter()
    pattern49_query = Counter()
    pattern49_query_by_addr_cmd = Counter()
    pattern49_response = Counter()
    pattern49_addrs = Counter()
    pattern49_records_total = 0
    selected_records = 0
    selected_minmax = {}
    live_counter = 0
    pressure_live_counter = 0
    pressure_baseline = {}
    pressure_baseline_sum = Counter()
    pressure_baseline_count = Counter()
    seen_addr_cmd = set()
    total_bytes = 0

    def _sigint_handler(_sig, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _sigint_handler)

    try:
        with open_serial(
            port=port,
            baud=baud,
            parity=parity,
            stopbits=stopbits,
            timeout_s=args.timeout_ms / 1000.0,
        ) as ser:
            for frame in read_frames(
                ser,
                idle_gap_s=args.idle_gap_ms / 1000.0,
                max_frame_s=args.max_frame_ms / 1000.0,
                max_frame_bytes=args.max_frame_bytes,
            ):
                print_frame(frame, hex_only=args.hex_only)
                total_bytes += len(frame.raw)
                byte_hist.update(frame.raw)
                key = frame.inferred_type if frame.modbus_valid_crc else "raw"
                counts[key] += 1
                if frame.modbus_valid_crc and frame.modbus_address is not None and frame.modbus_function is not None:
                    modbus_by_addr_fc[(frame.modbus_address, frame.modbus_function)] += 1

                # Non-Modbus heuristic decode for observed addr/0x49 streaming pattern.
                records = parse_pattern49_records(frame.raw)
                if records:
                    pattern49_records_total += len(records)
                    for rec in records:
                        pattern49_addrs[rec.addr] += 1
                        pattern49_query[(rec.query_class, rec.query_code)] += 1
                        pattern49_query_by_addr_cmd[(rec.addr, rec.query_code)] += 1
                        pattern49_response[rec.response_type] += 1

                        addr_cmd = (rec.addr, rec.query_code)
                        if args.pattern49_show_events and addr_cmd not in seen_addr_cmd:
                            seen_addr_cmd.add(addr_cmd)
                            ts = datetime.fromtimestamp(frame.timestamp).strftime("%H:%M:%S.%f")[:-3]
                            print(
                                f"[{ts}] EVENT new_query addr=0x{rec.addr:02X} cmd=0x{rec.query_code:02X} "
                                f"qclass=0x{rec.query_class:02X} qchk=0x{rec.query_checksum:02X}",
                                flush=True,
                            )

                        addr_match = args.pattern49_addr is None or rec.addr == args.pattern49_addr
                        cmd_match = args.pattern49_cmd is None or rec.query_code == args.pattern49_cmd
                        rtype_match = args.pattern49_rtype is None or rec.response_type == args.pattern49_rtype
                        if not (addr_match and cmd_match and rtype_match):
                            continue

                        decoded = decode_payload_candidates(rec.response_payload)
                        decoded_full = decode_pattern49_value_candidates(rec)
                        selected_records += 1

                        for key, val in decoded.items():
                            if isinstance(val, float) and math.isnan(val):
                                continue
                            if key not in selected_minmax:
                                selected_minmax[key] = [val, val]
                            else:
                                selected_minmax[key][0] = min(selected_minmax[key][0], val)
                                selected_minmax[key][1] = max(selected_minmax[key][1], val)

                        for key, val in decoded_full.items():
                            if isinstance(val, float) and math.isnan(val):
                                continue
                            if key not in selected_minmax:
                                selected_minmax[key] = [val, val]
                            else:
                                selected_minmax[key][0] = min(selected_minmax[key][0], val)
                                selected_minmax[key][1] = max(selected_minmax[key][1], val)

                        if args.pattern49_live:
                            live_counter += 1
                            if live_counter % max(1, args.pattern49_live_every) == 0:
                                ts = datetime.fromtimestamp(frame.timestamp).strftime("%H:%M:%S.%f")[:-3]
                                payload_hex = hexdump(rec.response_payload)
                                tail_hex = hexdump(rec.response_tail)
                                full_val_hex = hexdump(bytes([rec.response_type]) + rec.response_payload[:3])
                                print(
                                    f"[{ts}] P49 addr=0x{rec.addr:02X} cmd=0x{rec.query_code:02X} "
                                    f"data4={full_val_hex} status=0x{decoded_full['status_byte']:02X} "
                                    f"payload={payload_hex} tail={tail_hex} "
                                    f"f32be_full={fmt_float(decoded_full.get('f32_be_full', float('nan')))} "
                                    f"u32be_full={decoded_full.get('u32_be_full')}",
                                    flush=True,
                                )

                        if args.pressure_live and is_likely_float_status_record(rec):
                            value = decoded_full["f32_be_full"]
                            key = (rec.addr, rec.query_code)

                            if key not in pressure_baseline and pressure_baseline_count[key] < args.tare_samples:
                                pressure_baseline_sum[key] += value
                                pressure_baseline_count[key] += 1
                                if pressure_baseline_count[key] == args.tare_samples:
                                    pressure_baseline[key] = pressure_baseline_sum[key] / args.tare_samples
                                    print(
                                        f"[tare] addr=0x{rec.addr:02X} cmd=0x{rec.query_code:02X} "
                                        f"baseline={pressure_baseline[key]:.6f} from {args.tare_samples} samples",
                                        flush=True,
                                    )

                            if key in pressure_baseline:
                                pressure_live_counter += 1
                                if pressure_live_counter % max(1, args.pressure_live_every) == 0:
                                    ts = datetime.fromtimestamp(frame.timestamp).strftime("%H:%M:%S.%f")[:-3]
                                    baseline = pressure_baseline[key]
                                    pressure_bar = value - baseline
                                    pressure_mbar = pressure_bar * 1000.0
                                    print(
                                        f"[{ts}] PRESS addr=0x{rec.addr:02X} cmd=0x{rec.query_code:02X} "
                                        f"bar={pressure_bar:.6f} mbar={pressure_mbar:.1f} raw={value:.6f} "
                                        f"base={baseline:.6f} status=0x{decoded_full['status_byte']:02X}",
                                        flush=True,
                                    )
    except KeyboardInterrupt:
        pass
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 2

    print("\nSummary:", flush=True)
    for k, v in counts.items():
        print(f"  {k}: {v}", flush=True)

    if modbus_by_addr_fc:
        print("\nLikely command map (addr/fc):", flush=True)
        for (addr, fc), seen in modbus_by_addr_fc.most_common(10):
            intent = infer_command_type(fc)
            print(f"  addr={addr} fc=0x{fc:02X} ({intent}) seen={seen}", flush=True)

    if total_bytes > 0 and counts.get("raw", 0) > 0 and not modbus_by_addr_fc:
        hi_bytes = sum(c for b, c in byte_hist.items() if b >= 0xC0)
        hi_ratio = hi_bytes / total_bytes
        common = byte_hist.most_common(8)
        common_str = " ".join(f"{b:02X}:{c}" for b, c in common)

        print("\nDiagnostics:", flush=True)
        print(f"  Total bytes: {total_bytes}", flush=True)
        print(f"  High-byte ratio (>=C0): {hi_ratio:.1%}", flush=True)
        print(f"  Top bytes: {common_str}", flush=True)

        if hi_ratio >= 0.85 and total_bytes >= 128:
            print(
                "  Hint: data appears inverted/garbled. Most common causes are swapped A/B "
                "or wrong serial format (try 19200 8E1, then 9600 8E1).",
                flush=True,
            )

    if pattern49_records_total > 0:
        print("\nHeuristic non-Modbus pattern detected (addr/0x49 records):", flush=True)
        print(f"  Records parsed: {pattern49_records_total}", flush=True)

        if pattern49_addrs:
            addrs = ", ".join(f"0x{a:02X}" for a, _ in sorted(pattern49_addrs.items()))
            print(f"  Likely device addresses: {addrs}", flush=True)

        if pattern49_query:
            print("  Likely poll/read command bytes:", flush=True)
            for (qclass, qcode), seen in pattern49_query.most_common(10):
                print(f"    49 {qclass:02X} {qcode:02X} (seen {seen})", flush=True)

        if pattern49_query_by_addr_cmd:
            print("  Per-address command map:", flush=True)
            for (addr, qcode), seen in pattern49_query_by_addr_cmd.most_common(24):
                print(f"    addr=0x{addr:02X} cmd=0x{qcode:02X} seen={seen}", flush=True)

        if pattern49_response:
            print("  Likely response type bytes:", flush=True)
            for rtype, seen in pattern49_response.most_common(10):
                print(f"    49 {rtype:02X} (seen {seen})", flush=True)

        print(
            "  Notes: this stream appears structured but not Modbus RTU. "
            "Response payload is 4 bytes after response type and may contain pressure in a custom encoding.",
            flush=True,
        )

    if selected_records > 0:
        print("\nSelected-address decode summary:", flush=True)
        if args.pattern49_addr is not None:
            print(f"  Address filter: 0x{args.pattern49_addr:02X}", flush=True)
        else:
            print("  Address filter: none", flush=True)
        print(f"  Parsed records: {selected_records}", flush=True)
        for key in [
            "f32_be_full",
            "u32_be_full",
            "s32_be_full",
            "status_byte",
            "u32_be",
            "s32_be",
            "f32_be",
            "u32_le",
            "s32_le",
            "f32_le",
            "u16_be_hi",
            "u16_be_lo",
            "u16_le_hi",
            "u16_le_lo",
        ]:
            if key not in selected_minmax:
                continue
            vmin, vmax = selected_minmax[key]
            if isinstance(vmin, float):
                print(f"  {key}: min={fmt_float(vmin)} max={fmt_float(vmax)}", flush=True)
            else:
                print(f"  {key}: min={vmin} max={vmax}", flush=True)

    if pressure_baseline:
        print("\nTare baselines:", flush=True)
        for (addr, cmd), base in sorted(pressure_baseline.items()):
            print(f"  addr=0x{addr:02X} cmd=0x{cmd:02X} baseline={base:.6f}", flush=True)

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="RS485 listener for finding init/read commands (with Modbus RTU hints)."
    )
    parser.add_argument("--port", help="Serial port (example: COM4). If omitted, uses first detected port.")
    parser.add_argument("--scan", action="store_true", help="Scan common RS485 settings before listening.")
    parser.add_argument("--scan-dwell-s", type=float, default=3.0, help="Seconds per profile during scan.")

    parser.add_argument("--baud", type=int, default=9600, help="Baud rate for direct listen mode.")
    parser.add_argument("--parity", choices=["N", "E", "O"], default="N", help="Parity.")
    parser.add_argument("--stopbits", type=float, choices=[1, 2], default=1, help="Stop bits.")

    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=30,
        help="Serial read timeout in milliseconds (smaller is more responsive).",
    )
    parser.add_argument(
        "--idle-gap-ms",
        type=int,
        default=8,
        help="Idle time in ms used to split frames.",
    )
    parser.add_argument(
        "--max-frame-ms",
        type=int,
        default=50,
        help="Force frame split after this age even if traffic is continuous.",
    )
    parser.add_argument(
        "--max-frame-bytes",
        type=int,
        default=256,
        help="Force frame split when this many bytes are buffered.",
    )
    parser.add_argument(
        "--pattern49-addr",
        type=parse_int_auto,
        help="Optional address filter for pattern49 decode (example: 24 or 0x18).",
    )
    parser.add_argument(
        "--pattern49-cmd",
        type=parse_int_auto,
        help="Optional command-code filter for pattern49 decode (example: 0x97).",
    )
    parser.add_argument(
        "--pattern49-rtype",
        type=parse_int_auto,
        help="Optional response-type filter for pattern49 decode (example: 0xBC).",
    )
    parser.add_argument(
        "--pattern49-live",
        action="store_true",
        help="Print live decoded pattern49 records (use with --pattern49-addr for one channel).",
    )
    parser.add_argument(
        "--pattern49-live-every",
        type=int,
        default=1,
        help="Print one live decode line every N selected records.",
    )
    parser.add_argument(
        "--pattern49-show-events",
        action="store_true",
        help="Print event lines when a new addr+cmd query pair first appears.",
    )
    parser.add_argument(
        "--pressure-live",
        action="store_true",
        help="Print pressure with automatic startup tare per addr+cmd (requires status byte 0x00 records).",
    )
    parser.add_argument(
        "--pressure-live-every",
        type=int,
        default=10,
        help="Print one pressure line every N tare-ready records.",
    )
    parser.add_argument(
        "--tare-samples",
        type=int,
        default=40,
        help="Startup idle samples used to compute per addr+cmd tare baseline.",
    )
    parser.add_argument("--hex-only", action="store_true", help="Print raw hex only, no protocol hints.")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return run_listener(args)


if __name__ == "__main__":
    raise SystemExit(main())
