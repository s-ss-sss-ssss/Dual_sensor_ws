#!/usr/bin/env python3
"""USB CDC reader for CSV and MagGrad binary frames."""

import argparse
import re
import struct
import sys
import time

import serial
import serial.tools.list_ports

SYNC = b"\xA5\x5A"
VERSION = 2
MAX_PAYLOAD = 4080
TYPE_NAMES = {
    0x01: "AK",
    0x02: "TMAG",
    0x03: "ICM",
    0x04: "PHASE",
    0xE0: "ERR",
    0xF0: "STATS",
}


def find_port():
    ports = list(serial.tools.list_ports.comports())
    print("可用串口:")
    for p in ports:
        print(f"  {p.device}: {p.description}")

    for p in ports:
        if "usb" in p.device.lower() or "modem" in p.device.lower():
            return p.device

    return None


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class RoundParser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.dropped = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        rounds = []
        while True:
            start = self.buffer.find(SYNC)
            if start < 0:
                keep = 1 if self.buffer.endswith(SYNC[:1]) else 0
                self.dropped += len(self.buffer) - keep
                del self.buffer[:len(self.buffer) - keep]
                break
            self.dropped += start
            del self.buffer[:start]
            if len(self.buffer) < 14:
                break
            version, frame_type = self.buffer[2:4]
            payload_len = struct.unpack_from("<H", self.buffer, 12)[0]
            if version != VERSION or frame_type != 0x10 or payload_len > MAX_PAYLOAD:
                self.dropped += 1
                del self.buffer[0]
                continue
            total_len = 16 + payload_len
            if len(self.buffer) < total_len:
                break
            got_crc = struct.unpack_from("<H", self.buffer, total_len - 2)[0]
            calc_crc = crc16_ccitt_false(self.buffer[2:total_len - 2])
            if got_crc != calc_crc:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            seq, tick_ms = struct.unpack_from("<II", self.buffer, 4)
            payload = bytes(self.buffer[14:total_len - 2])
            records = []
            pos = 0
            while pos + 3 <= len(payload):
                kind, length = struct.unpack_from("<BH", payload, pos)
                pos += 3
                if pos + length > len(payload):
                    break
                records.append((kind, payload[pos:pos + length]))
                pos += length
            if pos == len(payload):
                rounds.append((seq, tick_ms, records))
            else:
                self.dropped += total_len
            del self.buffer[:total_len]
        return rounds


def parse_binary_frames(raw):
    parser = RoundParser()
    rounds = parser.feed(raw)
    return rounds, parser.crc_errors, parser.dropped


def decode_frame(frame):
    frame_type, seq, tick_ms, payload = frame
    if frame_type in (0x01, 0x02, 0x03):
        if len(payload) < 4:
            return "INVALID", "INVALID", "short sensor record"
        offset_us = struct.unpack_from("<I", payload)[0]
        payload = payload[4:]
    else:
        offset_us = None
    when = f"round={seq} t={tick_ms} done_us={offset_us}" if offset_us is not None else f"round={seq} t={tick_ms}"
    if frame_type == 0x01 and len(payload) == 11:
        bus, mask, hx, hy, hz, status, err, dor = struct.unpack("<BBhhhBBB", payload)
        key = f"Bus{bus}_Mask{mask:02X}"
        return "AK", key, f"AK {when} {key} h=({hx},{hy},{hz}) st={status} err={err} dor={dor}"
    if frame_type == 0x02 and len(payload) == 10:
        ch, addr, x, y, z, status, flags = struct.unpack("<BBhhhBB", payload)
        key = f"CH{ch}_{addr:02X}"
        return "TMAG", key, f"TMAG {when} {key} xyz=({x},{y},{z}) st={status} flags={flags}"
    if frame_type == 0x03 and len(payload) == 14:
        ax, ay, az, gx, gy, gz, temp = struct.unpack("<hhhhhhh", payload)
        return "ICM", "ICM", f"ICM {when} acc=({ax},{ay},{az}) gyr=({gx},{gy},{gz}) temp={temp}"
    if frame_type == 0x04 and len(payload) == 12:
        ch1_us, ch2_us, ch3_us = struct.unpack("<III", payload)
        return "PHASE", "PHASE", f"PHASE {when} age_us=({ch1_us},{ch2_us},{ch3_us})"
    if frame_type == 0xE0 and len(payload) == 7:
        source, code, detail = struct.unpack("<BHI", payload)
        key = f"SRC{source}"
        return "ERR", key, f"ERR {when} src={source} code={code} detail=0x{detail:08X}"
    if frame_type == 0xF0 and len(payload) == 20:
        ak, tmag, icm, skipped, errors = struct.unpack("<IIIII", payload)
        return "STATS", "STATS", f"STATS {when} ak={ak} tmag={tmag} icm={icm} skipped={skipped} errors={errors}"
    name = TYPE_NAMES.get(frame_type, f"0x{frame_type:02X}")
    return name, name, f"{name} {when} payload_len={len(payload)}"


def read_raw(ser, duration):
    start = time.time()
    raw = bytearray()
    last_print = start
    while time.time() - start < duration:
        chunk = ser.read(max(1, ser.in_waiting or 512))
        if chunk:
            raw.extend(chunk)
        if time.time() - last_print >= 1:
            print(f"  已接收 {len(raw)} bytes...")
            last_print = time.time()
    return bytes(raw), time.time() - start


def analyze_binary(raw, elapsed):
    frames, crc_errors, dropped = parse_binary_frames(raw)
    counts = {}
    sensors = {}
    seq_jumps = 0
    last_seq = None
    samples = []

    for seq, tick_ms, records in frames:
        for kind, payload in records:
            name, key, text = decode_frame((kind, seq, tick_ms, payload))
            counts[name] = counts.get(name, 0) + 1
            sensors.setdefault(name, {})
            sensors[name][key] = sensors[name].get(key, 0) + 1
            if len(samples) < 8:
                samples.append(text)
        if last_seq is not None and seq != ((last_seq + 1) & 0xFFFFFFFF):
            seq_jumps += 1
        last_seq = seq

    print("\n" + "=" * 60)
    print("Binary 分析结果")
    print("=" * 60)
    print(f"总 bytes: {len(raw)}")
    print(f"有效整轮帧: {len(frames)} ({len(frames) / elapsed:.2f} rounds/s)")
    print(f"CRC 错误: {crc_errors}")
    print(f"跳过非二进制 bytes: {dropped}")
    print(f"seq 跳变: {seq_jumps}")
    if samples:
        print("\n前几帧:")
        for sample in samples:
            print(f"  {sample}")

    print("\n--- 帧率统计 ---")
    for name in ["AK", "TMAG", "ICM", "PHASE", "ERR", "STATS"]:
        count = counts.get(name, 0)
        if count == 0:
            continue
        print(f"\n--- {name} ---")
        print(f"帧数: {count}")
        print(f"帧率: {count / elapsed:.2f} frames/s")
        sensor_count = len(sensors.get(name, {}))
        if sensor_count > 0 and name in ("AK", "TMAG"):
            print(f"传感器数: {sensor_count}")
            print(f"整轮频率: {count / sensor_count / elapsed:.2f} Hz")
            for key, value in sorted(sensors[name].items()):
                print(f"  {key}: {value}")
    print("=" * 60)


def analyze_csv_text(text, elapsed):
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    print("\n" + "=" * 60)
    print("CSV 分析结果")
    print("=" * 60)
    if not lines:
        print("未接收到 CSV 数据")
        return
    print(f"总行数: {len(lines)}")

    patterns = {
        "AK": re.compile(r"^AK,(\d+),0x([0-9a-fA-F]+),(-?\d+),(-?\d+),(-?\d+),\d+,\d+,\d+$"),
        "TMAG": re.compile(r"^TMAG,(\d+),0x([0-9a-fA-F]+),(-?\d+),(-?\d+),(-?\d+),\d+,\d+$"),
        "ICM": re.compile(r"^ICM,-?\d+,-?\d+,-?\d+,-?\d+,-?\d+,-?\d+,-?\d+$"),
        "PHASE": re.compile(r"^PHASE,(\d+),(\d+),(\d+)$"),
    }
    sensor_lines = {k: [] for k in patterns}
    for line in lines:
        for name, pattern in patterns.items():
            if pattern.match(line):
                sensor_lines[name].append(line)
                break

    for name, lines_list in sensor_lines.items():
        if not lines_list:
            continue
        print(f"\n--- {name} ---")
        print(f"读取频率: {len(lines_list) / elapsed:.1f} lines/s")
        for line in lines_list[:3]:
            print(f"  {line}")
        unique = set()
        for line in lines_list:
            m = patterns[name].match(line)
            if not m:
                continue
            if name == "AK":
                unique.add(f"Bus{m.group(1)}_Mask{m.group(2)}")
            elif name == "TMAG":
                unique.add(f"CH{m.group(1)}_{m.group(2)}")
            else:
                unique.add(name)
        if unique:
            print(f"传感器数: {len(unique)}")
            print(f"帧率: {len(lines_list) / len(unique) / elapsed:.2f} Hz")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--format", choices=["auto", "csv", "binary"], default="auto")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--port")
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        print("未找到可用串口")
        sys.exit(1)

    print(f"\n连接端口: {port} @ 115200")
    try:
        ser = serial.Serial(port, 115200, timeout=0.05)
        ser.reset_input_buffer()
        print(f"读取 {args.duration:.1f} 秒数据...\n")
        raw, elapsed = read_raw(ser, args.duration)
        ser.close()
    except Exception as exc:
        print(f"错误: {exc}")
        sys.exit(1)

    fmt = args.format
    if fmt == "auto":
        fmt = "binary" if SYNC in raw else "csv"
        print(f"自动识别格式: {fmt}")

    if fmt == "binary":
        analyze_binary(raw, elapsed)
    else:
        analyze_csv_text(raw.decode("utf-8", errors="ignore"), elapsed)


if __name__ == "__main__":
    main()
