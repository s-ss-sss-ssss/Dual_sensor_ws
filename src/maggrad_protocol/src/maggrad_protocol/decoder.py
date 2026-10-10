"""ROS-independent decoder for complete MagGrad v2 ROUND wire frames."""

from dataclasses import dataclass, field
import struct
from typing import Dict, List, Optional, Tuple


RECORD_LENGTHS = {0x01: 15, 0x02: 14, 0x03: 18, 0x04: 12, 0xE0: 7, 0xF0: 20}
MAX_PAYLOAD = 4080
INVALID_PHASE = 0xFFFFFFFF


class FrameError(ValueError):
    """An entire ROUND must be discarded."""


@dataclass
class Counters:
    frames: int = 0
    malformed: int = 0
    crc_errors: int = 0
    length_errors: int = 0
    version_errors: int = 0
    record_errors: int = 0
    seq_holes: int = 0
    seq_repeats: int = 0
    seq_out_of_order: int = 0
    tick_out_of_order: int = 0
    duplicate_sensors: int = 0


@dataclass
class Round:
    round_seq: int
    device_tick_ms: int
    device_tick_unwrapped_ms: int = 0
    ak: list = field(default_factory=list)
    tmag: list = field(default_factory=list)
    icm: Optional[tuple] = None
    phase: Optional[Tuple[int, int, int]] = None
    errors: list = field(default_factory=list)
    stats: Optional[Tuple[int, int, int, int, int]] = None
    complete: bool = False
    missing_sensor_mask: int = 0


class Decoder:
    """Decode one already framed message per call; state changes only on valid rounds.

    Expected identities are exact (bus, mask) AK and (ch_mask, addr) TMAG
    pairs. Bit positions are assigned AK first, then TMAG, then optional ICM.
    """

    def __init__(self, expected_ak=(), expected_tmag=(), expect_icm=False):
        self.expected_ak = self._check_pairs(expected_ak, 'expected_ak')
        self.expected_tmag = self._check_pairs(expected_tmag, 'expected_tmag')
        self.expect_icm = bool(expect_icm)
        if len(self.expected_ak) + len(self.expected_tmag) + self.expect_icm > 64:
            raise ValueError('missing_sensor_mask supports at most 64 expected sensors')
        self.counters = Counters()
        self._last_seq = None
        self._last_tick = None
        self._unwrapped_tick = None

    @staticmethod
    def _check_pairs(pairs, name):
        result = []
        for pair in pairs:
            if not isinstance(pair, (list, tuple)) or len(pair) != 2 or any(
                    type(x) is not int or x < 0 or x > 255 for x in pair):
                raise ValueError('%s entries must be [u8, u8]' % name)
            key = tuple(pair)
            if key in result:
                raise ValueError('%s has duplicate identity %r' % (name, key))
            result.append(key)
        return result

    def decode(self, data):
        """Return a Round or raise FrameError. Bad frames do not advance state."""
        try:
            return self._decode(data)
        except FrameError:
            self.counters.malformed += 1
            raise

    def _fail(self, category, reason):
        setattr(self.counters, category, getattr(self.counters, category) + 1)
        raise FrameError(reason)

    def _decode(self, data):
        if len(data) < 16 or data[:2] != b'\xa5\x5a':
            self._fail('length_errors', 'missing v2 frame header')
        if data[2] != 2 or data[3] != 0x10:
            self._fail('version_errors', 'unsupported version or frame type')
        seq, tick, size = struct.unpack_from('<IIH', data, 4)
        if size > MAX_PAYLOAD or len(data) != 16 + size:
            self._fail('length_errors', 'invalid payload or frame length')
        crc = 0xFFFF
        for byte in data[2:-2]:
            crc ^= byte << 8
            for _ in range(8):
                crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
        if crc != struct.unpack_from('<H', data, len(data) - 2)[0]:
            self._fail('crc_errors', 'CRC-16/CCITT-FALSE mismatch')

        round_data = Round(seq, tick)
        ak: Dict[Tuple[int, int], tuple] = {}
        tmag: Dict[Tuple[int, int], tuple] = {}
        duplicates = 0
        pos = 14
        end = 14 + size
        while pos < end:
            if end - pos < 3:
                self._fail('record_errors', 'truncated record header')
            kind, length = struct.unpack_from('<BH', data, pos)
            pos += 3
            if length > end - pos:
                self._fail('record_errors', 'record extends past payload')
            if kind in RECORD_LENGTHS and length != RECORD_LENGTHS[kind]:
                self._fail('record_errors', 'incorrect known record length')
            body = data[pos:pos + length]
            pos += length
            if kind == 0x01:
                item = struct.unpack('<IBBhhhBBB', body)
                key = item[1:3]
                duplicates += key in ak
                ak[key] = item
            elif kind == 0x02:
                item = struct.unpack('<IBBhhhBB', body)
                key = item[1:3]
                duplicates += key in tmag
                tmag[key] = item
            elif kind == 0x03:
                if round_data.icm is not None:
                    duplicates += 1
                round_data.icm = struct.unpack('<Ihhhhhhh', body)
            elif kind == 0x04:
                round_data.phase = struct.unpack('<III', body)
            elif kind == 0xE0:
                round_data.errors.append(struct.unpack('<BHI', body))
            elif kind == 0xF0:
                round_data.stats = struct.unpack('<IIIII', body)
            # Unknown types are skipped using their declared length.

        round_data.ak = list(ak.values())
        round_data.tmag = list(tmag.values())
        expected = ([key in ak for key in self.expected_ak] +
                    [key in tmag for key in self.expected_tmag] +
                    ([round_data.icm is not None] if self.expect_icm else []))
        round_data.missing_sensor_mask = sum(1 << bit for bit, present in enumerate(expected)
                                             if not present)
        round_data.complete = round_data.missing_sensor_mask == 0

        if self._last_seq is not None:
            delta = (seq - self._last_seq) & 0xFFFFFFFF
            if delta == 0:
                self.counters.seq_repeats += 1
            elif delta < 0x80000000:
                self.counters.seq_holes += delta - 1
                self._last_seq = seq
            else:
                self.counters.seq_out_of_order += 1
        else:
            self._last_seq = seq

        if self._last_tick is None:
            self._unwrapped_tick = tick
            self._last_tick = tick
        else:
            delta = (tick - self._last_tick) & 0xFFFFFFFF
            if delta < 0x80000000:
                self._unwrapped_tick += delta
                self._last_tick = tick
            else:
                self.counters.tick_out_of_order += 1
        round_data.device_tick_unwrapped_ms = self._unwrapped_tick
        self.counters.duplicate_sensors += duplicates
        self.counters.frames += 1
        return round_data
