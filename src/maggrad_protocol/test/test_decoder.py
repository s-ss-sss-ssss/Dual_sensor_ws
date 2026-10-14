import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from maggrad_protocol.decoder import Decoder, FrameError, MAX_PAYLOAD


def record(kind, body):
    return struct.pack('<BH', kind, len(body)) + body


def frame(seq, tick, *records):
    payload = b''.join(records)
    header = b'\xa5\x5a' + struct.pack('<BBIIH', 2, 0x10, seq, tick, len(payload))
    crc = 0xFFFF
    for byte in (header + payload)[2:]:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return header + payload + struct.pack('<H', crc)


AK = record(1, struct.pack('<IBBhhhBBB', 123, 2, 3, -1, 4, -5, 1, 0, 0))
TMAG = record(2, struct.pack('<IBBhhhBB', 456, 8, 0x35, -2, 7, 9, 1, 0))
ICM = record(3, struct.pack('<Ihhhhhhh', 789, 1, 2, 3, 4, 5, 6, -7))
PHASE = record(4, struct.pack('<III', 0, 0xffffffff, 42))
ERR = record(0xe0, struct.pack('<BHI', 4, 1, 99))
STATS = record(0xf0, struct.pack('<IIIII', 1, 2, 3, 4, 5))


class DecoderTests(unittest.TestCase):
    def test_all_records_and_raw_values(self):
        decoder = Decoder([(2, 3)], [(8, 0x35)], True)
        result = decoder.decode(frame(0, 12, AK, TMAG, ICM, PHASE, ERR, STATS))
        self.assertTrue(result.complete)
        self.assertEqual(result.missing_sensor_mask, 0)
        self.assertEqual(result.ak[0], (123, 2, 3, -1, 4, -5, 1, 0, 0))
        self.assertEqual(result.tmag[0][0], 456)
        self.assertEqual(result.icm[-1], -7)
        self.assertEqual(result.phase, (0, 0xffffffff, 42))
        self.assertEqual(result.errors, [(4, 1, 99)])
        self.assertEqual(result.stats, (1, 2, 3, 4, 5))

    def test_missing_unknown_and_duplicate_last_wins(self):
        decoder = Decoder([(2, 3), (1, 2)], [(8, 0x35)], True)
        replacement = record(1, struct.pack('<IBBhhhBBB', 77, 2, 3, 10, 11, 12, 0, 1, 1))
        result = decoder.decode(frame(1, 1, record(0x50, b'ignore'), AK, replacement, TMAG))
        self.assertEqual(result.ak[0][0], 77)
        self.assertEqual(result.missing_sensor_mask, 0b1010)
        self.assertFalse(result.complete)
        self.assertEqual(decoder.counters.duplicate_sensors, 1)
        self.assertIsNone(result.phase)

    def test_bad_records_drop_round_without_advancing_state(self):
        decoder = Decoder()
        decoder.decode(frame(1, 10))
        candidates = [record(1, b'\0' * 14), b'\x01\x0f',
                      b'\x01\x0f\x00' + b'\0' * 14]
        for payload in candidates:
            with self.assertRaises(FrameError):
                decoder.decode(frame(3, 20, payload))
        self.assertEqual(decoder.counters.record_errors, 3)
        result = decoder.decode(frame(2, 11))
        self.assertEqual(result.device_tick_unwrapped_us, 11)
        self.assertEqual(decoder.counters.seq_holes, 0)

    def test_wire_rejection_and_size_limit(self):
        decoder = Decoder()
        good = frame(1, 1)
        for bad in (good[:-1], good[:2] + b'\x01' + good[3:],
                    good[:4] + b'\0' + good[5:], good[:-1] + b'\0',
                    frame(1, 1, record(0x55, b'\0' * MAX_PAYLOAD))):
            with self.assertRaises(FrameError):
                decoder.decode(bad)
        self.assertEqual(decoder.counters.frames, 0)

    def test_wrap_holes_and_out_of_order(self):
        decoder = Decoder()
        first = decoder.decode(frame(0xfffffffe, 0xfffffffe))
        wrapped = decoder.decode(frame(1, 2))
        self.assertEqual(first.device_tick_unwrapped_us, 0xfffffffe)
        self.assertEqual(wrapped.device_tick_unwrapped_us, 0x100000002)
        self.assertEqual(decoder.counters.seq_holes, 2)
        decoder.decode(frame(1, 2))
        late = decoder.decode(frame(0xffffffff, 0xffffffff))
        self.assertEqual(late.device_tick_unwrapped_us, wrapped.device_tick_unwrapped_us)
        self.assertEqual(decoder.counters.seq_repeats, 1)
        self.assertEqual(decoder.counters.seq_out_of_order, 1)
        self.assertEqual(decoder.counters.tick_out_of_order, 1)

    def test_microsecond_values_and_completion_without_phase(self):
        decoder = Decoder()
        result = decoder.decode(frame(1, 1234567, AK))
        self.assertEqual(result.device_tick_us, 1234567)
        self.assertEqual(result.device_tick_unwrapped_us, 1234567)
        self.assertIsNone(result.phase)
        self.assertEqual(result.device_tick_unwrapped_us + result.ak[0][0], 1234690)
        result = decoder.decode(frame(2, 1234890))
        self.assertEqual(result.device_tick_unwrapped_us, 1234890)
        result = decoder.decode(frame(3, 1234890))
        self.assertEqual(result.device_tick_unwrapped_us, 1234890)
        self.assertEqual(decoder.counters.tick_out_of_order, 0)
        wrapped_decoder = Decoder()
        result = wrapped_decoder.decode(frame(1, 0xfffffff0, AK))
        self.assertEqual(result.device_tick_unwrapped_us + result.ak[0][0], 0x10000006b)
        result = wrapped_decoder.decode(frame(2, 0x10))
        self.assertEqual(result.device_tick_us, 0x10)
        self.assertEqual(result.device_tick_unwrapped_us, 0x100000010)

    def test_half_range_boundary(self):
        decoder = Decoder()
        decoder.decode(frame(1, 10))
        result = decoder.decode(frame(2, 0x80000009))
        self.assertEqual(result.device_tick_unwrapped_us, 0x80000009)
        result = decoder.decode(frame(3, 9))
        self.assertEqual(result.device_tick_us, 9)
        self.assertEqual(result.device_tick_unwrapped_us, 0x80000009)
        self.assertEqual(decoder.counters.tick_out_of_order, 1)

    def test_expected_sensor_validation(self):
        for kwargs in ({'expected_ak': [(1, 2), (1, 2)]},
                       {'expected_tmag': [(256, 0)]},
                       {'expected_ak': [(0, 0)] * 65}):
            with self.assertRaises(ValueError):
                Decoder(**kwargs)


if __name__ == '__main__':
    unittest.main()
