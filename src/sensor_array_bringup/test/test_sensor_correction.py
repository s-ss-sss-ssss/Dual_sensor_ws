import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from sensor_array_bringup.sensor_correction import AffineSensorCorrection, DeviceIdentityGate


class SensorCorrectionTests(unittest.TestCase):
    def load(self, sensors, scale=1.0, other_family=None):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'board' / 'ak' / 'affine.json'
            path.parent.mkdir(parents=True)
            path.write_text(json.dumps({'sensors': sensors}))
            if other_family is not None:
                other = path.parent.parent / 'tmag' / 'affine.json'
                other.parent.mkdir()
                other.write_text(json.dumps({'sensors': other_family}))
            return AffineSensorCorrection(directory, 'board', scale)

    def entry(self):
        return {'sensor_id': 1, 'D_i': [[1, 2, 0], [0, 3, 0], [0, 0, 4]],
                'e_i': [10, -1, 2]}

    def test_legacy_formula_and_scale_before_bias(self):
        model = self.load([self.entry()], 0.5)
        np.testing.assert_allclose(model.apply('ak', 1, [2, 4, 6]), [15.5, 5.6, 15.2])

    def test_same_id_different_families(self):
        model = self.load([self.entry()], other_family=[dict(self.entry(), e_i=[0, 0, 0])])
        np.testing.assert_allclose(model.apply('ak', 1, [1, 1, 1]), [13.3, 2.3, 6.4])
        np.testing.assert_allclose(model.apply('tmag', 1, [1, 1, 1]), [3, 3, 4])
        self.assertEqual(set(model.families), {'ak', 'tmag'})
        self.assertEqual(set(model.parameters), {('ak', 1), ('tmag', 1)})

    def test_ak_signed_counts_to_microtesla(self):
        identity = dict(self.entry(), D_i=np.eye(3).tolist(), e_i=[0, 0, 0])
        model = self.load([identity], other_family=[identity])
        for raw in ([0, -1, 1], [-32768, 32767, -100]):
            with self.subTest(raw=raw):
                np.testing.assert_allclose(model.apply('ak', 1, raw), np.array(raw) * 1.1)
                np.testing.assert_allclose(model.apply('tmag', 1, raw), raw)

    def test_unknown_and_missing_ids_or_families_skipped(self):
        model = self.load([self.entry()])
        self.assertIsNone(model.apply('ak', 255, [1, 2, 3]))
        self.assertIsNone(model.apply('ak', 2, [1, 2, 3]))
        self.assertIsNone(model.apply('tmag', 1, [1, 2, 3]))
        self.assertIsNone(model.apply('AK', 1, [1, 2, 3]))

    def test_directory_errors_and_device_isolation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(ValueError, 'does not exist'):
                AffineSensorCorrection(root, 'board')
            (root / 'board').mkdir()
            with self.assertRaisesRegex(ValueError, 'no family'):
                AffineSensorCorrection(root, 'board')
            (root / 'board' / 'ak').mkdir()
            with self.assertRaisesRegex(ValueError, 'missing affine.json'):
                AffineSensorCorrection(root, 'board')
            path = root / 'board' / 'ak' / 'affine.json'
            path.write_text(json.dumps({'sensors': [self.entry()]}))
            with self.assertRaisesRegex(ValueError, 'does not exist'):
                AffineSensorCorrection(root, 'other_board')
            for bad in ('../board', 'board/ak', '', None):
                with self.subTest(device_id=bad), self.assertRaises(ValueError):
                    AffineSensorCorrection(root, bad)

    def test_invalid_parameters(self):
        bad_entries = [dict(self.entry(), sensor_id=255),
                       dict(self.entry(), D_i=[[1, 2, 3]]),
                       dict(self.entry(), e_i=[[1], [2], [3]]),
                       dict(self.entry(), e_i=[float('nan'), 0, 0])]
        for entry in bad_entries:
            with self.subTest(entry=entry), self.assertRaises(ValueError):
                self.load([entry])
        with self.assertRaises(ValueError):
            self.load([self.entry(), self.entry()])
        with self.assertRaises(ValueError):
            self.load([])
        with self.assertRaises(ValueError):
            self.load([self.entry()], other_family=[])

    def test_identity_wait_verify_and_latch(self):
        gate = DeviceIdentityGate('4294967295')
        self.assertFalse(gate.accept(False))
        self.assertTrue(gate.accept(True, 0xFFFFFFFF))
        self.assertTrue(gate.accept(False))
        self.assertFalse(gate.accept(True, 3))
        self.assertFalse(gate.accept(False))
        self.assertFalse(gate.accept(True, 0xFFFFFFFF))

    def test_identity_check_disabled(self):
        gate = DeviceIdentityGate(42, False)
        self.assertTrue(gate.accept(False))
        self.assertTrue(gate.accept(True, 3))
        self.assertFalse(gate.verified)

    def test_invalid_identity_config(self):
        for value in (True, -1, 0x100000000, 'bad'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                DeviceIdentityGate(value)


if __name__ == '__main__':
    unittest.main()
