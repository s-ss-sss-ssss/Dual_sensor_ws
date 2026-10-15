import copy
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch, Mock

import yaml

from sensor_array_bringup.configuration import load_configuration

PACKAGE = Path(__file__).resolve().parents[1]


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.root = self.directory / 'corrections'
        (self.root / 'board').mkdir(parents=True)
        (self.directory / 'affine.json').write_text('{}', encoding='utf-8')
        self.source = dict(device_id='board', firmware_device_id=42,
                           port='/dev/example', sensor_correction_dir='corrections')

    def load(self, source=...):
        path = self.directory / 'board.yaml'
        path.write_text(yaml.safe_dump(self.source if source is ... else source), encoding='utf-8')
        return load_configuration(path)

    def test_defaults_and_relative_path_without_ros(self):
        with patch.dict(sys.modules, {'rospkg': None, 'rospy': None, 'sensor_array_config': None}):
            result = self.load()
        self.assertEqual(result['sensor_correction_dir'], str(self.root))
        self.assertEqual(result['baud_rate'], 115200)
        self.assertEqual(result['raw_scale'], 1.0)
        self.assertEqual(result['field_range'], 1000.0)
        self.assertTrue(result['check_device_id'])
        self.assertEqual(result['device_id'], 'board')
        self.assertNotIn('topics', result)
        self.assertNotIn('namespace', result)
        self.assertEqual(result['expected_ak'], [])
        self.assertEqual(result['expected_tmag'], [])
        self.assertFalse(result['expect_icm'])

    def test_device_id_is_the_only_board_name(self):
        (self.root / 'array_02').mkdir()
        self.source['device_id'] = 'array_02'
        result = self.load()
        self.assertEqual(result['device_id'], 'array_02')
        self.assertEqual(result['sensor_correction_dir'], str(self.root))

    def test_topic_overrides_are_rejected(self):
        for topics in ({}, {'sample': '/custom/sample'}, None):
            with self.subTest(topics=topics), self.assertRaisesRegex(ValueError, 'unknown fields: topics'):
                self.load(dict(self.source, topics=topics))

    def test_absolute_correction_root(self):
        self.source['sensor_correction_dir'] = str(self.root)
        self.assertEqual(self.load()['sensor_correction_dir'], str(self.root))

    def test_missing_root_device_directory_and_file_paths_are_rejected(self):
        empty_root = self.directory / 'empty'
        empty_root.mkdir()
        file_device_root = self.directory / 'file_device'
        file_device_root.mkdir()
        (file_device_root / 'board').write_text('{}', encoding='utf-8')
        for path in ('missing', 'affine.json', 'empty', 'file_device'):
            with self.subTest(path=path), self.assertRaisesRegex(ValueError, 'not a directory'):
                self.load(dict(self.source, sensor_correction_dir=path))

    def test_uint32_and_boolean_overrides(self):
        for identity in (0, 0xFFFFFFFF):
            self.source.update(firmware_device_id=identity, check_device_id=False, raw_scale=-0.5)
            result = self.load()
            self.assertEqual(result['firmware_device_id'], identity)
            self.assertFalse(result['check_device_id'])
            self.assertEqual(result['raw_scale'], -0.5)

    def test_lazy_package_expression(self):
        self.source['sensor_correction_dir'] = '$(find sensor_correction_data)/corrections'
        module = Mock()
        module.RosPack.return_value.get_path.return_value = str(self.directory)
        with patch.dict(sys.modules, {'rospkg': module}):
            self.assertEqual(self.load()['sensor_correction_dir'], str(self.root))
        module.RosPack.return_value.get_path.assert_called_once_with('sensor_correction_data')

    def test_valid_expected_pairs(self):
        self.source.update(expected_ak=[[0, 255]], expected_tmag=[[1, 53]], expect_icm=True)
        result = self.load()
        self.assertEqual(result['expected_ak'], [[0, 255]])
        self.assertEqual(result['expected_tmag'], [[1, 53]])
        self.assertTrue(result['expect_icm'])

    def test_invalid_fields(self):
        invalid = {
            'device_id': [0, True, '', ' board', '$(env ID)', '/', '/board', 'board/',
                                      'a/b', '..', '../board', '.', 'a..b', 'a\\b', 'bad-name',
                                      '123', '~board', 'a b'],
            'firmware_device_id': [True, -1, 0x100000000, '42', 42.0],
            'check_device_id': ['false', 0, None],
            'port': ['', False, '$(env PORT)'],
            'baud_rate': [True, 0, -1, '115200'],
            'raw_scale': [True, '1.0', [], float('nan'), float('inf'), -float('inf')],
            'field_range': [True, '1000', None, 0, -1, float('nan'), float('inf')],
            'sensor_correction_dir': ['', 'absent', '$(env HOME)/corrections', True],
            'expected_ak': [None, {}, [1], [[1]], [[1, 2, 3]], [[True, 2]],
                            [[-1, 2]], [[1, 256]], [[1, 2], [1, 2]]],
            'expected_tmag': ['[]', [[1, '2']], [[1, 2], [1, 2]]],
            'expect_icm': [1, 'true', None],
        }
        for key, values in invalid.items():
            for value in values:
                with self.subTest(key=key, value=value):
                    source = dict(self.source, **{key: value})
                    with self.assertRaises(ValueError):
                        self.load(source)

    def test_expected_sensor_limit(self):
        self.source['expected_tmag'] = [[1, address] for address in range(64)]
        self.load()
        self.source['expect_icm'] = True
        with self.assertRaisesRegex(ValueError, '64'):
            self.load()
        self.source['expect_icm'] = False
        self.source['expected_ak'] = [[1, 1]]
        with self.assertRaisesRegex(ValueError, '64'):
            self.load()

    def test_required_and_unknown_fields(self):
        for key in self.source:
            source = copy.deepcopy(self.source)
            del source[key]
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'missing required'):
                self.load(source)
        for field, value in (('hardware_config', 'tmag3001'),
                             ('sensor_correction_file', 'affine.json'),
                             ('namespace', '/other'), ('topics', {})):
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'unknown fields'):
                self.load(dict(self.source, **{field: value}))
        for source in ([], None, {'schema_version': 1, 'devices': [self.source]}):
            with self.assertRaises(ValueError):
                self.load(source)

    def test_examples_are_single_board(self):
        for filename in ('single_array.yaml', 'five_arrays.example.yaml'):
            result = load_configuration(PACKAGE / 'config' / filename)
            self.assertEqual(result['firmware_device_id'], 0)
            root = Path(result['sensor_correction_dir'])
            self.assertEqual(root, PACKAGE.parent / 'sensor_array_config' / 'config')
            self.assertTrue((root / result['device_id']).is_dir())
            for family in ('ak', 'tmag'):
                self.assertTrue((root / result['device_id'] / family / 'affine.json').is_file())
            source = yaml.safe_load((PACKAGE / 'config' / filename).read_text())
            self.assertNotIn('topics', source)
            self.assertNotIn('namespace', source)


if __name__ == '__main__':
    unittest.main()
