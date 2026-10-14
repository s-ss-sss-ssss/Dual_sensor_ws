import contextlib
import io
import signal
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch, Mock
import xml.etree.ElementTree as ET

import yaml

from sensor_array_bringup.configuration import load_configuration
from sensor_array_bringup.launcher import (
    configured_main, launch_arguments, launch_command, main, run_launch,
)

PACKAGE = Path(__file__).resolve().parents[1]


class LaunchTests(unittest.TestCase):
    def configuration(self):
        return load_configuration(PACKAGE / 'config/single_array.yaml')

    def test_fixed_command_and_uint32_serialization(self):
        config = self.configuration()
        config.update(firmware_device_id=0xFFFFFFFF, check_device_id=False,
                      expected_ak=[[1, 255]], expected_tmag=[[2, 53]], expect_icm=True)
        command = launch_command(config)
        self.assertEqual(command[:3], ['roslaunch', 'sensor_array_bringup', 'device.launch'])
        args = dict(argument.split(':=', 1) for argument in command[3:])
        self.assertEqual(args['firmware_device_id'], '4294967295')
        self.assertEqual(args['check_device_id'], 'false')
        self.assertEqual(args['expect_icm'], 'true')
        self.assertEqual(yaml.safe_load(args['expected_ak']), [[1, 255]])
        self.assertEqual(yaml.safe_load(args['expected_tmag']), [[2, 53]])
        self.assertEqual(args['sample_topic'] + '_corr', args['sensor_correction_sample_topic'])
        self.assertEqual(args['namespace'], '/sensor1')
        self.assertEqual(args['sensor_correction_dir'], config['sensor_correction_dir'])
        for argument, topic in (('raw_frame_topic', 'raw_frames'), ('tx_bytes_topic', 'tx_bytes'),
                                ('sample_topic', 'sample'), ('diagnostics_topic', 'diagnostics')):
            self.assertEqual(args[argument], config['topics'][topic])
            self.assertTrue(args[argument].startswith('/array_01/'))
        self.assertNotIn('sensor_correction_file', args)

    def test_argument_paths_with_spaces_are_not_shell_split(self):
        config = self.configuration()
        config['sensor_correction_dir'] = '/tmp/sensor_correction data'
        config['port'] = '/tmp/serial port'
        arguments = launch_arguments(config)
        self.assertIn('sensor_correction_dir:=/tmp/sensor_correction data', arguments)
        self.assertIn('port:=/tmp/serial port', arguments)

    def test_check_does_not_write_or_launch(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            (directory / 'corrections' / 'board').mkdir(parents=True)
            config = directory / 'board.yaml'
            config.write_text(yaml.safe_dump(dict(device_id='board', firmware_device_id=1,
                namespace='/board', port='/dev/example', sensor_correction_dir='corrections')))
            before = {path: path.read_bytes() if path.is_file() else None
                                  for path in directory.rglob('*')}
            with patch('sensor_array_bringup.launcher.subprocess.Popen') as popen:
                self.assertEqual(main(['--config', str(config), '--check']), 0)
            popen.assert_not_called()
            self.assertEqual(before, {path: path.read_bytes() if path.is_file() else None
                                                  for path in directory.rglob('*')})

    def test_main_uses_same_foreground_helper(self):
        config_path = str(PACKAGE / 'config/single_array.yaml')
        with patch('sensor_array_bringup.launcher.run_launch', return_value=7) as run:
            self.assertEqual(main(['--config', config_path]), 7)
        self.assertEqual(run.call_args.args[0], self.configuration())

    def test_configured_wrapper_strips_ros_remaps(self):
        with patch('sensor_array_bringup.launcher.main', return_value=0) as entry:
            self.assertEqual(configured_main(['--config', '/tmp/board.yaml',
                                              '__name:=wrapper', '__log:=/tmp/log']), 0)
        entry.assert_called_once_with(['--config', '/tmp/board.yaml'])

    def test_foreground_signals_and_exit_status(self):
        for status, expected in ((0, 0), (3, 3), (-signal.SIGTERM, 143)):
            with self.subTest(status=status):
                process = Mock()
                process.poll.return_value = None
                handlers = {}
                previous = {signal.SIGINT: object(), signal.SIGTERM: object()}

                def install(signum, handler):
                    handlers[signum] = handler
                    return previous[signum]

                def wait():
                    handlers[signal.SIGINT](signal.SIGINT, None)
                    handlers[signal.SIGTERM](signal.SIGTERM, None)
                    return status

                process.wait.side_effect = wait
                with patch('sensor_array_bringup.launcher.signal.signal', side_effect=install) as signals, \
                        patch('sensor_array_bringup.launcher.subprocess.Popen', return_value=process) as popen:
                    config = self.configuration()
                    self.assertEqual(run_launch(config), expected)
                popen.assert_called_once_with(launch_command(config), start_new_session=True)
                self.assertEqual([call.args[0] for call in process.send_signal.call_args_list],
                                 [signal.SIGINT, signal.SIGTERM])
                self.assertEqual(signals.call_args_list[-2].args, (signal.SIGINT, previous[signal.SIGINT]))
                self.assertEqual(signals.call_args_list[-1].args, (signal.SIGTERM, previous[signal.SIGTERM]))

    def test_missing_config_and_missing_roslaunch_report_failure(self):
        with contextlib.redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(main(['--config', '/nonexistent/board.yaml', '--check']), 1)
        self.assertIn('Bringup failed:', errors.getvalue())
        with patch('sensor_array_bringup.launcher.subprocess.Popen', side_effect=FileNotFoundError('roslaunch')), \
                contextlib.redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(main(['--config', str(PACKAGE / 'config/single_array.yaml')]), 1)
        self.assertIn('roslaunch', errors.getvalue())

    def test_removed_generation_options(self):
        for option in ('--generate-only', '--run-dir'):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main(['--config', 'board.yaml', option])
            self.assertEqual(error.exception.code, 2)

    def test_fixed_device_launch_wiring(self):
        root = ET.parse(PACKAGE / 'launch/device.launch').getroot()
        defaults = {arg.attrib['name']: arg.attrib.get('default') for arg in root.findall('arg')}
        declared = set(defaults)
        for argument, suffix in (('raw_frame_topic', 'serial_frame_bridge/raw_frames'),
                                 ('tx_bytes_topic', 'serial_frame_bridge/tx_bytes'),
                                 ('sample_topic', 'maggrad/sample'),
                                 ('diagnostics_topic', 'diagnostics')):
            self.assertEqual(defaults[argument], '/$(arg device_id)/' + suffix)
        self.assertEqual(defaults['sensor_correction_sample_topic'], '$(arg sample_topic)_corr')
        self.assertIsNone(defaults['namespace'])
        self.assertIsNone(defaults['sensor_correction_dir'])
        supplied = {arg.split(':=', 1)[0] for arg in launch_arguments(self.configuration())}
        self.assertEqual(declared, supplied)
        group = root.find('group')
        self.assertEqual(group.attrib['ns'], '$(arg namespace)')
        bridge, protocol, sensor_correction = group.findall('node')
        self.assertEqual(sensor_correction.attrib['pkg'], 'sensor_array_bringup')
        self.assertEqual([node.attrib['type'] for node in (bridge, protocol, sensor_correction)],
                         ['serial_frame_bridge_node', 'protocol_node', 'sensor_correction_node'])
        remaps = {r.attrib['from']: r.attrib['to'] for r in bridge.findall('remap')}
        self.assertEqual(remaps, {'~raw_frames': '$(arg raw_frame_topic)',
                                 '~tx_bytes': '$(arg tx_bytes_topic)',
                                 '~diagnostics': '$(arg diagnostics_topic)'})
        expected = {p.attrib['param']: p for p in protocol.findall('rosparam')}
        for key in ('expected_ak', 'expected_tmag'):
            self.assertEqual(expected[key].attrib['subst_value'], 'true')
            self.assertEqual(expected[key].text, '$(arg %s)' % key)
        params = {p.attrib['name']: p.attrib for p in sensor_correction.findall('param')}
        for key in ('device_id', 'sensor_correction_dir', 'raw_scale', 'firmware_device_id', 'check_device_id', 'diagnostics_topic'):
            self.assertEqual(params[key]['value'], '$(arg %s)' % key)
        self.assertEqual(params['firmware_device_id']['type'], 'str')
        self.assertEqual(params['sensor_correction_dir']['type'], 'str')
        self.assertNotIn('sensor_correction_file', params)
        self.assertEqual(params['input_topic']['value'], '$(arg sample_topic)')
        self.assertEqual(params['output_topic']['value'], '$(arg sensor_correction_sample_topic)')
        self.assertFalse(root.findall('.//include'))

    def test_fixed_array_launch_wrapper(self):
        root = ET.parse(PACKAGE / 'launch/array.launch').getroot()
        self.assertEqual(root.find('arg').attrib, {'name': 'config'})
        node = root.find('node')
        self.assertEqual(node.attrib['type'], 'configured_bringup')
        self.assertEqual(node.attrib['required'], 'true')
        self.assertEqual(node.attrib['name'], '$(arg launcher_name)')
        naming = root.find("arg[@name='launcher_name']").attrib['default']
        self.assertIn("anon(", naming)
        self.assertIn("arg('config')", naming)
        self.assertEqual(node.attrib['args'], '--config "$(arg config)"')


if __name__ == '__main__':
    unittest.main()
