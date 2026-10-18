import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml

from sensor_array_bringup.configuration import load_configuration
from sensor_array_bringup.launcher import launch_command, main

PACKAGE = Path(__file__).resolve().parents[1]


class LaunchTests(unittest.TestCase):
    def configuration(self):
        return load_configuration(PACKAGE / 'config/single_array.yaml')

    def test_command_contains_only_device_settings(self):
        config = self.configuration()
        config.update(firmware_device_id=0xFFFFFFFF, check_device_id=False,
                      expected_ak=[[1, 255]], expected_tmag=[[2, 53]], expect_icm=True)
        command = launch_command(config)
        self.assertEqual(command[:3], ['roslaunch', 'sensor_array_bringup', 'device.launch'])
        args = dict(argument.split(':=', 1) for argument in command[3:])
        self.assertEqual(args['device_id'], 'array_01')
        self.assertEqual(args['firmware_device_id'], '4294967295')
        self.assertEqual(args['check_device_id'], 'false')
        self.assertEqual(args['expect_icm'], 'true')
        self.assertEqual(yaml.safe_load(args['expected_ak']), [[1, 255]])
        self.assertEqual(yaml.safe_load(args['expected_tmag']), [[2, 53]])
        self.assertEqual(args['sensor_correction_dir'], config['sensor_correction_dir'])
        self.assertEqual(float(args['field_range']), config['field_range'])
        self.assertEqual(set(args), {
            'device_id', 'firmware_device_id', 'check_device_id', 'port', 'baud_rate',
            'sensor_correction_dir', 'raw_scale', 'expect_icm', 'expected_ak', 'expected_tmag', 'field_range'})

    def test_argument_paths_with_spaces_are_not_shell_split(self):
        config = self.configuration()
        config['sensor_correction_dir'] = '/tmp/sensor_correction data'
        config['port'] = '/tmp/serial port'
        command = launch_command(config)
        self.assertIn('sensor_correction_dir:=/tmp/sensor_correction data', command)
        self.assertIn('port:=/tmp/serial port', command)

    def test_check_does_not_write_or_launch(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            (directory / 'corrections' / 'board').mkdir(parents=True)
            config = directory / 'board.yaml'
            config.write_text(yaml.safe_dump(dict(device_id='board', firmware_device_id=1,
                port='/dev/example', sensor_correction_dir='corrections')))
            before = {path: path.read_bytes() if path.is_file() else None
                      for path in directory.rglob('*')}
            with patch('sensor_array_bringup.launcher.os.execvp') as execvp:
                self.assertEqual(main(['--config', str(config), '--check']), 0)
            execvp.assert_not_called()
            self.assertEqual(before, {path: path.read_bytes() if path.is_file() else None
                                      for path in directory.rglob('*')})

    def test_launcher_execs_roslaunch_and_ignores_ros_remappings(self):
        config_path = str(PACKAGE / 'config/single_array.yaml')
        command = launch_command(self.configuration())
        with patch('sensor_array_bringup.launcher.os.execvp') as execvp:
            main(['--config', config_path, '__name:=board_launcher', '__log:=/tmp/board.log'])
        execvp.assert_called_once_with('roslaunch', command)

    def test_missing_config_and_missing_roslaunch_report_failure(self):
        with contextlib.redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(main(['--config', '/nonexistent/board.yaml', '--check']), 1)
        self.assertIn('Bringup failed:', errors.getvalue())
        with patch('sensor_array_bringup.launcher.os.execvp', side_effect=FileNotFoundError('roslaunch')), \
                contextlib.redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(main(['--config', str(PACKAGE / 'config/single_array.yaml')]), 1)
        self.assertIn('roslaunch', errors.getvalue())

    def test_fixed_device_launch_wiring_uses_relative_topics(self):
        root = ET.parse(PACKAGE / 'launch/device.launch').getroot()
        defaults = {arg.attrib['name']: arg.attrib.get('default') for arg in root.findall('arg')}
        self.assertEqual(set(defaults), {
            'device_id', 'firmware_device_id', 'check_device_id', 'port', 'baud_rate',
            'sensor_correction_dir', 'raw_scale', 'expected_ak', 'expected_tmag', 'expect_icm', 'field_range'})
        self.assertEqual(defaults['device_id'], None)
        group = root.find('group')
        self.assertEqual(group.attrib['ns'], '/$(arg device_id)')
        bridge, protocol, correction, visualization = group.findall('node')
        self.assertEqual(visualization.attrib['type'], 'sensor_visualization_node')
        self.assertEqual(visualization.attrib['output'], 'screen')
        self.assertEqual({item.attrib['name']: item.attrib['value']
                          for item in visualization.findall('param')},
                         {'input_topic': 'maggrad/sample_corr', 'field_range': '$(arg field_range)'})
        self.assertEqual([node.attrib['type'] for node in (bridge, protocol, correction)],
                         ['serial_frame_bridge_node', 'protocol_node', 'sensor_correction_node'])
        bridge_remaps = {item.attrib['from']: item.attrib['to'] for item in bridge.findall('remap')}
        # Private topics already resolve under the node's name. Identity remaps
        # are optional; validate the actual wiring instead of their spelling.
        effective_topics = {
            topic: bridge_remaps.get(topic, bridge.attrib['name'] + '/' + topic[1:])
            for topic in ('~raw_frames', '~tx_bytes', '~diagnostics')}
        self.assertEqual(effective_topics, {
            '~raw_frames': 'serial_frame_bridge/raw_frames',
            '~tx_bytes': 'serial_frame_bridge/tx_bytes',
            '~diagnostics': 'diagnostics'})
        protocol_params = {item.attrib['name']: item.attrib['value']
                           for item in protocol.findall('param')}
        self.assertEqual(protocol_params, {
            'raw_frame_topic': 'serial_frame_bridge/raw_frames',
            'sample_topic': 'maggrad/sample', 'diagnostics_topic': 'diagnostics',
            'expect_icm': '$(arg expect_icm)'})
        for key in ('expected_ak', 'expected_tmag'):
            node = protocol.find("rosparam[@param='%s']" % key)
            self.assertEqual(node.attrib['subst_value'], 'true')
            self.assertEqual(node.text, '$(arg %s)' % key)
        correction_params = {item.attrib['name']: item.attrib['value']
                             for item in correction.findall('param')}
        self.assertEqual(correction_params['input_topic'], 'maggrad/sample')
        self.assertEqual(correction_params['output_topic'], 'maggrad/sample_corr')
        self.assertEqual(correction_params['diagnostics_topic'], 'diagnostics')

    def test_array_launch_uses_single_bringup_entrypoint(self):
        root = ET.parse(PACKAGE / 'launch/array.launch').getroot()
        self.assertEqual(root.find('arg').attrib, {'name': 'config'})
        node = root.find('node')
        self.assertEqual(node.attrib['type'], 'bringup')
        self.assertEqual(node.attrib['required'], 'true')
        self.assertEqual(node.attrib['name'], '$(arg launcher_name)')
        self.assertIn("anon(", root.find("arg[@name='launcher_name']").attrib['default'])
        self.assertEqual(node.attrib['args'], '--config "$(arg config)"')

    def test_bridge_and_protocol_launch_defaults_are_relative(self):
        bridge = ET.parse(PACKAGE.parent / 'serial_frame_bridge/launch/serial_frame_bridge.launch').getroot()
        bridge_args = {item.attrib['name']: item.attrib.get('default')
                       for item in bridge.findall('arg')}
        self.assertEqual(bridge_args['raw_frame_topic'], 'serial_frame_bridge/raw_frames')
        self.assertEqual(bridge_args['tx_bytes_topic'], 'serial_frame_bridge/tx_bytes')
        self.assertEqual(bridge_args['diagnostics_topic'], 'serial_frame_bridge/diagnostics')

        protocol = ET.parse(PACKAGE.parent / 'maggrad_protocol/launch/protocol.launch').getroot()
        protocol_args = {item.attrib['name']: item.attrib.get('default')
                         for item in protocol.findall('arg')}
        self.assertEqual(protocol_args['raw_frame_topic'], 'serial_frame_bridge/raw_frames')
        self.assertEqual(protocol_args['sample_topic'], 'maggrad/sample')
        self.assertEqual(protocol_args['diagnostics_topic'], 'diagnostics')
        self.assertTrue(all(not value.startswith('/') for value in protocol_args.values()))


if __name__ == '__main__':
    unittest.main()
