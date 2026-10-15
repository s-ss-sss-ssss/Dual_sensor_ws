"""Validate a board YAML and replace this process with its roslaunch."""
import argparse
import os
import sys

import yaml

from sensor_array_bringup.configuration import load_configuration


def launch_command(configuration):
    """Return the fixed single-board launch command as an argv list."""
    values = {key: configuration[key] for key in (
        'device_id', 'firmware_device_id', 'check_device_id', 'port',
        'baud_rate', 'sensor_correction_dir', 'raw_scale', 'expect_icm', 'field_range')}
    for key in ('expected_ak', 'expected_tmag'):
        values[key] = yaml.safe_dump(configuration[key], default_flow_style=True,
                                     width=100000).strip()
    arguments = ['%s:=%s' % (
        key, str(value).lower() if type(value) is bool else value)
        for key, value in values.items()]
    return ['roslaunch', 'sensor_array_bringup', 'device.launch'] + arguments


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, help='Single-board YAML file')
    parser.add_argument('--check', action='store_true',
                        help='Validate offline without launching ROS nodes')
    arguments = sys.argv[1:] if argv is None else argv
    # roslaunch appends node remappings such as __name:=... to this process.
    arguments = [argument for argument in arguments if ':=' not in argument]
    args = parser.parse_args(arguments)
    try:
        configuration = load_configuration(args.config)
        print('%s: %s -> /%s' % (
            configuration['device_id'], configuration['port'],
            configuration['device_id']), flush=True)
        if configuration['firmware_device_id'] == 0:
            print('WARNING: firmware_device_id is 0; confirm the actual firmware ID.',
                  file=sys.stderr)
        if args.check:
            return 0
        os.execvp('roslaunch', launch_command(configuration))
    except (OSError, ValueError, TypeError, ImportError, yaml.YAMLError) as error:
        print('Bringup failed: %s' % error, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
