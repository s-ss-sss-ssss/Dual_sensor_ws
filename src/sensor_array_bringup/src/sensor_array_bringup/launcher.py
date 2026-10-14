"""Validate a board YAML and run the fixed device launch in the foreground."""
import argparse
import signal
import subprocess
import sys

import yaml

from sensor_array_bringup.configuration import load_configuration


def launch_arguments(configuration):
    """Return roslaunch name:=value arguments, without shell interpretation."""
    values = {key: configuration[key] for key in (
        'device_id', 'firmware_device_id', 'check_device_id', 'namespace',
        'port', 'baud_rate', 'sensor_correction_dir', 'raw_scale', 'expect_icm')}
    values.update(
        raw_frame_topic=configuration['topics']['raw_frames'],
        tx_bytes_topic=configuration['topics']['tx_bytes'],
        sample_topic=configuration['topics']['sample'],
        sensor_correction_sample_topic=configuration['sensor_correction_sample'],
        diagnostics_topic=configuration['topics']['diagnostics'])
    for key in ('expected_ak', 'expected_tmag'):
        values[key] = yaml.safe_dump(configuration[key], default_flow_style=True, width=100000).strip()
    return ['%s:=%s' % (key, str(value).lower() if type(value) is bool else value)
            for key, value in values.items()]


def launch_command(configuration):
    return ['roslaunch', 'sensor_array_bringup', 'device.launch'] + launch_arguments(configuration)


def run_launch(configuration):
    """Let roslaunch manage the nodes, forwarding both shutdown signals."""
    previous = {}
    process = None
    pending = []

    def forward(signum, _frame):
        if process is None:
            pending.append(signum)
        elif process.poll() is None:
            process.send_signal(signum)

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, forward)
        # Separate session prevents a terminal SIGINT from reaching the child
        # twice: once from the terminal and once from the wrapper.
        process = subprocess.Popen(launch_command(configuration), start_new_session=True)
        for signum in pending:
            forward(signum, None)
        result = process.wait()
        return result if result >= 0 else 128 - result
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, help='Single-board YAML file')
    parser.add_argument('--check', action='store_true', help='Validate offline; no ports, subprocesses or output files')
    args = parser.parse_args(argv)
    try:
        configuration = load_configuration(args.config)
        print('%s: %s -> %s -> %s -> %s' % (
            configuration['device_id'], configuration['port'], configuration['namespace'],
            configuration['topics']['sample'], configuration['sensor_correction_sample']), flush=True)
        if configuration['firmware_device_id'] == 0:
            print('WARNING: firmware_device_id is 0; examples use this as a placeholder. '
                  'Confirm the actual firmware ID before connecting.', file=sys.stderr)
        if args.check:
            return 0
        return run_launch(configuration)
    except (OSError, ValueError, TypeError, ImportError, yaml.YAMLError) as error:
        print('Bringup failed: %s' % error, file=sys.stderr)
        return 1


def configured_main(argv=None):
    # roslaunch adds ROS remapping arguments; this wrapper has no rospy dependency.
    arguments = sys.argv[1:] if argv is None else argv
    return main([argument for argument in arguments if ':=' not in argument])


if __name__ == '__main__':
    raise SystemExit(main())
