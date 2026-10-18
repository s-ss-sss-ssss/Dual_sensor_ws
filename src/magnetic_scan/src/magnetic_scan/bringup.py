"""Generate an isolated dual hardware launch using upstream descriptions/MoveIt."""
import argparse
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml
from .configuration import load_config, load_points


def render(config, vision=False, sensors=False):
    c, h = config, config['hardware']
    arm = c['source_arm']
    inactive = 'arm2' if arm == 'arm1' else 'arm1'
    root = ET.Element('launch')

    def include(file, args):
        node = ET.SubElement(root, 'include', file=file)
        for name, value in args.items():
            ET.SubElement(node, 'arg', name=name, value=str(value))
        return node

    tools = dict(diana7_tool=h['sensor_tool'], **{arm + '_tool': h['source_tool'], inactive + '_tool': h['inactive_tool']})
    layout = h.get('layout_config', '$(find zlab_robots_bringup)/config/robot_layout.yaml')
    # Quote each shell word used by roslaunch's command attribute.
    import shlex
    command = "$(find xacro)/xacro '$(find multi_arm_description)/urdf/zlab_triple_arms.xacro' "
    command += ' '.join(shlex.quote(k + ':=' + v) for k, v in dict(layout_config=layout, **tools).items())
    ET.SubElement(root, 'param', name='robot_description', command=command)
    include('$(find diana7_hardware)/launch/diana7_hardware.launch', dict(
        robot_ip=h['diana7_ip'], tool_name=h['sensor_tool'], loop_hz=h.get('diana7_loop_hz', 100.),
        load_robot_description='false', publish_robot_state='false', prefix='diana7'))
    include('$(find fr5_hardware)/launch/fr5_hardware.launch', dict(
        arm_id=arm, robot_ip=h.get('source_ip', ''), tool_name=h['source_tool'], loop_hz=h.get('fr5_loop_hz', 30.),
        load_robot_description='false', publish_robot_state='false', spawn_joint_state_controller='true'))
    ET.SubElement(root, 'node', pkg='robot_state_publisher', type='robot_state_publisher', name='robot_state_publisher')
    merger = ET.SubElement(root, 'node', pkg='magnetic_scan', type='scan_joint_states', name='scan_joint_states', required='true', output='screen')
    for key, value in dict(config=c['config_path'], source_arm=arm).items():
        ET.SubElement(merger, 'param', name=key, value=value)
    include('$(find triple_arm_moveit_config)/launch/move_group.launch', dict(
        load_robot_description='false', moveit_controller_manager='simple', **tools))
    # All roslaunch parameters are loaded before nodes. Replace the upstream triple list.
    controllers = [dict(name='diana7/position_trajectory_controller', action_ns='follow_joint_trajectory',
                        type='FollowJointTrajectory', default=True, joints=['diana7_joint_%d' % i for i in range(1, 8)]),
                   dict(name=arm + '/' + arm + '_controller', action_ns='follow_joint_trajectory',
                        type='FollowJointTrajectory', default=True, joints=[arm + '_joint_%d' % i for i in range(1, 7)])]
    ET.SubElement(root, 'rosparam', param='/move_group/controller_list').text = yaml.safe_dump(controllers)
    if vision:
        # Track raw visual bodies only; physical offsets are owned by scan config.
        args = {k: 'none' for k in ('diana7_tool', 'arm1_tool', 'arm2_tool', 'sensor_tool')}
        args.update(c.get('vision_launch_args', {}))
        include('$(find zlab_robots_calibration)/launch/localization_tagslam.launch', args)
    if sensors:
        sensor_config = h.get('sensor_config')
        if not sensor_config:
            raise ValueError('hardware.sensor_config required to launch sensors')
        p = Path(sensor_config).expanduser()
        if not p.is_absolute():
            p = Path(c['config_path']).parent / p
        include('$(find sensor_array_bringup)/launch/array.launch', dict(config=str(p)))
    include('$(find magnetic_scan)/launch/scan.launch', dict(config=c['config_path'], source_arm=arm))
    return ET.tostring(root, encoding='unicode')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--source-arm', choices=['arm1', 'arm2'])
    parser.add_argument('--vision', action='store_true')
    parser.add_argument('--sensors', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--dump-launch', action='store_true')
    import sys
    import rospy
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    c = load_config(args.config, args.source_arm, hardware=True)
    load_points(c['points_file'])
    xml = render(c, args.vision, args.sensors)
    if args.dump_launch:
        print(xml)
        return
    if args.check:
        print('Valid dual hardware configuration; no nodes started')
        return
    import roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    with tempfile.TemporaryDirectory(prefix='magnetic_scan_') as folder:
        path = Path(folder) / 'dual.launch'
        path.write_text(xml, encoding='utf-8')
        parent = roslaunch.parent.ROSLaunchParent(uuid, [str(path)])
        try:
            parent.start()
            parent.spin()
        finally:
            parent.shutdown()
