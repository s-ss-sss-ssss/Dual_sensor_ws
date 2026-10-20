"""Experiment state machine, shared by ROS and deterministic mock adapters."""
from .geometry import aligned_target, inverse, limited_step, offset_target, residual, transform
from .initial_pose import load_initial_pose, make_initial_pose, save_initial_pose


class ScanError(RuntimeError):
    pass


class Stopped(ScanError):
    pass


class Engine:
    def __init__(self, config, points, motion, vision, sampler, recorder, cancelled=lambda: False,
                 status=lambda state, point: None, health=lambda: None):
        self.config, self.points = config, points['points']
        self.motion, self.vision, self.sampler, self.recorder = motion, vision, sampler, recorder
        self.cancelled, self.status, self.health = cancelled, status, health
        self.p = config['parameters']
        self.stage = 0
        self.source_initial = None
        self.mapping = None  # current T_planning_vision; local command compensation only
        self.last_vision = None
        self.point_id = None
        self.last_command = None

    def check(self):
        if self.cancelled():
            raise Stopped('stop requested')
        self.health()

    def state(self, name):
        self.check()
        self.status(name, self.point_id)
        self.recorder.event('state', state=name, point_id=self.point_id)

    def observe(self):
        self.check()
        observation = self.vision.observe(self.check)
        self.check()
        self.last_vision = observation
        self.recorder.event('vision', point_id=self.point_id, **observation)
        source = observation['source']
        if self.source_initial is not None:
            pos, angle = residual(source, self.source_initial)
            if pos > self.p['source_tolerance_m'] or angle > self.p['source_tolerance_rad']:
                raise ScanError('fixed source moved: %.6f m, %.6f rad' % (pos, angle))
        return observation

    def command(self, target):
        self.check()
        self.motion.sensor_pose()  # require a fresh complete robot TF before motion
        # Convert physical sensor target to the exact configured MoveIt link.
        link_to_sensor = self.motion.link_to_sensor
        cmd = self.mapping @ target @ inverse(link_to_sensor)
        self.recorder.event('motion_command', point_id=self.point_id, stage=self.stage,
                            target_vision=target, command_planning_link=cmd)
        self.last_command = cmd.copy()
        self.motion.move_to(cmd, self.check)
        self.motion.wait_still(self.check)
        self.check()
        self.recorder.event('motion_complete', point_id=self.point_id, **self.motion.snapshot())
        return cmd

    def correct(self, target, observation):
        for iteration in range(self.p['max_iterations'] + 1):
            actual = observation['sensor']
            pos, angle = residual(actual, target)
            self.recorder.event('residual', point_id=self.point_id, iteration=iteration,
                                translation_m=pos, rotation_rad=angle)
            # Re-anchor the command mapping to the observed physical sensor pose.
            # Physical origin/targets stay frozen; no TF or robot base is modified.
            self.mapping = self.motion.sensor_pose() @ inverse(actual)
            self.stage += 1
            self.recorder.event('compensation', point_id=self.point_id, stage=self.stage,
                                planning_from_vision=self.mapping)
            if pos <= self.p['position_tolerance_m'] and angle <= self.p['angle_tolerance_rad']:
                return observation
            if iteration == self.p['max_iterations']:
                raise ScanError('visual correction did not converge')
            step = limited_step(actual, target, self.p['correction_step_m'], self.p['correction_step_rad'])
            self.command(step)
            self.motion.wait(self.p['settle_s'], self.check)
            observation = self.observe()
        raise AssertionError('unreachable')

    def validate_geometry(self):
        self.check()
        self.motion.validate_geometry(self.check)
        self.vision.validate_geometry(self.check)
        self.config['resolved_geometry'] = dict(
            mode=self.motion.geometry_mode,
            robot_sensor_frame=self.config['robot_sensor_frame'],
            planning_link=self.config['planning_link'],
            fixed_transform=self.motion.link_to_sensor.tolist(),
            visual_profiles=self.config['visual_profiles'])
        self.recorder.artifact('geometry.json', self.config['resolved_geometry'])

    def prepare(self):
        self.validate_geometry()
        self.check()
        saved = load_initial_pose(self.config)
        self.recorder.artifact('initial_pose.json', saved)
        self.state('RESTORING')
        self.motion.restore_joints(saved['joints'], self.check)
        self.motion.wait_still(self.check)
        self.motion.wait(self.p['settle_s'], self.check)
        self.recorder.event('start_restored', **self.motion.snapshot())
        self.state('START_CHECKING')
        first = self.observe()
        actual = inverse(first['source']) @ first['sensor']
        pos, angle = residual(actual, transform(saved['relative_pose']))
        self.recorder.event('start_residual', translation_m=pos, rotation_rad=angle)
        if pos > self.p['position_tolerance_m'] or angle > self.p['angle_tolerance_rad']:
            raise ScanError('starting relative pose outside tolerance: %.6f m, %.6f rad' % (pos, angle))
        self.source_initial = first['source'].copy()
        self.mapping = self.motion.sensor_pose() @ inverse(first['sensor'])
        self.origin = first['sensor'].copy()
        self.recorder.event('origin', sensor_vision=self.origin, source_vision=self.source_initial,
                            stage=self.stage)

    def run(self):
        result = None
        try:
            self.prepare()
            for index, point in enumerate(self.points):
                self.point_id = point['id']
                target = offset_target(self.origin, point['offset_m'])
                result = dict(id=self.point_id, index=index, offset_m=point['offset_m'],
                              target_vision=target, state='started')
                self.state('MOVING')
                result['command_planning_link'] = self.command(target)
                self.motion.wait(point.get('settle_s', self.p['settle_s']), self.check)
                check_vision = point.get('vision_check', False)
                observation = None
                if check_vision:
                    self.state('CHECKING')
                    observation = self.correct(target, self.observe())
                # Last corrective movement must settle before any samples are collected.
                self.motion.wait_still(self.check)
                result.update(stage=self.stage, visual_measurement=observation,
                              last_visual_stamp=self.last_vision['stamp'],
                              actual=self.motion.snapshot(),
                              final_command_planning_link=self.last_command)
                self.vision.validate_geometry(self.check)
                self.motion.sensor_pose()
                self.state('SAMPLING')
                result['samples'] = self.sampler.collect(point.get('sample_s', self.p['sample_s']), self.check)
                self.check()
                result['state'] = 'complete'
                self.recorder.point(result)
                result = None
            self.state('COMPLETE')
            self.recorder.finish('COMPLETE', '')
            return 'COMPLETE'
        except Exception as error:
            self.motion.stop()
            if result is not None:
                result.update(state='failed', reason=str(error))
                self.recorder.point(result)
            state = 'STOPPED' if isinstance(error, Stopped) else 'FAILED'
            self.status(state, self.point_id)
            self.recorder.finish(state, str(error))
            return state


class AlignmentEngine(Engine):
    """Determine and persist a start without loading points or collecting samples."""
    def __init__(self, config, motion, vision, recorder, **kwargs):
        super().__init__(config, {'points': []}, motion, vision, None, recorder, **kwargs)

    def run(self):
        try:
            self.validate_geometry()
            self.state('OBSERVING')
            self.motion.wait_still(self.check)
            self.motion.wait(self.p['settle_s'], self.check)
            first = self.observe()
            self.source_initial = first['source'].copy()
            self.mapping = self.motion.sensor_pose() @ inverse(first['sensor'])
            rotation = transform(dict(translation_m=[0, 0, 0],
                                     quaternion_xyzw=self.config['source_to_sensor_quaternion_xyzw']))[:3, :3]
            target = aligned_target(first['source'], first['sensor'], rotation,
                                    self.config['center_distance_range_m'],
                                    self.config['alignment']['target_distance_m'])
            self.state('ALIGNING')
            aligned = self.correct(target, first)
            self.motion.wait_still(self.check)
            saved = make_initial_pose(self.config, aligned, self.motion.snapshot())
            self.check()
            self.recorder.artifact('initial_pose.json', saved)
            save_initial_pose(self.config, saved)
            self.recorder.event('initial_pose_saved', path=self.config['initial_pose_file'])
            self.status('COMPLETE', None)
            self.recorder.finish('COMPLETE', '')
            return 'COMPLETE'
        except Exception as error:
            self.motion.stop()
            state = 'STOPPED' if isinstance(error, Stopped) else 'FAILED'
            self.status(state, None)
            self.recorder.finish(state, str(error))
            return state
