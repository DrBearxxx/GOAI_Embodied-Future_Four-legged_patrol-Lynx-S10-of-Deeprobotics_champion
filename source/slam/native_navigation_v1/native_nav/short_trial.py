"""One-shot map-referenced 0.50 m trial; no ROS, sockets or actuation on import.

Distance is the frozen route's horizontal polyline length, not timed open-loop
travel. Bounds below are software budgets, not certified stopping distances.
"""
import copy
import math
import numpy as np
from .resilient_route import ResilientRoute
from .recovery import angle_distance, RecoveryWindow
from .protocol import MAX_FORWARD_SPEED
from .acceptance import code_hash
from .first_trial import PROFILE
from indoor.control import zero
from indoor.continuity import safety_policy

DISTANCE = .50
MAX_SECONDS = 60.
ARRIVAL_RADIUS = .05


def make_short_route(source):
    """Clip without changing the frozen asset, interpolating only the last edge."""
    route = copy.deepcopy(source)
    points, edges = route['waypoints'], route['edges']
    xyz = np.asarray([p['xyz'] for p in points], dtype=float)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or len(xyz) < 2 or not np.isfinite(xyz).all() or len(edges) != len(xyz)-1:
        raise ValueError('INVALID_TRIAL_SOURCE_ROUTE')
    output, clipped, total = [copy.deepcopy(points[0])], [], 0.
    output[0]['trial_arc_m'] = 0.
    for i, (a, b) in enumerate(zip(xyz, xyz[1:])):
        length = float(np.linalg.norm((b-a)[:2]))
        if length <= 1e-8 or edges[i].get('warnings') != []:
            raise ValueError('TRIAL_ROUTE_REQUIRES_REVIEW')
        take = min(length, DISTANCE-total)
        u = take/length
        end = copy.deepcopy(points[i+1])
        end['xyz'] = (a+u*(b-a)).tolist()
        total += take
        end['trial_arc_m'] = total
        if u < 1.:
            end['id'] = 'T050'
            end['name'] = 'T050'
            if 'source_arc_m' in points[i] and 'source_arc_m' in points[i+1]:
                end['source_arc_m'] = points[i]['source_arc_m']+u*(points[i+1]['source_arc_m']-points[i]['source_arc_m'])
        edge = copy.deepcopy(edges[i])
        edge.update(start=i, end=i+1, length_m=take, speed_limit_mps=MAX_FORWARD_SPEED)
        edge['from'] = output[-1].get('name', str(i))
        edge['to'] = end.get('name', str(i+1))
        clipped.append(edge)
        output[-1]['yaw'] = math.atan2(b[1]-a[1], b[0]-a[0])
        output.append(end)
        if total >= DISTANCE-1e-10:
            break
    if total < DISTANCE-1e-10:
        raise ValueError('TRIAL_SOURCE_TOO_SHORT')
    output[-1]['yaw'] = output[-2]['yaw']
    route.pop('reference_end_offset_s', None)
    route.update(waypoints=output, edges=clipped, stop_at_end=True, automatic_return=False,
                 selection='First 0.50 m horizontal polyline of frozen indoor route; interpolated endpoint',
                 trial=dict(schema='s10.native.trial50.v1', distance_m=DISTANCE,
                            distance_metric='horizontal_polyline', max_vx_mps=MAX_FORWARD_SPEED,
                            max_wz_radps=.20, max_seconds=MAX_SECONDS,
                            arrival_radius_m=ARRIVAL_RADIUS, resume_allowed=False))
    return route


class ShortTrial(ResilientRoute):
    def __init__(self, source, asset_id, boot_id, first_trial=False, allow_start_offset=False):
        if allow_start_offset and not first_trial:
            raise ValueError('START_OFFSET_ONLY_FOR_FIRST_TRIAL')
        super().__init__(make_short_route(source), asset_id, boot_id)
        self.allow_start_offset = allow_start_offset
        self.start_anchor = None
        self.anchor_window = RecoveryWindow()
        self.anchor_identity = None
        self.started = None
        self.terminal_reason = None
        self.command_distance = 0.
        self.command_turn = 0.
        self.budget_time = None
        self.previous_output = zero('IDLE')
        self.arriving = False
        self.arrival_since = None
        self.arrival_measurements = set()
        self.final_error = None
        self.expected_gateway_hash = code_hash()
        self.expected_backend = 'first_trial' if first_trial else 'execute'

    def _anchor_start(self, now, snapshot, feedback, env):
        """Translate this temporary 50 cm segment, never the pose or frozen map."""
        try:
            if feedback and feedback.get('session') != self.gateway_session:
                self.gateway_session = None
                self.anchor_window.clear()
            reason = self.backend_check(now, feedback, env)
            if reason:
                self.anchor_window.clear()
                return reason
            if (not snapshot or snapshot.get('schema') != 'goai.localization.continuity.v2'
                    or snapshot.get('asset_id') != self.asset_id or snapshot.get('boot_id') != self.boot_id):
                raise ValueError('MAP_OR_CLOCK_ID_MISMATCH')
            run, generation = snapshot['run_id'], snapshot['generation']
            if (not isinstance(run, str) or not run or not isinstance(generation, list) or len(generation) != 2
                    or any(type(x) is not int or x < 0 for x in generation)):
                raise ValueError('INVALID_LOCALIZER_IDENTITY')
            identity = (run, generation[0])
            if identity != self.anchor_identity:
                self.anchor_window.clear()
                self.anchor_identity = identity
            pose = np.asarray(snapshot['pose'], float)
            if pose.shape != (4,) or not np.isfinite(pose).all() or not 0 <= now-snapshot['mono'] <= .25:
                raise ValueError('STALE_OR_INVALID_ANCHOR_POSE')
            if snapshot.get('error'):
                raise ValueError('LOCALIZER_EXCEPTION')
            reason = self.localization_check(now, snapshot)
            if reason:
                raise ValueError(reason)
            ready = self.anchor_window.observe(now, snapshot, self.stationary(feedback),
                                               safety_policy(now, snapshot['safety_records'])['mode'] == 'FRESH')
            if not ready:
                return 'WAIT_STABLE_START_ANCHOR'
            center = np.median([p[:3] for _, p in self.anchor_window.samples], axis=0)
            offset = center-self.xyz[0]
            if np.linalg.norm(offset[:2]) > 2. or abs(offset[2]) > .20:
                return 'START_OFFSET_OUTSIDE_LOCAL_TRIAL_ENVELOPE'
            # Once set, no re-anchoring is possible, even after STOP/relocalization.
            self.xyz = self.xyz+offset
            for point, xyz in zip(self.route['waypoints'], self.xyz):
                point['xyz'] = xyz.tolist()
            self.start_anchor = dict(frame='joint_map', translation_m=offset.tolist(),
                                     start=self.xyz[0].tolist(), end=self.xyz[-1].tolist(),
                                     mono=now, localizer_run=run, map_generation=generation[0],
                                     kind='temporary_translated_trial_not_map_alignment')
            self.route['trial']['start_anchor'] = self.start_anchor
            return None
        except (KeyError, TypeError, ValueError, OverflowError, IndexError) as exc:
            self.anchor_window.clear()
            return str(exc)

    def backend_check(self, now, feedback, env):
        reason = super().backend_check(now, feedback, env)
        if reason:
            return reason
        if feedback.get('code_hash') != self.expected_gateway_hash:
            return 'TRIAL_GATEWAY_VERSION_MISMATCH'
        if self.expected_backend == 'first_trial':
            profile = feedback.get('first_trial') or {}
            if (profile.get('schema') != PROFILE or profile.get('acceptance') != 'UNVERIFIED'
                    or profile.get('max_distance_m') != .50 or profile.get('max_vx_mps') != .20
                    or profile.get('max_seconds') != 60. or profile.get('one_shot') is not True
                    or profile.get('guardian_ready') is not True):
                return 'INVALID_FIRST_TRIAL_PROFILE'
        return None

    def stop(self, reason='OPERATOR_STOP'):
        if self.started is not None and self.terminal_reason is None:
            self.terminal_reason = reason
        super().stop(reason)

    def check(self, now, snapshot, feedback, env):
        reason = super().check(now, snapshot, feedback, env)
        if reason:
            return reason
        try:
            p = np.asarray(snapshot['pose'], float)
            measured = np.asarray(snapshot['solution']['measured_pose'], float)
            f = feedback['feedback']
            if not math.isfinite(f['speed']) or not 0 <= f['speed'] <= .30 or not math.isfinite(f['wz']) or abs(f['wz']) > .35:
                return 'TRIAL_BODY_SPEED_OUT_OF_RANGE'
            cross, height = self._corridor(p[:3])
            if cross > (.08 if self.target == 0 else .12) or height > .10:
                return 'TRIAL_OUTSIDE_CORRIDOR'
            if self.started is None:
                if np.linalg.norm(measured[:3]-self.xyz[0]) > .08:
                    return 'TRIAL_START_NOT_REACHED'
                if angle_distance(p[3], self.route['waypoints'][0]['yaw']) > .35:
                    return 'TRIAL_START_HEADING'
            if self.armed:
                if np.linalg.norm(p[:2]-self.xyz[0,:2]) > .55:
                    return 'TRIAL_DISPLACEMENT_BUDGET'
                final_direction = self.xyz[-1,:2]-self.xyz[-2,:2]
                final_direction /= np.linalg.norm(final_direction)
                if (p[:2]-self.xyz[-1,:2]) @ final_direction > .03:
                    return 'TRIAL_ENDPOINT_OVERSHOOT'
            return None
        except (KeyError, TypeError, ValueError, OverflowError):
            return 'INVALID_TRIAL_INPUT'

    def velocity_limits(self, snapshot):
        # All parent quality/obstacle/time checks run first. Only fully fresh
        # TRACKING may use the requested ceiling; degraded caps stay unchanged.
        if (self.solution_mode == 'TRACKING' and self.measurement_age <= .25
                and self.last_cycle >= self.recover_after
                and safety_policy(self.last_cycle, snapshot['safety_records'])['mode'] == 'FRESH'):
            return MAX_FORWARD_SPEED, .20
        vx, wz = super().velocity_limits(snapshot)
        return min(vx, MAX_FORWARD_SPEED), min(wz, .20)

    def goal_reached(self, snapshot, distance):
        # The last point needs a stationary dwell, not a single near-goal pose.
        if self.target == len(self.xyz)-1:
            return False
        radius = .08 if self.target == 0 else .05
        measured = np.asarray(snapshot['solution']['measured_pose'][:3])
        return (self.solution_mode in ('TRACKING', 'DEGRADED') and self.measurement_age <= .25
                and distance <= radius and np.linalg.norm(measured-self.xyz[self.target]) <= radius)

    def arm(self, now):
        if self.allow_start_offset and self.start_anchor is None:
            return False, 'WAIT_STABLE_START_ANCHOR'
        if self.started is not None or self.terminal_reason:
            return False, 'TRIAL_FINISHED_RESTART_REQUIRED'
        ok, reason = super().arm(now)
        if ok:
            self.started = self.budget_time = now
            self.previous_output = zero('ARMED')
        return ok, reason

    def _output(self, command, now):
        if self.terminal_reason:
            command = zero(self.terminal_reason, target=self.target, armed=False, ready=False,
                           complete=self.complete, mission_state='COMPLETE' if self.complete else 'FINISHED')
        command = dict(command, trial_distance_m=DISTANCE, max_vx_mps=MAX_FORWARD_SPEED,
                       allow_start_offset=self.allow_start_offset, start_anchor=self.start_anchor,
                       elapsed_s=None if self.started is None or not math.isfinite(now) else max(0., now-self.started),
                       commanded_distance_m=self.command_distance, commanded_turn_rad=self.command_turn,
                       endpoint_error_m=self.final_error, trial_finished=bool(self.terminal_reason))
        self.previous_output = command
        return command

    def tick(self, now, snapshot, feedback, env):
        if self.terminal_reason:
            return self._output(zero(self.terminal_reason), now)
        if self.allow_start_offset and self.start_anchor is None:
            reason = self._anchor_start(now, snapshot, feedback, env)
            if reason:
                return self._output(zero(reason, target=0, armed=False, ready=False), now)
        if self.started is not None and self.budget_time is not None:
            dt = now-self.budget_time
            if math.isfinite(dt) and 0 <= dt <= .20:
                self.command_distance += self.previous_output['vx']*dt
                self.command_turn += abs(self.previous_output['wz'])*dt
            self.budget_time = now
            if now-self.started >= MAX_SECONDS:
                self.stop('TRIAL_TIMEOUT')
            elif self.command_distance >= DISTANCE:
                self.stop('TRIAL_COMMAND_DISTANCE_BUDGET')
            elif self.command_turn >= 1.20:
                self.stop('TRIAL_TURN_BUDGET')
            if self.terminal_reason:
                return self._output(zero(self.terminal_reason), now)
        c = super().tick(now, snapshot, feedback, env)
        if self.holding:
            # Long missions may resume a HOLD; this commissioning trial never
            # does. Bounded DEGRADED/PREDICT/ODOM_BRIDGE still work normally.
            self.stop('TRIAL_STOP:'+str(self.hold_reason))
        if self.armed and self.target == len(self.xyz)-1:
            p = np.asarray(snapshot['pose'][:3], float)
            self.final_error = float(np.linalg.norm(p-self.xyz[-1]))
            if self.final_error <= ARRIVAL_RADIUS:
                self.arriving = True
            if self.arriving:
                c = zero('TRIAL_CONFIRM_STOP', target=self.target, armed=True, ready=False)
                self.last_command = c
                self.last_tick = now
                if self.final_error > .08:
                    self.stop('TRIAL_ARRIVAL_DRIFT')
                s = snapshot['solution']
                f = feedback['feedback']
                confirmed = (self.final_error <= ARRIVAL_RADIUS and self.measurement_age <= .25
                             and self.solution_mode in ('TRACKING', 'DEGRADED')
                             and np.linalg.norm(np.asarray(s['measured_pose'][:3])-self.xyz[-1]) <= ARRIVAL_RADIUS
                             and f['speed'] <= .015 and abs(f['wz']) <= .03)
                if confirmed and self.armed:
                    if self.arrival_since is None:
                        self.arrival_since = now
                    self.arrival_measurements.add(s['measurement_mono'])
                    if now-self.arrival_since >= .60 and len(self.arrival_measurements) >= 3:
                        self.complete = True
                        self.reached.append(self.target)
                        self.stop('COMPLETE')
                else:
                    self.arrival_since = None
                    self.arrival_measurements.clear()
        return self._output(c, now)
