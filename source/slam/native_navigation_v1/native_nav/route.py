"""Same V2 localization gates, independent factory backend/arming semantics."""
import math
from . import bootstrap
from indoor.control import Guard, zero
from indoor.goai_control import GoaiRoute
from indoor.continuity import safety_policy
from .dds_ownership import environment_reason


class NativeRoute(GoaiRoute):
    expected_backend = 'execute'
    # Inherit only V2 quality/corridor/arrival hooks and stored state. The G12 /
    # GOAI permission workflow is NOT used by this backend.
    def __init__(self, route, asset_id, boot_id):
        super().__init__(route, asset_id, boot_id)
        self.gateway_session = None
        self.dds_revision = None

    def backend_check(self, now, feedback, env):
        if not feedback or not 0 <= now - feedback['mono'] <= .15:
            return 'FACTORY_GATEWAY_STALE'
        if feedback['backend'] != self.expected_backend:
            return 'FACTORY_GATEWAY_NOT_EXECUTING'
        if feedback['ready'] is not True:
            return 'FACTORY_GATEWAY:' + str(feedback.get('feedback_reason'))
        if self.gateway_session and feedback['session'] != self.gateway_session:
            return 'FACTORY_GATEWAY_RESTART'
        self.gateway_session = feedback['session']
        if self.armed and feedback['armed'] is not True:
            return 'FACTORY_GATEWAY_DISARMED'
        if not env or not 0 <= now - env['mono'] <= .5:
            return 'OWNERSHIP_CHECK_STALE'
        if env['localization_publishers'] != 1 or env['other_nav_publishers'] != 0:
            return 'AMBIGUOUS_CONTROL_PUBLISHERS'
        if env['other_named_joint_publishers'] or env['mode_publishers'] or env['rl_service_active']:
            return 'COMPETING_CONTROLLER'
        reason = environment_reason(env, now)
        if reason: return reason
        if 'dds_ownership' in env:
            revision = env['dds_ownership']['revision']
            changed = self.dds_revision is not None and revision != self.dds_revision
            self.dds_revision = revision
            if self.armed and changed: return 'FACTORY_DDS_TOPOLOGY_CHANGED'
        return None

    def check(self, now, snapshot, feedback, env):
        try:
            if not snapshot:
                return 'NO_LOCALIZATION'
            if snapshot.get('schema') != 'goai.localization.continuity.v2':
                return 'CONTINUITY_V2_REQUIRED'
            if snapshot['asset_id'] != self.asset_id or snapshot['boot_id'] != self.boot_id:
                return 'MAP_OR_CLOCK_ID_MISMATCH'
            run = snapshot['run_id']
            if not isinstance(run, str) or not run:
                return 'INVALID_LOCALIZER_RUN'
            if run != self.localizer_run:
                active = self.armed
                self.localizer_run = run
                self.last_seq = -1
                self.last_pose = None
                self.good_since = self.strong_since = None
                if active:
                    return 'LOCALIZER_RESTART'
            if snapshot['generation'] != self.observed_generation:
                self.observed_generation = list(snapshot['generation'])
                self.good_since = self.strong_since = None
                if self.armed:
                    return 'RELOCALIZATION_REQUIRES_REARM'
            if self.last_pose and snapshot['seq'] == self.last_seq and snapshot['mono'] != self.last_pose[0]:
                return 'REPEATED_POSE_RESTAMPED'
            if self.last_pose and snapshot['seq'] != self.last_seq and snapshot['mono'] <= self.last_pose[0]:
                return 'POSE_TIME_REGRESSION'
            return Guard.check(self, now, snapshot, feedback, env)
        except (KeyError, TypeError, ValueError, OverflowError):
            return 'INVALID_INPUT'

    def arm(self, now):
        if self.strong_since is None or now - self.strong_since < 2.:
            return False, 'WAIT_2S_FRESH_MAP_AND_COVERAGE'
        return Guard.arm(self, now)

    def tick(self, now, snapshot, feedback, env):
        if self.last_cycle is not None and (now <= self.last_cycle or now - self.last_cycle > .20):
            self.stop('ROUTE_LOOP_STALLED')
        self.last_cycle = now
        reason = self.check(now, snapshot, feedback, env)
        if reason:
            # check() tracks stream identities. Latch before the second base
            # check, otherwise a run-id transition could be consumed and hidden.
            self.stop(reason)
            self.good_since = None
        else:
            strong = (self.solution_mode == 'TRACKING' and self.measurement_age <= .25
                      and safety_policy(now, snapshot['safety_records'])['mode'] == 'FRESH')
            if not strong:
                self.strong_since = None
            elif self.strong_since is None:
                self.strong_since = now
        if self.armed:
            self.renew_deadman(now)
        command = Guard.tick(self, now, snapshot, feedback, env)
        command['ready'] = bool(not self.armed and not self.complete and not reason
                                and self.strong_since is not None and now-self.strong_since >= 2.)
        command['navigation_quality'] = self.solution_mode
        return command
