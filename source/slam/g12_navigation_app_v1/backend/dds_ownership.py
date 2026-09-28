"""Factory ASDU backend: distinguish native DDS topology from foreign writers.

This is an operational conflict monitor, NOT DDS authentication or a receiver
ownership lock. Native NAV writers may stay advertised but must remain silent.
"""
import math

BARE = '_CREATED_BY_BARE_DDS_APP_'
COMMAND_TYPES = {'/NAV_CMD': 'drdds/msg/NavCmd', '/JOINTS_CMD': 'drdds/msg/JointsDataCmd',
                 '/GAIT': 'drdds/msg/Gait', '/MOTION_CMD': None,
                 '/wym/goai/nav_cmd_vel': None}
ANCHORS = {'/MOTION_INFO': 'drdds/msg/MotionInfo', '/JOINTS_DATA': 'drdds/msg/JointsData',
           '/IMU_DATA': 'drdds/msg/ImuData', '/MOTION_STATE': 'drdds/msg/MotionState',
           '/NAV_STATUS': 'drdds/msg/StdMsgInt32', '/OOA_STATUS': 'drdds/msg/StdStatus',
           '/PLANNER_STATUS': 'drdds/msg/PlannerStatus'}


def participant(endpoint, expected_type):
    try:
        gid = bytes.fromhex(endpoint['gid'])
        if len(gid) != 16 or gid[:2] != b'\x01\x0f' or not any(gid[:12]): return None
        if endpoint['name'] != BARE or endpoint['namespace'] != BARE: return None
        if endpoint['type'] != expected_type: return None
        return gid[:12].hex()
    except (KeyError, ValueError, TypeError):
        return None


class DdsOwnership:
    def __init__(self):
        self.motion = None
        self.handler = None
        self.last_nav = -math.inf
        self.nav_messages = 0
        self.last_nav_gid = None
        self.stable_since = None
        self.fingerprint = None
        self.revision = 0

    def telemetry(self, topic, gid, now, valid=True, idle=True):
        value = (bytes(gid).hex(), now, bool(valid), bool(idle))
        if topic == '/MOTION_INFO': self.motion = value
        if topic == '/OOA_STATUS': self.handler = value

    def nav_received(self, gid, now):
        # ANY NAV_CMD, including zero or a malformed serialized sample, indicates
        # a second active input stream. Never ignore it just because it is native.
        self.last_nav = now
        self.nav_messages += 1
        self.last_nav_gid = bytes(gid).hex()

    def audit(self, graph, now, rmw='rmw_fastrtps_cpp'):
        def anchor(topic):
            rows = graph.get(topic, [])
            roles = {participant(e, ANCHORS[topic]) for e in rows}
            return next(iter(roles)) if roles and None not in roles and len(roles) == 1 else None
        def observed(value, topic, prefix, max_age):
            return bool(value and prefix and value[0][:24] == prefix and value[2]
                        and 0 <= now-value[1] <= max_age
                        and any(e.get('gid') == value[0] for e in graph.get(topic, [])))
        motion = anchor('/MOTION_INFO')
        motion = motion if motion == anchor('/JOINTS_DATA') == anchor('/IMU_DATA') else None
        # Identity and freshness are separate. Expired telemetry still blocks
        # control, but must not relabel a correlated factory writer as foreign.
        motion_fresh = observed(self.motion, '/MOTION_INFO', motion, 1.)
        server = anchor('/MOTION_STATE')
        server = server if server == anchor('/NAV_STATUS') else None
        handler = anchor('/OOA_STATUS')
        # OOA_STATUS is event-driven on the observed firmware. An advertised
        # handler is not an active velocity stream. If an active/error status IS
        # received, retain that exclusion until an idle update is received.
        handler_active = bool(self.handler and self.handler[0][:24] == handler
                              and (not self.handler[2] or not self.handler[3]))
        planner = anchor('/PLANNER_STATUS')
        known, unknown = [], []
        for topic, expected_type in COMMAND_TYPES.items():
            for endpoint in graph.get(topic, []):
                prefix = participant(endpoint, expected_type) if expected_type else None
                role = None
                if rmw in ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp') and prefix:
                    if topic == '/JOINTS_CMD' and prefix == motion: role = 'factory_motion'
                    if topic == '/GAIT' and prefix == server: role = 'factory_server'
                    if topic == '/NAV_CMD' and prefix == handler: role = 'silent_factory_handler'
                    if topic == '/NAV_CMD' and prefix == planner: role = 'silent_factory_planner'
                record = dict(endpoint, topic=topic, role=role)
                (known if role else unknown).append(record)
        fingerprint = tuple(sorted((r['topic'], r.get('gid', ''), r.get('role') or 'UNKNOWN') for r in known+unknown))
        if fingerprint != self.fingerprint:
            self.fingerprint = fingerprint
            self.revision += 1
            self.stable_since = now
        recent_nav = not math.isfinite(now) or now < self.last_nav or now-self.last_nav < 1.
        reason = ('UNSUPPORTED_DDS_RMW' if rmw not in ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp') else
                  'UNKNOWN_DDS_WRITER' if unknown else
                  'DDS_CLOCK_ORDER' if self.motion and now < self.motion[1] else
                  'DDS_TELEMETRY_UNAVAILABLE' if not motion_fresh else
                  'FACTORY_SERVER_TOPOLOGY_UNAVAILABLE' if not server else
                  'FACTORY_HANDLER_NOT_IDLE' if handler_active else
                  'COMPETING_NAV_TRAFFIC' if recent_nav else
                  'WAIT_DDS_STABLE_1S' if self.stable_since is None or now-self.stable_since < 1. else None)
        return dict(ok=reason is None, reason=reason, revision=self.revision, native=known, unknown=unknown,
                    nav_messages=self.nav_messages, last_nav_gid=self.last_nav_gid,
                    nav_age_s=None if not math.isfinite(self.last_nav) else now-self.last_nav,
                    note='Role correlation and silence monitoring, not authenticated or atomic ownership')


class RosDdsOwnership:
    """Subscriptions only; no robot command publishers or services."""
    def __init__(self, node):
        import time
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.utilities import get_rmw_implementation_identifier
        from drdds.msg import MotionInfo, NavCmd, StdStatus
        self.node, self.time = node, time
        self.rmw = get_rmw_implementation_identifier()
        self.audit_state = DdsOwnership()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        def gid(info, topic=None):
            value = info.get('publisher_gid') if isinstance(info, dict) else getattr(info, 'publisher_gid', None)
            if value is not None: return value
            # Jazzy's Python MessageInfo on this host omits publisher_gid. Use
            # only an unambiguous single-writer graph for telemetry correlation.
            rows = node.get_publishers_info_by_topic(topic) if topic else []
            if len(rows) != 1: raise ValueError('AMBIGUOUS_MESSAGE_SOURCE')
            return rows[0].endpoint_gid
        def motion(msg, info):
            try:
                values = msg.data
                valid = all(math.isfinite(float(getattr(values, k))) for k in ('vel_x', 'vel_y', 'vel_yaw', 'height'))
                self.audit_state.telemetry('/MOTION_INFO', gid(info, '/MOTION_INFO'), time.monotonic(), valid)
            except (AttributeError, KeyError, ValueError, TypeError): self.audit_state.motion = None
        def handler(msg, info):
            try:
                self.audit_state.telemetry('/OOA_STATUS', gid(info, '/OOA_STATUS'), time.monotonic(),
                                           type(msg.state) is int, msg.state == 0 and msg.error_code == 0)
            except (AttributeError, KeyError, ValueError, TypeError): self.audit_state.handler = None
        def nav(raw, info):
            try: source = gid(info)
            except (AttributeError, KeyError, ValueError, TypeError): source = bytes(16)
            self.audit_state.nav_received(source, time.monotonic())
        self.subscriptions = [node.create_subscription(MotionInfo, '/MOTION_INFO', motion, qos),
                              node.create_subscription(StdStatus, '/OOA_STATUS', handler, qos),
                              node.create_subscription(NavCmd, '/NAV_CMD', nav, qos, raw=True)]

    def graph(self):
        return {topic: [dict(name=e.node_name, namespace=e.node_namespace, type=e.topic_type,
                            gid=bytes(e.endpoint_gid).hex()) for e in self.node.get_publishers_info_by_topic(topic)]
                for topic in {**ANCHORS, **COMMAND_TYPES}}

    def snapshot(self, now):
        return self.audit_state.audit(self.graph(), now, self.rmw)


def apply_audit(env, audit):
    """Keep compatibility fields, but counts now mean unrecognized writers."""
    result = dict(env, dds_ownership=audit)
    unknown = audit['unknown']
    result['other_nav_publishers'] = sum(e['topic'] == '/NAV_CMD' for e in unknown)
    result['other_named_joint_publishers'] = [e.get('gid', '?') for e in unknown if e['topic'] == '/JOINTS_CMD']
    result['mode_publishers'] = sum(e['topic'] not in ('/NAV_CMD', '/JOINTS_CMD') for e in unknown)
    return result


def environment_reason(env, now):
    if not env or not 0 <= now-env['mono'] <= .5: return 'OWNERSHIP_CHECK_STALE'
    if env.get('rl_service_active'): return 'COMPETING_CONTROLLER_PROCESS'
    if env.get('other_nav_publishers') or env.get('other_named_joint_publishers') or env.get('mode_publishers'):
        return 'COMPETING_CONTROLLER:'+str(env.get('dds_ownership', {}).get('reason', 'UNKNOWN_DDS_WRITER'))
    if 'dds_ownership' in env and env['dds_ownership'].get('ok') is not True:
        return env['dds_ownership'].get('reason') or 'DDS_OWNERSHIP_UNAVAILABLE'
    return None
