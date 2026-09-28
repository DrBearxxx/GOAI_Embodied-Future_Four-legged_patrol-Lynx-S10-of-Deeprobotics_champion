"""Map route via factory gateway. Default monitor; explicit one-shot 50 cm option."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from native_nav.bootstrap import ROOT, INDOOR, verify
from native_nav.resilient_route import ResilientRoute
from native_nav.short_trial import ShortTrial, make_short_route
from native_nav.reconnecting_client import ReconnectingClient
from native_nav.ingress import ResilientIngress
from native_nav.route_checkpoint import RouteCheckpoint
from native_nav.bounded_log import BoundedLog
from native_nav.gateway import ownership_check
from native_nav.dds_ownership import RosDdsOwnership, apply_audit
from native_nav.command_replies import CommandReplies, parse_line
from native_nav.operation_feedback import feedback_text
from native_nav.operator_actions import OperatorActions, ACTION_NAMES, action_denial
from indoor.control import zero


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--port', type=int, default=18891)
    p.add_argument('--seconds', type=float, default=0)
    p.add_argument('--resume-checkpoint', action='store_true', help='Restore matching progress only; still requires explicit ARM')
    p.add_argument('--trial-50cm', action='store_true', help='One-shot first 0.50 m of the frozen map route, max 0.20 m/s')
    p.add_argument('--describe', action='store_true', help='Print verified route/settings and exit without ROS or gateway connection')
    p.add_argument('--first-trial', action='store_true', help='Require the independent UNVERIFIED one-shot commissioning gateway')
    p.add_argument('--allow-start-offset', action='store_true', help='Anchor a temporary translated 50 cm segment to a stable current map pose; no map changes')
    a = p.parse_args()
    if not math.isfinite(a.seconds) or a.seconds < 0:
        p.error('--seconds must be finite and nonnegative')
    if a.trial_50cm and a.resume_checkpoint:
        p.error('The one-shot 50 cm trial cannot resume a checkpoint')
    if a.first_trial and not a.trial_50cm:
        p.error('--first-trial requires --trial-50cm; not available for the 10 m route')
    if a.allow_start_offset and not a.first_trial:
        p.error('--allow-start-offset requires --first-trial')
    if a.describe and a.execute:
        p.error('--describe and --execute are mutually exclusive')
    if a.execute and not sys.stdin.isatty():
        p.error('Execution requires an interactive operator terminal')
    asset_id = verify()
    source_route = json.loads((INDOOR/'assets/route.json').read_text())
    route = make_short_route(source_route) if a.trial_50cm else source_route
    if a.describe:
        print(json.dumps(dict(asset_id=asset_id, execute=False, first_trial=a.first_trial,
                              allow_start_offset=a.allow_start_offset, route=route), indent=2))
        return
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    lock = open('/tmp/wym-native-navigation-route.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    guard = ShortTrial(source_route, asset_id, boot, first_trial=a.first_trial,
                       allow_start_offset=a.allow_start_offset) if a.trial_50cm else ResilientRoute(route, asset_id, boot)
    checkpoint = RouteCheckpoint(ROOT/'live_logs'/('trial50-progress.json' if a.trial_50cm else 'route-progress.json'), route, asset_id)
    if a.resume_checkpoint:
        try:checkpoint.restore(guard)
        except (OSError,ValueError,KeyError,TypeError) as exc:p.error('Checkpoint rejected: '+str(exc))
    client = ReconnectingClient(a.port)
    ingress = ResilientIngress()
    stopping = [False]
    def request_stop(signum,frame):stopping[0] = True
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,request_stop)
    contexts, nodes, executors = [], [], []
    for domain in (88, 0):
        context = Context()
        rclpy.init(context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
        node = Node('wym_native_route_' + str(domain) + '_' + str(os.getpid()),
                    context=context, enable_rosout=False, start_parameter_services=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        contexts.append(context)
        nodes.append(node)
        executors.append(executor)
    ownership = RosDdsOwnership(nodes[1])

    def receive(m, info):
        endpoints = nodes[0].get_publishers_info_by_topic('/wym/indoor/localization')
        if len(endpoints) != 1:
            ingress.value = None
            ingress.error = 'AMBIGUOUS_LOCALIZATION_PUBLISHERS'
            guard.stop(ingress.error)
            return
        ingress.receive(m.data, info, time.time_ns(), time.monotonic(), bytes(endpoints[0].endpoint_gid))
        if ingress.hard_fault:guard.stop(ingress.hard_fault)

    nodes[0].create_subscription(String, '/wym/indoor/localization', receive,
                                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    logs = ROOT/'live_logs'
    # Four 8 MiB files across runs; storage failure must not kill the route loop.
    log = BoundedLog(logs/('native-trial50.jsonl' if a.trial_50cm else 'native-route-resilient.jsonl'))
    replies = CommandReplies(logs/'command-replies.jsonl')
    actions = OperatorActions(replies, first_trial=a.first_trial)
    env = None
    env_time = -1e9
    last_tick = -1e9
    last_print = -1e9
    last_state = None
    pending_arm = None
    owned = False
    checkpoint_time = -1e9
    started = time.monotonic()
    operator_session = os.environ.get('S10_FIRST_TRIAL_SESSION') if a.first_trial else None
    replies.echo = not bool(operator_session)
    terminal_state = None
    if operator_session and sys.stdin.isatty():
        import termios
        terminal_state = termios.tcgetattr(sys.stdin.fileno())
        no_echo = list(terminal_state)
        no_echo[3] &= ~termios.ECHO
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, no_echo)
    print(('EXECUTE: SDK_OFF -> STAND -> MODE_NAV -> FLAT -> ARM; STATUS checks feedback'
           if a.first_trial else 'EXECUTE: SDK_OFF -> MODE_NAV -> STAND -> FLAT -> ARM (each explicit)') if a.execute
          else 'MONITOR ONLY: no velocity, mode, gait or joint commands', flush=True)
    if a.trial_50cm:
        print(json.dumps(dict(trial=route['trial'], start=route['waypoints'][0], end=route['waypoints'][-1],
                              note='One ARM only. STOP/fault/completion ends trial; restart does not ARM.')), flush=True)
        if a.allow_start_offset:
            print('Start offset enabled: temporary 50 cm segment anchors once to stable measured map pose; original map/route unchanged.', flush=True)
    try:
        while not stopping[0] and not client.closed and (not a.seconds or time.monotonic()-started < a.seconds):
            for executor in executors:
                executor.spin_once(timeout_sec=.001)
            now = time.monotonic()
            if now-last_tick < .04:
                time.sleep(.002)
                continue
            last_tick = now
            client.poll(now)
            if now-env_time >= .1:
                # Check services outside the ROS timer, bounded; a stall causes the
                # robot-side lease to expire instead of preserving last velocity.
                try:
                    r = subprocess.run(['systemctl', 'is-active', 'rl_deploy.service'],
                                       capture_output=True, text=True, timeout=.15)
                    active = r.stdout.strip() not in ('inactive', 'failed')
                except (OSError, subprocess.TimeoutExpired):
                    active = True
                active = active or not ownership_check()
                if operator_session:
                    try:
                        presence = subprocess.run(['tmux', 'display-message', '-p', '-t', operator_session,
                                                   '#{session_attached}'], capture_output=True, text=True, timeout=.1)
                        attached = presence.returncode == 0 and int(presence.stdout.strip()) > 0
                    except (OSError, ValueError, subprocess.TimeoutExpired):
                        attached = False
                    if not attached:
                        guard.stop('OPERATOR_TERMINAL_DETACHED')
                        print('Operator terminal detached: ending trial; no automatic resume', flush=True)
                        break
                env = dict(mono=time.monotonic(), localization_publishers=nodes[0].count_publishers('/wym/indoor/localization'),
                           rl_service_active=active)
                env = apply_audit(env, ownership.snapshot(env['mono']))
                env_time = env['mono']
            now = time.monotonic()
            status = client.get(now)
            replies.poll(client.drain_acks(), client.connection_count, now,
                         status.get('feedback') if status else None, status=status, status_mono=client.received)
            feedback = dict(status, mono=client.received) if status else None
            was_armed = guard.armed
            c = guard.tick(now, ingress.get(now), feedback, env)
            if pending_arm is not None:
                if now-pending_arm > .20 or not c.get('ready'):
                    guard.stop('ARM_HANDSHAKE_FAILED')
                    client.stop()
                    pending_arm = None
                    replies.arm_result(False, 'ARM_HANDSHAKE_FAILED')
                elif status and status['armed']:
                    ok, why = guard.arm(now)
                    pending_arm = None
                    if not ok:
                        client.stop()
                    replies.arm_result(ok, why)
            if a.execute and guard.armed:
                client.action('VELOCITY', now, [c['vx'], c['vy'], c['wz']])
            elif a.execute and owned and (was_armed or (status and status['armed'] and pending_arm is None)):
                client.stop()
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line:
                    break
                try:
                    name, request_id = parse_line(line)
                except ValueError:
                    continue
                if name == 'STATUS':
                    state_text = feedback_text(status.get('feedback')) if status else '网关状态过期/不可用'
                    latest = replies.latest_action
                    previous = (f"；上一动作 {latest['command']}：{latest['phase']} — {latest['reason']}"
                                if latest else '；本进程暂无动作记录')
                    pose = (ingress.get(now) or {}).get('pose')
                    offset = ''
                    if pose and len(pose) >= 3:
                        distance = math.dist(pose[:3], guard.route['waypoints'][0]['xyz'])
                        if a.allow_start_offset and guard.start_anchor is None:
                            offset = '；允许起点偏移，等待当前稳定定位生成临时 50 cm 路段'
                        elif a.trial_50cm:
                            label = '已固定的临时起点' if a.allow_start_offset else '地图原始起点'
                            offset = f'；定位距{label} {distance:.3f} m（要求 ≤0.08 m）'
                            if a.allow_start_offset:
                                offset += f"；路段平移量={guard.start_anchor['translation_m']}"
                    replies.emit(name, request_id, 'STATUS',
                                 state_text+f"；ready={c.get('ready')}；路线状态={c['state']}"+offset+previous,
                                 None)
                elif name in ('STOP', 'QUIT'):
                    actions.cancel('OPERATOR_STOP')
                    replies.cancel_waits('OPERATOR_STOP')
                    guard.stop('OPERATOR_STOP')
                    pending_arm = None
                    if a.execute:
                        client.stop()
                    replies.emit(name, request_id, 'LOCAL_STOP',
                                 '路线已停止；已请求零速，实际停车须看本体反馈', True)
                    if name == 'QUIT':
                        break
                elif name in ACTION_NAMES and a.execute:
                    denied = action_denial(name, guard.armed, pending_arm, c, env, now)
                    if denied:
                        replies.emit(name, request_id, 'REJECTED', denied, False,
                                     ownership=env.get('dds_ownership') if env else None,
                                     feedback=status.get('feedback') if status else None)
                    else:
                        actions.submit(name, request_id, client, now,
                                       context=env.get('dds_ownership', {}).get('revision'))
                elif name:
                    replies.emit(name, request_id, 'REJECTED', 'MONITOR_ONLY', False)
            # Process STOP/QUIT before an unsent intent. Recheck eligibility on
            # every tick, without blocking ROS, polling or the control loop.
            if actions.pending is not None:
                denied = action_denial(actions.pending.name, guard.armed, pending_arm, c, env, now)
                sent = actions.tick(client, now, denied,
                                    context=env.get('dds_ownership', {}).get('revision') if env else None)
                if sent:
                    owned = True
                    if sent == 'ARM':
                        pending_arm = now
            state = (c['state'], c.get('target'), c.get('ready'), c.get('navigation_quality'))
            if a.execute and now-checkpoint_time>=1.:
                checkpoint.save(guard);checkpoint_time=now
            log.write(json.dumps(dict(mono=now, command=c, execute=a.execute, gateway=status,
                                      environment=env, localization=ingress.get(now),
                                      ingress_error=ingress.error, dropped_packets=ingress.dropped,
                                      gateway_error=client.error, gateway_connections=client.connection_count,
                                      checkpoint_error=checkpoint.error), allow_nan=False)+'\n')
            if state != last_state or now-last_print >= 2:
                print(json.dumps(dict(command=c, gateway_reason=status.get('reason') if status else client.error,
                                      robot_feedback=status.get('feedback') if status else None,
                                      ownership_reason=env.get('dds_ownership', {}).get('reason') if env else None)), flush=True)
                last_print, last_state = now, state
    finally:
        actions.cancel('SHUTDOWN')
        replies.cancel_waits('SHUTDOWN')
        guard.stop('SHUTDOWN')
        if a.execute and not client.closed:
            client.stop()
            for _ in range(10):
                client.poll(time.monotonic())
                time.sleep(.01)
        client.close()
        if a.execute:checkpoint.save(guard)
        log.write(json.dumps(dict(exit=True, reached=guard.reached, complete=guard.complete))+'\n')
        log.close()
        replies.close()
        if terminal_state is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, terminal_state)
        for executor in executors:
            executor.shutdown()
        for node in nodes:
            node.destroy_node()
        for context in contexts:
            if context.ok():
                context.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
