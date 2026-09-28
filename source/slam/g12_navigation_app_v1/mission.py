"""Navigation/manual arbitration and feedback-confirmed policy selection."""
import math
import copy
import uuid
import json
import os
from pathlib import Path
from navigation import Navigator
from planning import Plans, OFFICIAL, MANUAL, NATIVE_POLICIES, WAYPOINT_POLICIES
from route_recording import RouteRecorder

RC_TIMEOUT = .35  # Missing input produces zero; it is not an enable gate.


class Mission:
    def __init__(self,routes,bridge,presets=None):
        self.plans=Plans(routes,presets)
        self.navigator=Navigator({k:self.plans.route(k) for k in self.plans.sources})
        self.recorder=RouteRecorder(Path(presets).parent/'route-recordings' if presets else None)
        self.settings_path=Path(presets).with_name('navigation-settings.json') if presets else None
        if self.settings_path and self.settings_path.exists():
            self.navigator.cruise_mps=self.validate_speed(json.loads(self.settings_path.read_text())['cruise_mps'])
        self.bridge=bridge; self.owner='paused'; self.override=None
        self.rc=None; self.rc_seq=-1; self.rc_session=None; self.rc_at=-1e9
        self.transition=None; self.last={}; self.reason='PAUSED';self.execution=False
        self.manual_policy='basic'; self.manual_id=str(uuid.uuid4())
        self.native_input_kind='velocity'
        self.pending_stand=None
        self.live_policy=None
        self.policy_selection_pending=False
        self.resume_pending=False

    def pause(self,reason='PAUSED'):
        self.navigator.pause(reason); self.owner='paused'; self.transition=None
        self.pending_stand=None
        self.live_policy=None
        self.reason=reason;self.resume_pending=False;self.bridge.stop()

    def keepalive(self,run_id,now):
        self.navigator.keepalive(run_id,now)

    def restore_paused(self,saved):
        """Restore an upgrade checkpoint without enabling or sending commands."""
        nav=self.navigator;name=saved['route_id'];route=saved['route']
        if name not in self.plans.sources or not route.get('waypoints'):raise ValueError('INVALID_RESUME_ROUTE')
        count=len(route['waypoints']);target=saved['target_index']
        if type(target) is not int or not 0<=target<count:raise ValueError('INVALID_RESUME_TARGET')
        reached=saved.get('reached_indices',[]);skipped=saved.get('skipped_indices',[])
        if any(type(v) is not int or not 0<=v<count for v in reached+skipped):raise ValueError('INVALID_RESUME_PROGRESS')
        override=saved.get('override')
        manual_target=saved.get('manual_target')
        if manual_target is not None and (type(manual_target) is not int or manual_target!=target):raise ValueError('INVALID_RESUME_MANUAL_TARGET')
        if override is not None and override not in NATIVE_POLICIES:raise ValueError('INVALID_RESUME_POLICY')
        for e in route['edges']:
            if e.get('source_route') not in self.plans.presets or not 0<=e.get('source_index',-1)<len(self.plans.presets[e['source_route']]):
                raise ValueError('INVALID_RESUME_SEGMENT')
        nav.routes[name]=copy.deepcopy(route);nav.select(name);nav.target=target
        nav.reached=list(reached);nav.skipped=list(skipped)
        nav.manual_target=manual_target;nav.manual_progress=copy.deepcopy(saved.get('manual_progress'))
        if nav.manual_progress:
            completed=list(range(nav.manual_progress['target_index']))
            nav.reached=sorted(set(nav.reached+completed));nav.skipped=[i for i in nav.skipped if i not in nav.reached]
            nav.manual_progress.pop('manually_skipped_indices',None)
            nav.manual_progress['manually_completed_indices']=completed
        self.override=override;self.manual_policy=saved.get('manual_policy','basic')
        self.native_input_kind=saved.get('input_kind','velocity')
        self.owner='paused';self.execution=False;self.reason='PAUSED'
        if saved.get('temporary_waypoints'):nav.temporary.restore(saved['temporary_waypoints'])

    def operator(self,body,now):
        values=body.get('axes'); seq=body.get('sample_seq'); age=body.get('sample_age_ms')
        session=body.get('session')
        if not isinstance(session,str) or len(session)>80:raise ValueError('INVALID_RC_SESSION')
        if self.rc_session!=session:
            if self.rc_session is not None and self.owner!='paused':self.pause('OPERATOR_SESSION_CHANGED')
            self.rc_seq=-1;self.rc_session=session;self.rc=None
        if body.get('stop') is True:self.pause('OPERATOR_STOP');return
        if (not isinstance(values,list) or len(values)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>1 for v in values)
            or type(seq) is not int or type(age) not in (float,int) or not math.isfinite(age)):
            raise ValueError('INVALID_RC_SAMPLE')
        if seq==self.rc_seq and values==self.rc and body.get('fresh') and 0<=age<=200 and 0<=now-self.rc_at<=RC_TIMEOUT:
            return  # polling may be faster than RC sampling; never renew its timestamp
        if not body.get('fresh') or not 0<=age<=200 or seq<=self.rc_seq:
            self.rc=None;return
        self.rc=values; self.rc_seq=seq;self.rc_at=now-age*.001
        if self.owner=='auto' and max(map(abs,values))>.12:
            # A deliberate stick deflection takes authority from navigation.
            policy=self.desired_policy()
            self.navigator.pause('MANUAL_TAKEOVER');self.owner='manual'
            self.manual_policy=policy;self.transition=None;self.pending_stand=None
            self.last={};self.live_policy=None;self.resume_pending=False
            self.policy_selection_pending=False
            self.native_input_kind='native_axes'
            self.reason='MANUAL_TAKEOVER'

    def desired_policy(self):
        if self.owner=='manual':return self.manual_policy
        return self.override or self.plans.policy(self.navigator.route,self.navigator.target)

    def policy_mode(self):
        return 'route_preset' if self.override is None else 'override'

    def manual_choice(self,body):
        policy=body.get('policy',self.manual_policy)
        if policy not in MANUAL:raise ValueError('UNKNOWN_POLICY')
        session=body.get('operator_session')
        if session is not None:
            if not isinstance(session,str) or len(session)>80:raise ValueError('INVALID_RC_SESSION')
            if session!=self.rc_session:
                self.rc_session=session;self.rc_seq=-1;self.rc=None;self.rc_at=-1e9
        return policy

    def activate_manual(self,policy,execute):
        self.manual_policy=policy
        self.owner='manual';self.execution=execute;self.native_input_kind='native_axes'
        self.manual_id=str(uuid.uuid4());self.last={};self.reason='MANUAL'
        self.policy_selection_pending=False;self.resume_pending=False

    @staticmethod
    def validate_speed(value):
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
            raise ValueError('INVALID_CRUISE_SPEED')
        return float(value)

    def command(self,body,solution,now):
        action=body.get('action');nav=self.navigator;rid=body.get('request_id',str(uuid.uuid4()))
        if isinstance(action,str) and action.startswith('temporary_'):
            if body.get('route')!=nav.route_id:raise ValueError('TEMPORARY_ROUTE_CHANGED_REFRESH')
            result=nav.edit_temporary(action[len('temporary_'):],body,solution)
            return {'temporary_waypoints':result}
        if isinstance(action,str) and action.startswith('recording_'):
            policy=self.desired_policy() if self.owner=='auto' else self.manual_policy
            result=self.recorder.command(body,solution,now,policy,self.bridge.state(),self.plans.recorded)
            if action=='recording_save':
                name=self.recorder.saved_route
                nav.routes[name]=self.plans.load_recorded(name)
                result['saved_route']=name;result['route_name']=nav.routes[name]['display_name']
            return result
        if isinstance(action,str) and action.startswith('calibration_'):
            calibration=self.plans.calibration
            if action!='calibration_apply':return calibration.prepare(body,solution,now)
            # Applying is an explicit operator action. Draft capture/editing
            # never changes control ownership or the active path.
            result=calibration.apply(body)
            self.pause('ROUTE_CALIBRATED');self.plans.preview=None
            routes={k:self.plans.route(k) for k in self.plans.sources}
            fresh=routes[nav.route_id];by_id={p['id']:p for p in fresh['waypoints']}
            route=copy.deepcopy(nav.route)
            route['waypoints']=[copy.deepcopy(by_id.get(p.get('id'),p)) for p in route['waypoints']]
            for i,e in enumerate(route['edges']):
                route['edges'][i]=copy.deepcopy(routes[e['source_route']]['edges'][e['source_index']])
                if route['waypoints'][i].get('id')=='reentry' and route['edges'][i].get('control_points'):
                    from route_geometry import edge_points,project_polyline
                    source=routes[e['source_route']];projection=project_polyline(route['waypoints'][i]['xyz'],edge_points(source,e['source_index']))
                    route['edges'][i]['control_points']=route['edges'][i]['control_points'][projection['index']:]
            route['calibration_revision']=calibration.data['revision']
            nav.routes=routes;nav.route=route;nav.entry=None;nav.trajectory_key=None;self.last={}
            return result
        if action=='set_speed':
            speed=self.validate_speed(body.get('cruise_mps'))
            if self.settings_path:
                self.settings_path.parent.mkdir(parents=True,exist_ok=True)
                tmp=self.settings_path.with_suffix('.tmp')
                tmp.write_text(json.dumps(dict(cruise_mps=speed)),encoding='utf-8');os.replace(tmp,self.settings_path)
            nav.cruise_mps=speed
            return {'cruise_mps':speed}
        if action=='pause':self.pause();return {}
        if action=='select':
            name=body.get('route')
            if name not in self.plans.sources:raise ValueError('UNKNOWN_ROUTE')
            self.pause('ROUTE_SELECTED');nav.routes[name]=self.plans.route(name);nav.select(name)
            self.plans.preview=None;return {}
        if action=='set_policy':
            self.pause('PRESET_UPDATED')
            self.plans.set_policy(body.get('route'),body.get('segments'),body.get('policy'));return {}
        if action=='set_progress':
            # A replan is an execution route, not a replacement for the saved
            # route. The progress picker can select any original waypoint.
            original=body.get('route_scope')=='preset'
            route=self.plans.route(nav.route_id) if original else nav.route
            index=body.get('target_index');points=route['waypoints']
            if type(index) is not int or not 0<=index<len(points):raise ValueError('INVALID_PROGRESS_TARGET')
            if body.get('route')!=nav.route_id or body.get('waypoint_id')!=points[index]['id']:
                raise ValueError('PROGRESS_ROUTE_CHANGED_REFRESH')
            self.pause('PROGRESS_SET')
            if original:
                nav.routes[nav.route_id]=route;nav.select(nav.route_id)
            nav.set_progress(index,rid,now)
            self.plans.preview=None;self.last={}
            return {'progress':copy.deepcopy(nav.manual_progress)}
        if action=='override':
            policy=body.get('policy')
            if policy not in MANUAL and policy is not None:raise ValueError('UNKNOWN_POLICY')
            if self.owner!='auto':self.native_input_kind='native_axes'
            if policy in WAYPOINT_POLICIES:
                self.pause('WAYPOINT_MANUAL_ONLY');self.manual_policy=policy;self.override=None
                self.bridge.action('select:'+policy,rid,now,input_kind='waypoint_axes')
            elif self.owner=='manual':
                self.manual_policy=policy or 'basic_normal';self.override=policy;self.transition=None
            else:self.override=policy;self.transition=None
            self.policy_selection_pending=self.owner=='auto'
            if self.owner=='paused' and policy in NATIVE_POLICIES:
                self.manual_policy=policy;self.bridge.action('select:'+policy,rid,now,input_kind=self.input_kind(policy))
            return {}
        if action in ('stand','lie','damping','wake'):
            auto_manual=action=='stand' and body.get('manual_after_stand') is True
            policy=self.manual_choice(body) if auto_manual else None
            self.pause('POSTURE_ACTION');self.bridge.action(action,rid,now)
            if auto_manual:
                self.activate_manual(policy,True);self.pending_stand=rid
            return {}
        if action in ('start','shadow'):
            unsupported={self.plans.policy(nav.route,i) for i in range(nav.target,len(nav.route['waypoints']))}-set(NATIVE_POLICIES)
            if unsupported:raise ValueError('录制保留了 waypoint 模式；该模式目前只接入人工摇杆，请在分段预设中为这些路段选择原生模式后导航')
            if self.rc is not None and now-self.rc_at<.3 and any(self.rc):raise ValueError('CENTER_STICKS_TO_START')
            if action=='start' and not self.bridge.client:raise ValueError('HARDWARE_GATEWAY_NOT_CONNECTED')
            nav.start(solution,now,execute=action=='start')
            # Every successful start/resume enters the route's presets. A
            # rejected start must leave the operator's existing choice intact.
            self.override=None;self.live_policy=None;self.policy_selection_pending=False
            self.native_input_kind='velocity'
            self.owner='auto';self.execution=action=='start';self.transition=None;self.reason='STARTING'
            # Manual authorization and pending posture feedback belong to the
            # previous owner, never to this new navigation run.
            self.last={};self.pending_stand=None;self.resume_pending=True
            self.plans.preview=None
            return {'entry':nav.entry}
        if action in ('manual','manual_shadow'):
            if action=='manual' and not self.bridge.client:raise ValueError('HARDWARE_GATEWAY_NOT_CONNECTED')
            policy=self.manual_choice(body)
            if self.owner=='manual' and self.execution==(action=='manual'):
                self.manual_policy=policy
                self.transition=None
            else:
                self.pause('MANUAL_TAKEOVER');self.activate_manual(policy,action=='manual')
            return {}
        if action=='replan':
            self.pause('REPLAN_PREVIEW')
            return {'preview':self.plans.propose(nav.route_id,body.get('goal'),body.get('blocked_edges',[]),solution,now)}
        if action=='apply_plan':
            self.pause('PLAN_APPLIED_RESUME_REQUIRED')
            route=self.plans.apply(body.get('plan_id'),solution,now)
            nav.routes[nav.route_id]=route;nav.select(nav.route_id)
            nav.reason='PLAN_APPLIED_RESUME_REQUIRED';return {}
        if action=='discard_plan':self.plans.preview=None;return {}
        raise ValueError('UNKNOWN_MISSION_ACTION')

    def input_kind(self,policy):
        if policy in WAYPOINT_POLICIES:return 'waypoint_axes'
        return 'velocity' if self.owner=='auto' else self.native_input_kind

    def _ensure_policy(self,policy,now):
        if not self.execution:return True
        kind=self.input_kind(policy)
        if self.pending_stand:
            status=self.bridge.state();action=status.get('last_action') or {}
            if action.get('request_id')!=self.pending_stand:return False
            if action.get('state')=='UNCONFIRMED_NO_RETRY':raise ValueError(action.get('reason','STAND_REQUEST_FAILED'))
            if action.get('state')!='RECEIVED':return False
            self.pending_stand=None
            if status.get('requested')==policy and status.get('input_kind')==kind:
                self.transition=dict(policy=policy,input_kind=kind,since=now,run_sent=False)
        status=self.bridge.state()
        if (policy in NATIVE_POLICIES and status.get('requested') in NATIVE_POLICIES and status.get('enabled')
                and status.get('input_kind')==kind and self.bridge.ready()):
            target=(policy,kind)
            if self.policy_selection_pending or (target!=self.live_policy if self.live_policy is not None else status.get('requested')!=policy):
                self.bridge.action('select:'+policy,str(uuid.uuid4()),now,input_kind=kind)
            self.policy_selection_pending=False
            self.live_policy=target;self.transition=None
            return True
        self.live_policy=None
        if self.bridge.policy_ready(policy,kind) and self.bridge.ready():
            self.transition=None;self.policy_selection_pending=False;return True
        if self.transition is None or self.transition['policy']!=policy or self.transition['input_kind']!=kind:
            self.transition=dict(policy=policy,input_kind=kind,since=now,run_sent=False)
            if not self.bridge.policy_ready(policy,kind):
                self.bridge.action('select:'+policy,str(uuid.uuid4()),now,input_kind=kind)
            self.policy_selection_pending=False
        if self.owner=='auto' and now-self.transition['since']>8:
            self.pause('POLICY_SWITCH_TIMEOUT_EXPLICIT_RESUME');return False
        if self.bridge.policy_ready(policy,kind) and not self.transition['run_sent']:
            self.bridge.action('run',str(uuid.uuid4()),now,input_kind=kind);self.transition['run_sent']=True
        return False

    def step(self,solution,perception,now,pending=False):
        nav=self.navigator
        if pending and self.owner!='paused':self.pause('RELOCALIZATION_REQUESTED')
        policy=self.desired_policy()
        if (self.execution and self.owner!='paused' and self.last.get('state')=='POLICY_SWITCH_PAUSE'
                and self.bridge.state().get('policy_switch_error')):
            self.pause('POLICY_SWITCH_FAILED')
        # Repeated gateway reconnects never silently re-arm a running mission.
        if self.execution and self.owner!='paused' and self.pending_stand is None and self.transition is None and self.last.get('motion_authorized') and not self.bridge.ready():
            self.pause('CONTROL_LINK_LOST')
        if self.owner=='auto':
            # Establish the input interface before advancing route progress or
            # integrating PID. Handover can finish at a different pose from the
            # one captured when the user pressed Resume.
            try:ready=self._ensure_policy(policy,now)
            except ValueError as exc:self.pause(str(exc));ready=False
            switching=self.execution and self.bridge.state().get('policy_switch_paused')
            hold='POLICY_SWITCH_PAUSE' if switching else None if ready else 'WAIT_POLICY_FEEDBACK'
            if self.owner=='auto' and self.resume_pending and hold is None:
                if (solution and solution.get('mode') in ('TRACKING','DEGRADED','ODOM_BRIDGE','PREDICT_ONLY')
                        and 0<=now-solution['estimate_mono']<=.25):
                    nav.reconnect(solution)
                    self.resume_pending=False
                else:hold='HOLD_LOCALIZATION'
            out=nav.step(solution,perception,now,pending,control_ready=True,
                control_hold=hold,
                policy_for_target=lambda target:self.override or self.plans.policy(nav.route,target))
            if not nav.active:self.pause(out['state'])
        else:
            if self.owner=='manual':nav.temporary.observe_manual(solution)
            snapshot=nav.step(solution,perception,now,pending,control_ready=True)
            out=dict(snapshot,active=self.owner=='manual',execution_requested=self.execution,
                vx=0.,vy=0.,wz=0.,state=self.reason,motion_authorized=False,
                run_id=self.manual_id if self.owner=='manual' else nav.run_id)
            if self.owner=='manual':
                fresh=self.rc is not None and 0<=now-self.rc_at<=RC_TIMEOUT
                values=self.rc if fresh else [0.,0.,0.]
                out.update(axes=list(values),vx=0.,vy=0.,wz=0.,
                    state='MANUAL' if fresh else 'MANUAL_WAIT_INPUT',rc_fresh=fresh,speed_limit_mps=None)
                out['motion_authorized']=self.execution
        try:
            # An arrival can select a new segment during this tick.
            policy=self.desired_policy()
            if self.owner!='paused' and not self._ensure_policy(policy,now):
                out.update(vx=0.,vy=0.,wz=0.,axes=[0.,0.,0.],state='WAIT_POLICY_FEEDBACK',motion_authorized=False)
            if self.owner!='paused' and self.execution and self.bridge.state().get('policy_switch_paused'):
                out.update(vx=0.,vy=0.,wz=0.,axes=[0.,0.,0.],state='POLICY_SWITCH_PAUSE',motion_authorized=False)
            if self.owner=='paused':out.update(active=False,vx=0.,vy=0.,wz=0.,axes=[0.,0.,0.],motion_authorized=False)
        except ValueError as exc:
            self.pause(str(exc));out.update(active=False,vx=0.,vy=0.,wz=0.,axes=[0.,0.,0.],state=str(exc),motion_authorized=False)
        self.last=dict(out,input_kind=self.input_kind(policy),owner=self.owner,policy=policy,policy_name=MANUAL[policy],override=self.override,
            policy_mode=self.policy_mode(),preset_policy=self.plans.policy(nav.route,nav.target),
            presets_revision=self.plans.revision,transition=self.transition,resume_pending=self.resume_pending)
        self.last['temporary_waypoints']=nav.temporary.state()
        if nav.temporary.pending:self.last['next_temporary']=nav.temporary.pending[0].copy()
        returning=(nav.entry or {}).get('temporary_return_index')
        if returning is not None:self.last['temporary_return']=copy.deepcopy(nav.route['waypoints'][returning])
        self.bridge.send(self.last,now)
        self.recorder.tick(solution,now,policy if self.owner=='auto' else self.manual_policy,self.bridge.state())
        return self.last

    def state(self,now=None,include_catalog=True):
        result=dict(owner=self.owner,manual_policy=self.manual_policy,override=self.override,route_recording=self.recorder.state(),
            policy_mode=self.policy_mode(),preset_policy=self.plans.policy(self.navigator.route,self.navigator.target),
            presets_revision=self.plans.revision,policies=OFFICIAL,manual_policies=MANUAL,override_policies=NATIVE_POLICIES,cruise_mps=self.navigator.cruise_mps,
            route_id=self.navigator.route_id,target_index=self.navigator.target,
            calibration_revision=self.plans.calibration.data['revision'],
            catalog_key=':'.join(map(str,(self.navigator.route_id,self.navigator.run_id,self.plans.revision,
                self.plans.calibration.data['revision'],(self.plans.preview or {}).get('id','')))),
            manual_target=self.navigator.manual_target,manual_progress=self.navigator.manual_progress,
            temporary_waypoints=self.navigator.temporary.state(),
            rc_fresh=self.rc is not None and (now is None or 0<=now-self.rc_at<=RC_TIMEOUT),
            rc_age_s=None if now is None else max(0.,now-self.rc_at),rc_seq=self.rc_seq,reason=self.reason)
        if include_catalog:result.update(catalog=self.plans.catalog(),preview=self.plans.preview,current_route=self.navigator.route)
        return result
