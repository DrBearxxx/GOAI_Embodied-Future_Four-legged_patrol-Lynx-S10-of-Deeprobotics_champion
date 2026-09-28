#!/bin/bash
# Subscription only: no mode, joint, key, velocity or authority publishers.
set -eo pipefail
source /home/wym/s10_goai_ws/env.sh
source /home/wym/s10_indoor_navigation_v1/sdk_ws/install/local_setup.bash
python3 - <<'PY'
import collections,json,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data as Q
from drdds.msg import MotionInfo
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('wym_standing_retest_preflight',enable_rosout=False,start_parameter_services=False)
counts=collections.Counter();sample={}
def motion(m):
    counts['motion']+=1
    sample['motion']=dict(state=int(m.data.motion_state.state),gait=int(m.data.gait_state.gait))
subs=[n.create_subscription(MotionInfo,'/MOTION_INFO',motion,Q),
      n.create_subscription(String,'/wym/goai/status',lambda m:sample.update(goai=json.loads(m.data)),Q)]
start=time.monotonic()
while time.monotonic()-start<6:rclpy.spin_once(n,timeout_sec=.01)
def endpoint(e):return dict(node=e.node_name,namespace=e.node_namespace,gid=bytes(e.endpoint_gid).hex())
report=dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),seconds=time.monotonic()-start,
 counts=dict(counts),samples=sample,publishers={t:[endpoint(e) for e in n.get_publishers_info_by_topic(t)]
 for t in ['/JOINTS_CMD','/MOTION_INFO','/STEER','/wym/goai/nav_cmd_vel','/wym/goai/nav_authority']},
 joint_receivers=[endpoint(e) for e in n.get_subscriptions_info_by_topic('/JOINTS_CMD') if e.node_name!=n.get_name()],
 commands_sent=0)
out=Path('/home/wym/s10_indoor_navigation_v1/results')/f'standing-preflight-{time.time_ns()}.json'
out.write_text(json.dumps(report,indent=2)+'\n');print(str(out));print(json.dumps(report,indent=2))
n.destroy_node();rclpy.shutdown()
PY
