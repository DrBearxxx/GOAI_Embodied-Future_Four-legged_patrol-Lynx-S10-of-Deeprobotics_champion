"""Subscription/discovery only. No robot command publishers or service calls."""
import json,time,collections
import rclpy
from rclpy.qos import qos_profile_sensor_data
from drdds.msg import Steer,StdStatus,JointsDataCmd
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('wym_indoor_sdk_inspect',enable_rosout=False,start_parameter_services=False)
counts=collections.Counter();samples={}
def callback(name,msg):
    counts[name]+=1
    if name=='steer':samples[name]=dict(x=msg.data.x,y=msg.data.y,yaw=msg.data.yaw)
    if name=='key':samples[name]=msg.data
    if name=='charge':samples[name]=dict(state=msg.state,error_code=msg.error_code)
subs=[n.create_subscription(cls,topic,lambda msg,k=name:callback(k,msg),qos_profile_sensor_data) for name,cls,topic in [
    ('steer',Steer,'/STEER'),('key',String,'/GAMEPAD_KEY'),('charge',StdStatus,'/CHARGE_STATUS'),('joint_cmd',JointsDataCmd,'/JOINTS_CMD')]]
stop=time.monotonic()+6
while time.monotonic()<stop:rclpy.spin_once(n,timeout_sec=.02)
topics=n.get_topic_names_and_types()
def endpoints(topic,publish):
    f=n.get_publishers_info_by_topic if publish else n.get_subscriptions_info_by_topic
    return [dict(node=v.node_name,namespace=v.node_namespace,type=v.topic_type,qos=str(v.qos_profile)) for v in f(topic)]
print(json.dumps(dict(counts=counts,samples=samples,topics=topics,endpoints={topic:dict(publishers=endpoints(topic,True),subscribers=endpoints(topic,False)) for topic in ['/NAV_CMD','/MOTION_INFO','/STEER','/JOINTS_CMD','/GAIT','/CHARGE_STATUS']}),indent=2),flush=True)
n.destroy_node();rclpy.shutdown()
