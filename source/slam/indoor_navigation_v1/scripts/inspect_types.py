"""Read-only SDK type discovery; requests only GetTypeDescription services."""
import json,time
import rclpy
from type_description_interfaces.srv import GetTypeDescription
from rosidl_runtime_py.convert import message_to_ordereddict
rclpy.init();n=rclpy.create_node('wym_indoor_type_inspect',enable_rosout=False,start_parameter_services=False)
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
services=[name for name,types in n.get_service_names_and_types() if 'type_description_interfaces/srv/GetTypeDescription' in types]
print('TYPE_SERVICES',services,flush=True)
for topic in ['/NAV_CMD','/MOTION_INFO','/MOTION_STATUS','/NAV_STATUS','/GAIT']:
    eps=n.get_publishers_info_by_topic(topic)+n.get_subscriptions_info_by_topic(topic)
    for ep in eps:
        print(topic,ep.node_name,ep.topic_type,str(ep.topic_type_hash),flush=True)
        if ep.node_name.startswith('_'):continue
        service=ep.node_namespace.rstrip('/')+'/'+ep.node_name+'/get_type_description'
        if service not in services:continue
        c=n.create_client(GetTypeDescription,service)
        req=GetTypeDescription.Request();req.type_name=ep.topic_type;req.type_hash=str(ep.topic_type_hash);req.include_type_sources=True
        future=c.call_async(req);rclpy.spin_until_future_complete(n,future,timeout_sec=2)
        if future.done() and future.result():print(json.dumps(message_to_ordereddict(future.result())),flush=True)
        n.destroy_client(c)
n.destroy_node();rclpy.shutdown()
