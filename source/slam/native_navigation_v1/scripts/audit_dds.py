"""Read-only DDS role/traffic audit; no ASDU connection or command publisher."""
import argparse
import json
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from native_nav.dds_ownership import RosDdsOwnership

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5)
    args = parser.parse_args()
    import rclpy
    rclpy.init(domain_id=0)
    node = rclpy.create_node('wym_native_dds_audit', enable_rosout=False, start_parameter_services=False)
    monitor = RosDdsOwnership(node)
    deadline = time.monotonic()+args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            monitor.snapshot(time.monotonic())
        print(json.dumps(dict(audit=monitor.snapshot(time.monotonic()), graph=monitor.graph(),
                             motion=monitor.audit_state.motion, handler=monitor.audit_state.handler), indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
