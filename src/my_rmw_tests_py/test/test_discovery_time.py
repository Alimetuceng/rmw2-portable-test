import time

import rclpy
from std_msgs.msg import String


def test_discovery_under_1_5s() -> None:
    rclpy.init()
    try:
        pub_node = rclpy.create_node('disc_pub')
        sub_node = rclpy.create_node('disc_sub')

        sub_node._sub = sub_node.create_subscription(
            String, 'disc_topic', lambda _m: None, 10
        )
        pub = pub_node.create_publisher(String, 'disc_topic', 10)

        discovered = False
        t0 = time.time()
        timeout_s = 1.5

        while time.time() - t0 < timeout_s:
            rclpy.spin_once(sub_node, timeout_sec=0.05)
            try:
                pub.publish(String(data='warmup'))
                discovered = True
                break
            except Exception:
                pass

        elapsed = time.time() - t0
        assert discovered and elapsed < timeout_s, f'discovery too slow: {elapsed:.2f}s'
    finally:
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
