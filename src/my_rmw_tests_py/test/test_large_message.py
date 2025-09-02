import os
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import ByteMultiArray


def test_large_message_1mb_reliable() -> None:
    rclpy.init()
    try:
        pub_node = rclpy.create_node('pub_large')
        sub_node = rclpy.create_node('sub_large')
        got = {'ok': False}

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        def _cb(msg: ByteMultiArray) -> None:
            got['ok'] = (len(msg.data) == 1024 * 1024)

        sub_node._sub = sub_node.create_subscription(
            ByteMultiArray, 'big', _cb, qos
        )
        pub = pub_node.create_publisher(ByteMultiArray, 'big', qos)

        t0 = time.time()
        while time.time() - t0 < 1.0:
            rclpy.spin_once(sub_node, timeout_sec=0.01)

        payload = ByteMultiArray()
        payload.data = bytearray(os.urandom(1024 * 1024))
        pub.publish(payload)

        t0 = time.time()
        while time.time() - t0 < 5.0 and not got['ok']:
            rclpy.spin_once(sub_node, timeout_sec=0.01)

        assert got['ok'], '1MB message not received'
    finally:
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
