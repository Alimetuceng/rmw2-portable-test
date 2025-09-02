import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


def test_transient_local_late_joiner_receives() -> None:
    rclpy.init()
    try:
        topic = 'tl_topic'
        pub_node = rclpy.create_node('tl_pub')
        sub_node = rclpy.create_node('tl_sub')

        exec_ = SingleThreadedExecutor()
        exec_.add_node(pub_node)
        exec_.add_node(sub_node)

        tl_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        vol_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        pub = pub_node.create_publisher(String, topic, tl_qos)

        t0 = time.time()
        while time.time() - t0 < 0.6:
            exec_.spin_once(timeout_sec=0.01)

        pub.publish(String(data='latched'))

        t0 = time.time()
        while time.time() - t0 < 0.8:
            exec_.spin_once(timeout_sec=0.01)

        got_tl = {'msg': None}
        sub_node._sub_tl = sub_node.create_subscription(
            String,
            topic,
            lambda m: got_tl.__setitem__('msg', m.data),
            tl_qos,
        )

        t0 = time.time()
        timeout = 3.5
        while time.time() - t0 < timeout and got_tl['msg'] is None:
            exec_.spin_once(timeout_sec=0.02)

        assert got_tl['msg'] == 'latched', (
            'Transient-local late-joiner did not get the previous sample'
        )

        got_vol = {'msg': None}
        sub_node._sub_vol = sub_node.create_subscription(  # noqa: SLF001
            String,
            topic,
            lambda m: got_vol.__setitem__('msg', m.data),
            vol_qos,
        )

        t0 = time.time()
        while time.time() - t0 < 0.7:
            exec_.spin_once(timeout_sec=0.02)

        assert got_vol['msg'] is None, (
            'Volatile late-joiner unexpectedly received a past sample'
        )

        pub_node.destroy_publisher(pub)

    finally:
        exec_.remove_node(pub_node)
        exec_.remove_node(sub_node)
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
