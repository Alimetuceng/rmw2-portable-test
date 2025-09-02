import time  # stdlib önce, sonra üçüncü parti

import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


def test_pub_sub_best_effort() -> None:
    rclpy.init()
    try:
        pub_node = rclpy.create_node('pub_node')
        sub_node = rclpy.create_node('sub_node')

        got = {'msg': None}
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        # GC'ye gitmemesi için subscription referansını node üzerinde tutalım
        sub_node._sub = sub_node.create_subscription(  # noqa: SLF001 (private attr usage)
            String,
            'chatter',
            lambda m: got.__setitem__('msg', m.data),
            qos,
        )
        pub = pub_node.create_publisher(String, 'chatter', qos)

        # discovery için biraz bekle (Fast DDS için 1.0s daha stabil)
        t0 = time.time()
        while time.time() - t0 < 1.0:
            rclpy.spin_once(sub_node, timeout_sec=0.01)

        pub.publish(String(data='hello'))

        # mesajın gelmesini en fazla 2 sn bekle
        t0 = time.time()
        while time.time() - t0 < 2.0 and got['msg'] is None:
            rclpy.spin_once(sub_node, timeout_sec=0.01)

        assert got['msg'] == 'hello'
    finally:
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
