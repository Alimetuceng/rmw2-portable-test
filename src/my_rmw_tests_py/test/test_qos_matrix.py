import itertools
import time

import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


def _pub_sub_once(qos: QoSProfile) -> bool:
    pub_node = rclpy.create_node('pub_node_qos')
    sub_node = rclpy.create_node('sub_node_qos')
    got = {'ok': False}

    def _cb(_msg):  # noqa: D401
        got['ok'] = True

    sub_node._sub = sub_node.create_subscription(
        String, 'qos_topic', _cb, qos
    )
    pub = pub_node.create_publisher(String, 'qos_topic', qos)

    t0 = time.time()
    while time.time() - t0 < 1.0:
        rclpy.spin_once(sub_node, timeout_sec=0.01)

    pub.publish(String(data='x'))

    t0 = time.time()
    while time.time() - t0 < 2.0 and not got['ok']:
        rclpy.spin_once(sub_node, timeout_sec=0.01)

    pub_node.destroy_node()
    sub_node.destroy_node()
    return got['ok']


def test_qos_matrix() -> None:
    rclpy.init()
    try:
        reliabilities = [ReliabilityPolicy.BEST_EFFORT, ReliabilityPolicy.RELIABLE]
        histories = [(HistoryPolicy.KEEP_LAST, 1), (HistoryPolicy.KEEP_LAST, 10)]
        durabilities = [DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL]

        for rel, (hist, depth), dur in itertools.product(reliabilities, histories, durabilities):
            qos = QoSProfile(depth=depth, reliability=rel, history=hist, durability=dur)
            assert _pub_sub_once(qos), f'QoS failed: {rel.name}, {hist.name}/{depth}, {dur.name}'
    finally:
        rclpy.shutdown()
