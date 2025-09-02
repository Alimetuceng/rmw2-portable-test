import statistics
import time

import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


def test_latency_p95_small_msgs() -> None:
    rclpy.init()
    try:
        pub_node = rclpy.create_node('perf_pub')
        sub_node = rclpy.create_node('perf_sub')

        qos = QoSProfile(
            depth=20,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        times = []
        target = 300

        t_sent = {'t': None}
        got = {'ok': False, 'seq': 0}

        def cb(msg: String) -> None:
            if t_sent['t'] is None:
                return
            times.append(time.time() - t_sent['t'])
            got['seq'] += 1
            if got['seq'] >= target:
                got['ok'] = True

        sub_node._sub = sub_node.create_subscription(
            String, 'perf', cb, qos
        )
        pub = pub_node.create_publisher(String, 'perf', qos)

        # discovery
        t0 = time.time()
        while time.time() - t0 < 1.0:
            rclpy.spin_once(sub_node, timeout_sec=0.01)

        # yayın döngüsü
        for i in range(target):
            t_sent['t'] = time.time()
            pub.publish(String(data=str(i)))
            rclpy.spin_once(sub_node, timeout_sec=0.005)

        t0 = time.time()
        while time.time() - t0 < 2.0 and not got['ok']:
            rclpy.spin_once(sub_node, timeout_sec=0.01)

        assert got['ok'] and len(times) >= target * 0.95, 'Not enough samples received'
        p95 = statistics.quantiles(times, n=20)[18]  # ~p95
        avg = sum(times) / len(times)
        assert p95 < 0.05 and avg < 0.02, f'latency too high: p95={p95:.3f}s avg={avg:.3f}s'
    finally:
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
