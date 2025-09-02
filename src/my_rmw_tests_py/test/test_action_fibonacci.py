import time

from example_interfaces.action import Fibonacci
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def _start_server(node: Node) -> None:
    def execute_cb(goal_handle):
        order = int(goal_handle.request.order)
        seq = [0, 1] if order >= 2 else [0][:order]
        for _ in range(max(0, order - len(seq))):
            seq.append(seq[-1] + seq[-2])
            time.sleep(0.001)  # latency
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = seq[:order]
        return result

    node._server = ActionServer(
        node, Fibonacci, 'fib', execute_cb
    )


def test_action_fibonacci_roundtrip() -> None:
    rclpy.init()
    try:
        server_node = rclpy.create_node('fib_server')
        client_node = rclpy.create_node('fib_client')

        _start_server(server_node)

        exec_ = SingleThreadedExecutor()
        exec_.add_node(server_node)
        exec_.add_node(client_node)

        client = ActionClient(client_node, Fibonacci, 'fib')
        t0 = time.time()
        while not client.wait_for_server(timeout_sec=0.1):
            exec_.spin_once(timeout_sec=0.01)
            if time.time() - t0 > 5.0:
                raise AssertionError('Action server timeout')

        goal = Fibonacci.Goal()
        goal.order = 6

        goal_future = client.send_goal_async(goal)
        t0 = time.time()
        while not goal_future.done():
            exec_.spin_once(timeout_sec=0.02)
            if time.time() - t0 > 5.0:
                raise AssertionError('Goal handle timeout')

        goal_handle = goal_future.result()
        assert goal_handle is not None and goal_handle.accepted, 'Goal rejected or None'

        result_future = goal_handle.get_result_async()
        t0 = time.time()
        while not result_future.done():
            exec_.spin_once(timeout_sec=0.02)
            if time.time() - t0 > 8.0:
                raise AssertionError('Result timeout')

        result_msg = result_future.result()
        assert result_msg is not None and result_msg.result is not None, 'None result'

        seq = list(result_msg.result.sequence)
        assert seq == [0, 1, 1, 2, 3, 5]
    finally:
        exec_.remove_node(server_node)
        exec_.remove_node(client_node)
        server_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
