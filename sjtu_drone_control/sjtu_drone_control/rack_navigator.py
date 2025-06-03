#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient


RACK_GOALS = {
    'rack_A': {
        'position': {'x': -3.0, 'y': -9.44, 'z': 0.034},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.999981, 'w': 0.006135},
    },
    'rack_B': {
        'position': {'x': 1.5, 'y': -5.2, 'z': 0.034},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.707, 'w': 0.707},
    },
}


class RackNavigator(Node):

    def __init__(self):
        super().__init__('rack_navigator')

        # ถาม rack
        print("What's rack ?\n- rack_A\n- rack_B")
        rack = input(">> ").strip()

        if rack not in RACK_GOALS:
            self.get_logger().error(f"Invalid rack: {rack}")
            rclpy.shutdown()
            return

        self.rack = rack
        self.goal_pose = RACK_GOALS[rack]

        # สร้าง publisher สำหรับ takeoff
        self.takeoff_pub = self.create_publisher(Empty, '/simple_drone/takeoff', 10)

        # สร้าง action client
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

        self.send_goal()

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = self.goal_pose['position']['x']
        goal_msg.pose.pose.position.y = self.goal_pose['position']['y']
        goal_msg.pose.pose.position.z = self.goal_pose['position']['z']
        goal_msg.pose.pose.orientation.x = self.goal_pose['orientation']['x']
        goal_msg.pose.pose.orientation.y = self.goal_pose['orientation']['y']
        goal_msg.pose.pose.orientation.z = self.goal_pose['orientation']['z']
        goal_msg.pose.pose.orientation.w = self.goal_pose['orientation']['w']

        self.get_logger().info(f'Sending goal to {self.rack}...')
        self._send_future = self.client.send_goal_async(goal_msg)
        self._send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Arrived at {self.rack}, sending takeoff command...')
        self.takeoff()

    def takeoff(self):
        msg = Empty()
        self.takeoff_pub.publish(msg)
        self.get_logger().info('Takeoff message published to /simple_drone/takeoff')


def main(args=None):
    rclpy.init(args=args)
    node = RackNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
