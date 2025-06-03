#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point, Quaternion
import math

def euler_to_quaternion(roll, pitch, yaw):
    """
    แปลง Euler angles (roll, pitch, yaw) เป็น Quaternion
    สูตรจาก ROS wiki
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
         math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
         math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_drone')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    if len(sys.argv) < 9:
        node.get_logger().error("Usage: spawn_drone.py <urdf_xml> <namespace> <x> <y> <z> <roll> <pitch> <yaw>")
        return

    content = sys.argv[1]
    namespace = sys.argv[2]

    # แปลง argument string เป็น float
    x = float(sys.argv[3])
    y = float(sys.argv[4])
    z = float(sys.argv[5])
    roll = float(sys.argv[6])
    pitch = float(sys.argv[7])
    yaw = float(sys.argv[8])

    # แปลง Euler angles เป็น quaternion
    quat = euler_to_quaternion(roll, pitch, yaw)

    # สร้าง Pose สำหรับตำแหน่งและมุมหมุน
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = quat

    req = SpawnEntity.Request()
    req.name = namespace
    req.xml = content
    req.robot_namespace = namespace
    req.reference_frame = "world"
    req.initial_pose = pose  # ใส่ตำแหน่งและมุมที่ spawn

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result: ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().error('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
