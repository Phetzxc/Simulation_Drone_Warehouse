#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import rclpy
from gazebo_msgs.srv import SpawnEntity
from tf_transformations import quaternion_from_euler

def main():
    rclpy.init()

    # สร้าง node
    node = rclpy.create_node('spawn_marker_0')

    # เตรียม client เรียก service `/spawn_entity`
    client = node.create_client(SpawnEntity, '/spawn_entity')

    while not client.wait_for_service(timeout_sec=3.0):
        node.get_logger().info('Waiting for /spawn_entity service...')

    # กำหนดชื่อ marker
    marker_name = 'marker_00'

    # ใช้ get_package_share_directory เพื่อหา path ของ model.sdf
    sdf_file_path = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "models", "aruco_gazebo", marker_name, "model.sdf"
    )

    with open(sdf_file_path, 'r') as f:
        sdf_content = f.read()

    # ตั้งค่า Euler angles และแปลงเป็น quaternion
    roll = 0.0
    pitch = 0.0
    yaw = 3.14
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

    # เตรียมคำขอ
    request = SpawnEntity.Request()
    request.name = marker_name
    request.xml = sdf_content
    request.robot_namespace = ''
    request.initial_pose.position.x = -4.73
    request.initial_pose.position.y = -9.9
    request.initial_pose.position.z = 6.07
    request.initial_pose.orientation.x = qx
    request.initial_pose.orientation.y = qy
    request.initial_pose.orientation.z = qz
    request.initial_pose.orientation.w = qw
    request.reference_frame = 'world'

    # ส่งคำขอ
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Spawn success: {future.result().status_message}")
    else:
        node.get_logger().error("Failed to spawn marker_0")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
