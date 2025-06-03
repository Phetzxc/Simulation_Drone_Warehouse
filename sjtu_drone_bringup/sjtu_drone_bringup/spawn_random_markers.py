#!/usr/bin/env python3

import os
import random
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory
import yaml
import matplotlib.pyplot as plt

def main():
    rclpy.init()
    node = rclpy.create_node('spawn_random_markers')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    while not client.wait_for_service(timeout_sec=3.0):
        node.get_logger().info('Waiting for /spawn_entity service...')

    positions = [
        (-4.736, -9.9, 6.07), (-4.736, -9.9, 5.51), (-4.736, -9.9, 4.70), (-4.736, -9.9, 3.864),
        (-4.736, -9.9, 2.71), (-4.736, -9.9, 1.7), (-4.736, -9.9, 1.17), (-4.736, -9.9, 0.43),
        (-4.736, -5.437, 6.07), (-4.736, -5.437, 5.51), (-4.736, -5.437, 4.70), (-4.736, -5.437, 3.864),
        (-4.736, -5.437, 2.71), (-4.736, -5.437, 1.7),
        (-4.736, -1.06, 6.07), (-4.736, -1.06, 5.51), (-4.736, -1.06, 4.70), (-4.736, -1.06, 3.864),
        (-4.736, -1.06, 2.71), (-4.736, -1.06, 1.7),
        (-4.736, 3.43, 6.07), (-4.736, 3.43, 5.51), (-4.736, 3.43, 4.70), (-4.736, 3.43, 3.864),
        (-4.736, 3.43, 2.71), (-4.736, 3.43, 1.7),
        (-4.736, 8.0, 6.07), (-4.736, 8.0, 5.51), (-4.736, 8.0, 4.70), (-4.736, 8.0, 3.864),
        (-4.736, 8.0, 2.71), (-4.736, 8.0, 1.7),
        (-4.8, -8.6, 4.7), (-4.8, -6.72, 4.7), (-4.8, -4.15, 4.7), (-4.8, -2.4, 4.7),
        (-4.8, 0.0, 4.7), (-4.8, 2.0, 4.7), (-4.8, 4.7, 4.7), (-4.8, 6.7, 4.7),
        (-4.8, -8.6, 1.7), (-4.8, -6.72, 1.7), (-4.8, -4.15, 1.7), (-4.8, -2.4, 1.7),
        (-4.8, 0.0, 1.7), (-4.8, 2.0, 1.7), (-4.8, 4.7, 1.7), (-4.8, 6.7, 1.7),
    ]

    num_markers = len(positions)
    marker_indices = random.sample(range(61), num_markers)

    roll, pitch, yaw = 0.0, 0.0, 3.14
    qx, qy, qz, qw = [float(i) for i in quaternion_from_euler(roll, pitch, yaw)]

    marker_log = []
    y_coords = []
    z_coords = []
    ids = []

    for i, (x, y, z) in zip(marker_indices, positions):
        marker_id = f"{i:02d}"
        marker_name = f"marker_{marker_id}"

        sdf_file_path = os.path.join(
            get_package_share_directory("sjtu_drone_description"),
            "models", "aruco_gazebo", marker_name, "model.sdf"
        )

        if not os.path.exists(sdf_file_path):
            node.get_logger().error(f"Model {marker_name} not found at {sdf_file_path}")
            continue

        with open(sdf_file_path, 'r') as f:
            sdf_content = f.read()

        request = SpawnEntity.Request()
        request.name = marker_name
        request.xml = sdf_content
        request.robot_namespace = ''
        request.initial_pose.position.x = float(x)
        request.initial_pose.position.y = float(y)
        request.initial_pose.position.z = float(z)
        request.initial_pose.orientation.x = qx
        request.initial_pose.orientation.y = qy
        request.initial_pose.orientation.z = qz
        request.initial_pose.orientation.w = qw
        request.reference_frame = 'world'

        node.get_logger().info(f"Spawning {marker_name} at ({x}, {y}, {z})")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            node.get_logger().info(f"✅ {marker_name} spawned: {future.result().status_message}")
            marker_log.append({
                'id': marker_id,
                'name': marker_name,
                'position': {'x': float(x), 'y': float(y), 'z': float(z)},
                'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}
            })
            y_coords.append(y)
            z_coords.append(z)
            ids.append(marker_id)
        else:
            node.get_logger().error(f"❌ Failed to spawn {marker_name}")

    # --- Save to YAML and PNG ---
    config_path = os.path.join(
        get_package_share_directory("sjtu_drone_control"),
        "config", "aruco_gazebo"
    )
    os.makedirs(config_path, exist_ok=True)

    yaml_path = os.path.join(config_path, "spawned_marker_log.yaml")
    with open(yaml_path, 'w') as f:
        yaml.dump(marker_log, f, sort_keys=False)

    image_path = os.path.join(config_path, "marker_layout.png")
    plt.figure(figsize=(10, 12))
    plt.scatter(y_coords, z_coords, c='blue', s=50)

    for i, (y, z, marker_id, mdata) in enumerate(zip(y_coords, z_coords, ids, marker_log)):
        label_offset = 0.25 if i % 2 == 0 else -0.3
        x_pos = mdata["position"]["x"]
        y_pos = mdata["position"]["y"]
        z_pos = mdata["position"]["z"]
        plt.text(
            y, z + label_offset,
            f"{marker_id}\n({x_pos:.1f},{y_pos:.1f},{z_pos:.1f})",
            fontsize=7, ha='center', va='center', color='white',
            bbox=dict(facecolor='black', edgecolor='none', boxstyle='round,pad=0.3')
        )

    plt.xlabel('Y position (m)')
    plt.ylabel('Z position (m)')
    plt.title('ArUco Marker Layout (Y-Z side view)')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(image_path)

    node.get_logger().info(f"✅ Saved layout to {image_path}")
    node.get_logger().info(f"✅ Saved marker poses to {yaml_path}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
