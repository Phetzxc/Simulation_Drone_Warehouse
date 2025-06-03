#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tkinter as tk
from PIL import Image as PILImage, ImageTk
from tf_transformations import quaternion_from_euler
import math
import os
import yaml
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
from time import time

# -------- ArUco Detection Node --------
class ArucoTkinterNode(Node):
    def __init__(self):
        super().__init__('aruco_track')

        # Parameters
        self.declare_parameter('marker_size', 0.08)
        self.marker_size = self.get_parameter('marker_size').value

        width, height = 640, 360
        hfov_rad = 2.09
        fx = fy = (width / 2) / math.tan(hfov_rad / 2)
        cx, cy = width / 2, height / 2

        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        self.T_cam_to_base = np.array([
            [-1, 0, 0, -0.09],
            [0, -1, 0, 0.00],
            [0, 0, 1, 0.10],
            [0, 0, 0, 1.0]
        ], dtype=np.float32)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/simple_drone/front/image_raw',
            self.image_callback,
            10
        )
        self.pose_pub = self.create_publisher(PoseStamped, '/base_to_aruco', 10)
        self.gt_pose_sub = self.create_subscription(
            Pose,
            '/simple_drone/gt_pose',
            self.gt_pose_callback,
            10
        )

        self.drone_position_world = np.array([0.0, 0.0, 0.0])

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        parameters = cv2.aruco.DetectorParameters()
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 23
        parameters.adaptiveThreshWinSizeStep = 10
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.adaptiveThreshConstant = 7
        parameters.minMarkerPerimeterRate = 0.03
        parameters.maxMarkerPerimeterRate = 4.0
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, parameters)

        # GUI setup
        self.window = tk.Tk()
        self.window.title("Real-time ArUco Detection")
        self.label = tk.Label(self.window)
        self.label.pack()

        # YAML path setup
        config_path = os.path.join(
            get_package_share_directory("sjtu_drone_control"),
            "config", "aruco_gazebo"
        )
        os.makedirs(config_path, exist_ok=True)
        self.yaml_path = os.path.join(config_path, "saw_marker_from_drone.yaml")

        # Memory to store detected markers data
        self.marker_data = {}

        self.get_logger().info(f"Aruco Detector Started | marker_size = {self.marker_size}")

    def gt_pose_callback(self, msg):
        self.drone_position_world = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z
        ])

    def save_marker_to_memory(self, marker_id, position, orientation):
        key = f"marker_{marker_id}"
        if key not in self.marker_data:
            self.marker_data[key] = []

        self.marker_data[key].append({
            "position": {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
            },
            "orientation": {
                "x": float(orientation[0]),
                "y": float(orientation[1]),
                "z": float(orientation[2]),
                "w": float(orientation[3]),
            }
        })

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )

            for i, marker_id in enumerate(ids):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                R_cam_to_marker, _ = cv2.Rodrigues(rvec)
                T_cam_to_marker = np.eye(4)
                T_cam_to_marker[:3, :3] = R_cam_to_marker
                T_cam_to_marker[:3, 3] = tvec

                T_base_to_marker = self.T_cam_to_base @ T_cam_to_marker
                x_bf, y_bf, z_bf = T_base_to_marker[:3, 3]
                y_bf = -y_bf
                R_base_marker = T_base_to_marker[:3, :3]
                yaw = np.arctan2(R_base_marker[1, 0], R_base_marker[0, 0])
                pitch = np.arctan2(-R_base_marker[2, 0], np.sqrt(R_base_marker[2, 1]**2 + R_base_marker[2, 2]**2))
                roll = np.arctan2(R_base_marker[2, 1], R_base_marker[2, 2])

                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size / 2
                )

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = str(marker_id[0])

                pose_msg.pose.position.x = -(z_bf - self.drone_position_world[0])
                pose_msg.pose.position.y = -(x_bf - self.drone_position_world[1])
                pose_msg.pose.position.z = -(y_bf - self.drone_position_world[2])

                qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
                pose_msg.pose.orientation.x = qx
                pose_msg.pose.orientation.y = qy
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw

                self.pose_pub.publish(pose_msg)

                self.save_marker_to_memory(
                    marker_id[0],
                    (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z),
                    (qx, qy, qz, qw)
                )

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        imgtk = ImageTk.PhotoImage(image=PILImage.fromarray(rgb))
        self.label.imgtk = imgtk
        self.label.configure(image=imgtk)
        self.window.update_idletasks()
        self.window.update()

    def save_yaml_on_exit(self):
        # Save raw observations
        with open(self.yaml_path, 'w') as f:
            yaml.safe_dump(self.marker_data, f, sort_keys=False)
        self.get_logger().info(f"✅ Saved raw observations to:\n    {self.yaml_path}")

        # Process average data
        avg_data = {}
        x_list, y_list, z_list, id_list = [], [], [], []

        for marker_id, poses in self.marker_data.items():
            pos = np.array([
                [p['position']['x'], p['position']['y'], p['position']['z']]
                for p in poses
            ])
            ori = np.array([
                [p['orientation']['x'], p['orientation']['y'],
                 p['orientation']['z'], p['orientation']['w']]
                for p in poses
            ])

            avg_pos = np.mean(pos, axis=0)
            avg_ori = np.mean(ori, axis=0)

            avg_data[marker_id] = {
                'position': {
                    'x': float(avg_pos[0]),
                    'y': float(avg_pos[1]),
                    'z': float(avg_pos[2])
                },
                'orientation': {
                    'x': float(avg_ori[0]),
                    'y': float(avg_ori[1]),
                    'z': float(avg_ori[2]),
                    'w': float(avg_ori[3])
                }
            }

            x_list.append(avg_pos[0])
            y_list.append(avg_pos[1])
            z_list.append(avg_pos[2])
            id_list.append(marker_id)

        # Save averaged YAML
        output_path = os.path.join(
            os.path.dirname(self.yaml_path),
            "estimate_marker_from_drone.yaml"
        )
        with open(output_path, 'w') as f:
            yaml.dump(avg_data, f, sort_keys=False)
        self.get_logger().info(f"📄 Saved averaged result to:\n    {output_path}")

        # Save plot
        plt.figure(figsize=(14, 6))
        plt.scatter(y_list, z_list, c='green', label='Marker')
        for i, marker_id in enumerate(id_list):
            plt.text(
                y_list[i],
                z_list[i] + 0.2,
                marker_id,
                fontsize=10,
                ha='center',
                va='bottom',
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray')
            )
        plt.title('Marker Position (Y vs Z)')
        plt.xlabel('Y (m)')
        plt.ylabel('Z (m)')
        plt.grid(True)
        plt.axis('equal')
        plt.tight_layout()

        image_path = os.path.join(
            os.path.dirname(self.yaml_path),
            "estimate_marker_from_drone.jpg"
        )
        plt.savefig(image_path)
        plt.close()
        self.get_logger().info(f"🖼️  Saved marker plot to:\n    {image_path}")

# -------- Pose Listener Node --------
class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.pose = None
        self.subscription = self.create_subscription(
            Pose,
            '/simple_drone/gt_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.pose = msg

# -------- Helper Functions --------
def interpolate_points(p1, p2, step=0.2):
    vec = p2 - p1
    dist = np.linalg.norm(vec)
    if dist < 1e-6:
        return [p1]
    num_points = max(int(dist / step), 1)
    return [p1 + vec * (i / num_points) for i in range(num_points)]

def compute_yaw(p_from, p_to):
    dx = p_to[0] - p_from[0]
    dy = p_to[1] - p_from[1]
    return np.arctan2(dy, dx)

def yaw_to_quaternion(yaw):
    return (0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0))

# Quintic segment generator
def quintic_segment(p_start, p_end, v_start, v_end, T, dt):
    p0 = p_start
    pf = p_end
    v0 = v_start
    vf = v_end
    a0 = np.zeros(3)
    af = np.zeros(3)

    T2 = T**2
    T3 = T**3
    T4 = T**4
    T5 = T**5

    A = np.array([
        [   T3,    T4,     T5],
        [ 3*T2,  4*T3,   5*T4],
        [ 6*T,  12*T2,  20*T3]
    ])

    coeffs = np.zeros((3,6))
    for i in range(3):
        b = np.array([
            pf[i] - (p0[i] + v0[i]*T + 0.5*a0[i]*T2),
            vf[i] - (v0[i] + a0[i]*T),
            af[i] - a0[i]
        ])
        x = np.linalg.solve(A, b)
        coeffs[i,0] = p0[i]
        coeffs[i,1] = v0[i]
        coeffs[i,2] = 0.0
        coeffs[i,3] = x[0]
        coeffs[i,4] = x[1]
        coeffs[i,5] = x[2]

    pos_list, vel_list, acc_list = [], [], []
    t = 0.0
    while t <= T + 1e-6:
        pos = np.zeros(3)
        vel = np.zeros(3)
        acc = np.zeros(3)
        for i in range(3):
            a0_, a1_, a2_, a3_, a4_, a5_ = coeffs[i]
            pos[i] = a0_ + a1_*t + a2_*t**2 + a3_*t**3 + a4_*t**4 + a5_*t**5
            vel[i] = a1_ + 2*a2_*t + 3*a3_*t**2 + 4*a4_*t**3 + 5*a5_*t**4
            acc[i] = 2*a2_ + 6*a3_*t + 12*a4_*t**2 + 20*a5_*t**3
        pos_list.append(pos)
        vel_list.append(vel)
        acc_list.append(acc)
        t += dt

    return pos_list, vel_list, acc_list

# -------- Path & Trajectory Generation --------
def generate_snake_path_and_trajectory():
    # Initialize ROS2 to get current drone pose
    rclpy.init()
    node = PoseListener()

    start_time = time()
    print("⏳ Waiting for /simple_drone/gt_pose...")
    while rclpy.ok() and node.pose is None and (time() - start_time < 10):
        rclpy.spin_once(node)

    if node.pose is None:
        raise RuntimeError("❌ Could not get drone position from /simple_drone/gt_pose")

    pose = node.pose
    drone_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    print(f"✅ Got drone position: {drone_pos}")
    node.destroy_node()
    rclpy.shutdown()

    # Paths
    pkg_path = get_package_share_directory("sjtu_drone_control")
    config_path = os.path.join(pkg_path, "config", "aruco_gazebo")
    os.makedirs(config_path, exist_ok=True)

    saw_yaml = os.path.join(config_path, "saw_marker_from_drone.yaml")
    gen_path_yaml = os.path.join(config_path, "gen_path_modified.yaml")
    compare_plot = os.path.join(config_path, "compare_path.jpg")

    # Load raw marker observations
    with open(saw_yaml, 'r') as f:
        raw_data = yaml.safe_load(f)

    # Average marker positions
    marker_positions = []
    for marker_id, poses in raw_data.items():
        pos = np.array([
            [p['position']['x'], p['position']['y'], p['position']['z']]
            for p in poses
        ])
        avg_pos = np.mean(pos, axis=0)
        marker_positions.append({'id': marker_id, 'pos': avg_pos})

    # Group into Z rows
    z_threshold = 0.5
    rows = []
    for marker in sorted(marker_positions, key=lambda m: m['pos'][2]):
        placed = False
        for row in rows:
            if abs(row[0]['pos'][2] - marker['pos'][2]) <= z_threshold:
                row.append(marker)
                placed = True
                break
        if not placed:
            rows.append([marker])

    # Snake path ordering by Y
    path_points = []
    direction = 1
    for row in rows:
        sorted_row = sorted(
            row,
            key=lambda m: m['pos'][1],
            reverse=(direction == -1)
        )
        path_points.extend([m['pos'] for m in sorted_row])
        direction *= -1

    # Build full path including drone start/end
    full_path = []
    interp_from_drone = interpolate_points(drone_pos, path_points[0], step=0.2)
    full_path.extend(interp_from_drone)
    full_path.extend(path_points)
    interp_back_to_drone = interpolate_points(path_points[-1], drone_pos, step=0.2)
    full_path.extend(interp_back_to_drone)

    # Offset markers but not drone position
    via_points_mod = []
    for pt in full_path:
        if np.allclose(pt, drone_pos, atol=1e-3):
            via_points_mod.append(pt)
        else:
            via_points_mod.append(np.array([pt[0] + 1.3, pt[1], pt[2] + 0.83]))

    # Save via points with orientation to gen_path_modified.yaml
    yaml_path_mod = {}
    for i, pt in enumerate(via_points_mod):
        if i < len(via_points_mod) - 1:
            yaw = compute_yaw(pt, via_points_mod[i + 1])
        else:
            yaw = compute_yaw(via_points_mod[i - 1], pt)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        yaml_path_mod[f"via_point_{i}"] = {
            'x': float(pt[0]),
            'y': float(pt[1]),
            'z': float(pt[2]),
            'orientation': {
                'x': float(qx),
                'y': float(qy),
                'z': float(qz),
                'w': float(qw)
            }
        }

    with open(gen_path_yaml, 'w') as f:
        yaml.dump(yaml_path_mod, f, sort_keys=False)

    # Plot comparison of original vs offset path (Y-Z)
    y1 = [pt[1] for pt in full_path]
    z1 = [pt[2] for pt in full_path]
    y2 = [pt[1] for pt in via_points_mod]
    z2 = [pt[2] for pt in via_points_mod]

    plt.figure(figsize=(10, 7))
    plt.plot(y1, z1, '-o', label="Original Path (with drone)", markersize=3)
    plt.plot(y2, z2, '-o', label="Modified Path (Offset)", markersize=3)

    for i in range(len(y1) - 1):
        plt.arrow(
            y1[i], z1[i], y1[i + 1] - y1[i], z1[i + 1] - z1[i],
            head_width=0.05, head_length=0.05, length_includes_head=True
        )
    for i in range(len(y2) - 1):
        plt.arrow(
            y2[i], z2[i], y2[i + 1] - y2[i], z2[i + 1] - z2[i],
            head_width=0.05, head_length=0.05, length_includes_head=True
        )

    for marker in marker_positions:
        y, z = marker['pos'][1], marker['pos'][2]
        plt.plot(y, z, 'ro')
        plt.text(y + 0.02, z + 0.02, f"id: {marker['id']}", fontsize=8, color='red')

    plt.title("Comparison of Original and Offset Paths (Y-Z plane)")
    plt.xlabel("Y (m)")
    plt.ylabel("Z (m)")
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.tight_layout()
    plt.savefig(compare_plot)
    plt.close()

    print("\n✅ Snake path (starting and ending at drone position) with offset & orientation saved.")
    print(f"📄 Modified YAML:   {gen_path_yaml}")
    print(f"🖼️  Path plot:      {compare_plot}")

    # -------- Quintic Trajectory Generation --------
    # Load via-points from gen_path_modified.yaml
    with open(gen_path_yaml, 'r') as f:
        via_data = yaml.safe_load(f)

    via_points = [np.array([pt['x'], pt['y'], pt['z']]) for pt in via_data.values()]

    dt = 0.05
    v_max = 0.4  # m/s

    all_pos, all_vel, all_acc, all_time = [], [], [], []
    t_now = 0.0
    v_prev_end = np.zeros(3)

    for i in range(len(via_points) - 1):
        p_start = via_points[i]
        p_end = via_points[i + 1]

        dist = np.linalg.norm(p_end - p_start)
        T = max(dist / v_max, 0.1)

        if i == len(via_points) - 2:
            v_end = np.zeros(3)
        else:
            next_dir = via_points[i + 2] - p_end
            norm_next = np.linalg.norm(next_dir)
            if norm_next > 1e-6:
                next_dir /= norm_next
            else:
                next_dir = np.zeros(3)
            v_end_mag = np.linalg.norm(v_prev_end)
            if v_end_mag < 1e-3:
                v_end_mag = v_max * 0.8
            v_end = next_dir * v_end_mag

        pos_seg, vel_seg, acc_seg = quintic_segment(
            p_start, p_end, v_prev_end, v_end, T, dt
        )
        v_prev_end = vel_seg[-1]

        seg_len = len(pos_seg)
        time_seg = [t_now + j * dt for j in range(seg_len)]

        all_pos.extend(pos_seg)
        all_vel.extend(vel_seg)
        all_acc.extend(acc_seg)
        all_time.extend(time_seg)

        t_now = all_time[-1] + dt

    all_pos = np.array(all_pos)
    all_vel = np.array(all_vel)
    all_acc = np.array(all_acc)
    all_time = np.array(all_time)

    # Plot trajectory (positions, velocities, accelerations)
    traj_plot = os.path.join(config_path, "gen_trajectory_plot.jpg")
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    axs[0].plot(all_time, all_pos[:, 0], label='X')
    axs[0].plot(all_time, all_pos[:, 1], label='Y')
    axs[0].plot(all_time, all_pos[:, 2], label='Z')
    axs[0].set_ylabel("Position (m)")
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(all_time, all_vel[:, 0], label='X')
    axs[1].plot(all_time, all_vel[:, 1], label='Y')
    axs[1].plot(all_time, all_vel[:, 2], label='Z')
    axs[1].set_ylabel("Linear Velocity (m/s)")
    axs[1].legend()
    axs[1].grid(True)

    axs[2].plot(all_time, all_acc[:, 0], label='X')
    axs[2].plot(all_time, all_acc[:, 1], label='Y')
    axs[2].plot(all_time, all_acc[:, 2], label='Z')
    axs[2].set_ylabel("Linear Acceleration (m/s²)")
    axs[2].set_xlabel("Time (s)")
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout()
    plt.savefig(traj_plot)
    plt.close()

    # Save trajectory data to YAML
    output_yaml_path = os.path.join(config_path, "gen_trajectory_output.yaml")
    output_data = {}
    for i, t in enumerate(all_time):
        output_data[f"t{i:04d}"] = {
            "t": float(np.round(t, 4)),
            "position": {
                "x": float(np.round(all_pos[i, 0], 6)),
                "y": float(np.round(all_pos[i, 1], 6)),
                "z": float(np.round(all_pos[i, 2], 6)),
            },
            "velocity": {
                "x": float(np.round(all_vel[i, 0], 6)),
                "y": float(np.round(all_vel[i, 1], 6)),
                "z": float(np.round(all_vel[i, 2], 6)),
            },
            "acceleration": {
                "x": float(np.round(all_acc[i, 0], 6)),
                "y": float(np.round(all_acc[i, 1], 6)),
                "z": float(np.round(all_acc[i, 2], 6)),
            },
        }

    with open(output_yaml_path, 'w') as f:
        yaml.dump(output_data, f, sort_keys=False)

    print("\n✅ Quintic polynomial trajectory generated from via-points with velocity continuity.")
    print(f"🖼️  Trajectory plot saved to:\n    {traj_plot}")
    print(f"📄  Trajectory data saved to:\n    {output_yaml_path}")

# -------- Main --------
def main(args=None):
    # Run ArUco detection node
    rclpy.init(args=args)
    aruco_node = ArucoTkinterNode()
    try:
        rclpy.spin(aruco_node)
    except KeyboardInterrupt:
        print("\n[INFO] Aruco node killed by user (Ctrl+C)")
    finally:
        aruco_node.save_yaml_on_exit()
        aruco_node.destroy_node()
        rclpy.shutdown()

        # After ArUco node finishes, generate path and trajectory
        generate_snake_path_and_trajectory()

if __name__ == '__main__':
    main()
