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
        self.image_sub = self.create_subscription(Image, '/simple_drone/front/image_raw', self.image_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/base_to_aruco', 10)
        self.gt_pose_sub = self.create_subscription(Pose, '/simple_drone/gt_pose', self.gt_pose_callback, 10)

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

        # GUI
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
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

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

                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size / 2)

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
        # Save raw data
        with open(self.yaml_path, 'w') as f:
            yaml.safe_dump(self.marker_data, f, sort_keys=False)
        self.get_logger().info(f"✅ Saved raw observations to:\n    {self.yaml_path}")

        # Process average data
        avg_data = {}
        x_list, y_list, z_list, id_list = [], [], [], []

        for marker_id, poses in self.marker_data.items():
            pos = np.array([[p['position']['x'], p['position']['y'], p['position']['z']] for p in poses])
            ori = np.array([[p['orientation']['x'], p['orientation']['y'],
                             p['orientation']['z'], p['orientation']['w']] for p in poses])

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
        output_path = os.path.join(os.path.dirname(self.yaml_path), "estimate_marker_from_drone.yaml")
        with open(output_path, 'w') as f:
            yaml.dump(avg_data, f, sort_keys=False)
        self.get_logger().info(f"📄 Saved averaged result to:\n    {output_path}")

        # Save plot
        plt.figure(figsize=(14, 6))
        plt.scatter(y_list, z_list, c='green', label='Marker')
        for i, marker_id in enumerate(id_list):
            plt.text(y_list[i], z_list[i] + 0.2, marker_id, fontsize=10, ha='center', va='bottom',
                     bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray'))
        plt.title('Marker Position (Y vs Z)')
        plt.xlabel('Y (m)')
        plt.ylabel('Z (m)')
        plt.grid(True)
        plt.axis('equal')
        plt.tight_layout()

        image_path = os.path.join(os.path.dirname(self.yaml_path), "estimate_marker_from_drone.jpg")
        plt.savefig(image_path)
        plt.close()
        self.get_logger().info(f"🖼️  Saved marker plot to:\n    {image_path}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTkinterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Node killed by user (Ctrl+C)")
    finally:
        node.save_yaml_on_exit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
