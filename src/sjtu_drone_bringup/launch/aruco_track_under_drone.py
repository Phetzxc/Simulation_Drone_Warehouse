#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv2
import numpy as np
import tkinter as tk
from PIL import Image as PILImage, ImageTk
from tf_transformations import quaternion_from_euler
import math

class ArucoTkinterNode(Node):
    def __init__(self):
        super().__init__('aruco_track')

        # Parameters
        self.marker_size = 0.30

        width, height = 640, 360
        hfov_rad = 2.09
        fx = fy = (width / 2) / math.tan(hfov_rad / 2)
        cx, cy = width / 2, height / 2

        self.camera_matrix = np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        self.T_cam_to_base = np.array([
            [-1, 0, 0, -0.15],
            [ 0,-1, 0,  0.0],
            [ 0, 0, 1,  0.10],
            [ 0, 0, 0,  1.0]
        ], dtype=np.float32)


        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/simple_drone/bottom/image_raw', self.image_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/base_to_aruco', 10)
        self.cmd_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.land_pub = self.create_publisher(Empty, '/simple_drone/land', 10)
        self.gt_pose_sub = self.create_subscription(Pose, '/simple_drone/gt_pose', self.gt_pose_callback, 10)
        self.drone_position_world = np.array([0.0, 0.0, 0.0])  # default

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, parameters)

        # GUI
        self.window = tk.Tk()
        self.window.title("Real-time ArUco Detection")
        self.label = tk.Label(self.window)
        self.label.pack()

        self.get_logger().info(f"Aruco Detector Started | marker_size = {self.marker_size}")

    def gt_pose_callback(self, msg):
        self.drone_position_world = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z
        ])

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
                y_bf1 = y_bf
                x_bf1 = x_bf
                z_bf1 = z_bf
                y_bf = -x_bf1
                x_bf = y_bf1
                R_base_marker = T_base_to_marker[:3, :3]
                yaw = np.arctan2(R_base_marker[1, 0], R_base_marker[0, 0])
                pitch = np.arctan2(-R_base_marker[2, 0], np.sqrt(R_base_marker[2, 1]**2 + R_base_marker[2, 2]**2))
                roll = np.arctan2(R_base_marker[2, 1], R_base_marker[2, 2])

                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size / 2)

                # Display text
                text = [
                    f"ID: {marker_id[0]}",
                    f"Forward/Back (X): {-((1.0*x_bf)-self.drone_position_world[0]):.2f} m",
                    f"Left/Right  (Y): {-((1.0*y_bf)-self.drone_position_world[1]):.2f} m",
                    f"Height (Z):       {-((1.0*z_bf)-self.drone_position_world[2]):.2f} m",
                    f"Yaw: {pitch:.3f} rad",
                    f"Pitch: {yaw:.3f} rad",
                    f"Roll: {roll:.3f} rad"
                ]

                for j, line in enumerate(text):
                    cv2.putText(frame, line, (20, 30 + j * 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # Publish PoseStamped
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = str(marker_id[0])

                pose_msg.pose.position.x = (x_bf - self.drone_position_world[0])
                pose_msg.pose.position.y = -(y_bf - self.drone_position_world[1])
                pose_msg.pose.position.z = -(z_bf - self.drone_position_world[2])

                qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
                pose_msg.pose.orientation.x = qx
                pose_msg.pose.orientation.y = qy
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw

                self.pose_pub.publish(pose_msg)

                # --- PID Control ---
                error_marker_x = pose_msg.pose.position.x
                error_marker_y = pose_msg.pose.position.y
                error_marker_z = pose_msg.pose.position.z

                error_cmd_x = error_marker_x
                error_cmd_y = error_marker_y
                error_cmd_z = error_marker_z

                Kp_x = 0.5
                Kp_y = 0.5
                Kp_z = 0.05

                twist_msg = Twist()
                twist_msg.linear.x = Kp_x * error_cmd_x
                twist_msg.linear.y = Kp_y * error_cmd_y

                if abs(error_cmd_z) > 1.0:
                    twist_msg.linear.z = -Kp_z * abs(error_cmd_z)
                else:
                    twist_msg.linear.z = -0.1
                    if abs(error_cmd_z) < 0.6:
                        self.get_logger().info("Landing...")
                        self.land_pub.publish(Empty())
                        return

                # --- Yaw Control ---
                target_yaw = 0.0
                yaw_error = target_yaw - yaw
                yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi

                Kp_yaw = 0.5
                twist_msg.angular.z = Kp_yaw * yaw_error

                self.cmd_pub.publish(twist_msg)

        # Update GUI
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        imgtk = ImageTk.PhotoImage(image=PILImage.fromarray(rgb))
        self.label.imgtk = imgtk
        self.label.configure(image=imgtk)
        self.window.update_idletasks()
        self.window.update()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTkinterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down gracefully...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
