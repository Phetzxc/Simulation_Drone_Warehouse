import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import yaml
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from transforms3d.euler import quat2euler

class PID:
    def __init__(self, kp, ki, kd, limit=1.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return np.clip(output, -self.limit, self.limit)

class CascadeController(Node):
    def __init__(self, trajectory, rate_hz=20.0):
        super().__init__('cascade_controller')
        self.pub_cmd_vel = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.sub_pose = self.create_subscription(Pose, '/simple_drone/gt_pose', self.pose_callback, 10)

        self.current_pose = None
        self.trajectory = trajectory
        self.traj_keys = sorted(trajectory.keys())
        self.traj_index = 0

        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz

        # PID for position -> output velocity reference
        self.pid_pos_x = PID(0.6, 0.0, 0.1, limit=1.0)
        self.pid_pos_y = PID(0.6, 0.0, 0.1, limit=1.0)
        self.pid_pos_z = PID(0.4, 0.0, 0.08, limit=0.8)

        # PID for velocity -> output velocity command
        self.pid_vel_x = PID(0.8, 0.0, 0.15, limit=1.5)
        self.pid_vel_y = PID(0.8, 0.0, 0.15, limit=1.5)
        self.pid_vel_z = PID(0.5, 0.0, 0.1, limit=1.0)

        # Fixed desired yaw
        self.desired_yaw = -3.14
        self.pid_yaw = PID(0.015, 0.0, 0.005, limit=1.0)

        self.timer = self.create_timer(self.dt, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            self.get_logger().warn("Waiting for pose...")
            return

        if self.traj_index >= len(self.traj_keys):
            self.get_logger().info("Trajectory completed, stopping.")
            self.pub_cmd_vel.publish(Twist())  # stop
            return

        traj_key = self.traj_keys[self.traj_index]
        point = self.trajectory[traj_key]

        # แปลงแกน x,y ให้ติดลบก่อนใช้
        desired_pos = np.array([
            -point['position']['x'],  # * -1
            -point['position']['y'],  # * -1
            point['position']['z']
        ])
        feedforward_vel = np.array([
            -point['velocity']['x'],
            -point['velocity']['y'],
            point['velocity']['z']
        ])

        current_pos = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        ])

        pos_error = desired_pos - current_pos
        dist = np.linalg.norm(pos_error)

        # ถ้าเข้าใกล้จุด waypoint ให้ไปจุดถัดไป
        if dist < 0.1:
            self.get_logger().info(f"Reached waypoint {traj_key}, moving on.")
            self.traj_index += 1
            return

        # --- Cascade Control ---
        # 1) Position PID to get desired velocity reference
        vel_ref_x = self.pid_pos_x.compute(pos_error[0], self.dt)
        vel_ref_y = self.pid_pos_y.compute(pos_error[1], self.dt)
        vel_ref_z = self.pid_pos_z.compute(pos_error[2], self.dt)

        vel_ref = np.array([vel_ref_x, vel_ref_y, vel_ref_z]) + feedforward_vel

        # 2) Velocity PID to compute velocity command (assuming measured velocity = 0 here)
        measured_vel = np.array([0.0, 0.0, 0.0]) 

        vel_error = vel_ref - measured_vel

        cmd_vel_x = self.pid_vel_x.compute(vel_error[0], self.dt)
        cmd_vel_y = self.pid_vel_y.compute(vel_error[1], self.dt)
        cmd_vel_z = self.pid_vel_z.compute(vel_error[2], self.dt)

        # Yaw control
        q = self.current_pose.orientation
        current_quat = [q.x, q.y, q.z, q.w]
        _, _, current_yaw = quat2euler(current_quat, axes='sxyz')
        yaw_error = self._normalize_angle(self.desired_yaw - current_yaw)
        yaw_rate = self.pid_yaw.compute(yaw_error, self.dt)

        # Publish velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = float(cmd_vel_x)
        cmd_msg.linear.y = float(cmd_vel_y)
        cmd_msg.linear.z = float(cmd_vel_z)
        cmd_msg.angular.z = float(yaw_rate)
        self.pub_cmd_vel.publish(cmd_msg)

        self.get_logger().info(
            f"[{traj_key}] pos_err: {pos_error.round(3)}, dist: {dist:.3f}, "
            f"vel_ref: {vel_ref.round(3)}, cmd_vel: {[cmd_vel_x, cmd_vel_y, cmd_vel_z]}, yaw_err: {yaw_error:.2f}"
        )


    @staticmethod
    def _normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

def load_trajectory():
    pkg_path = get_package_share_directory("sjtu_drone_control")
    yaml_path = os.path.join(pkg_path, "config", "aruco_gazebo", "gen_trajectory_output.yaml")
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)

def main():
    rclpy.init()
    trajectory = load_trajectory()
    node = CascadeController(trajectory)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
