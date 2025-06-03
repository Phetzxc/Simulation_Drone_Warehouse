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
        if dt <= 1e-6:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / dt
        self.integral += error * dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Optional: anti-windup (limit integral term)
        # max_integral = self.limit / max(self.ki, 1e-6)
        # self.integral = np.clip(self.integral, -max_integral, max_integral)

        self.prev_error = error
        return np.clip(output, -self.limit, self.limit)

class CascadeController(Node):
    def __init__(self, trajectory):
        super().__init__('cascade_controller')

        # Declare parameters with default values
        self.declare_parameter('cmd_vel_topic', '/simple_drone/cmd_vel')
        self.declare_parameter('pose_topic', '/simple_drone/gt_pose')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('fixed_yaw', -3.14)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.fixed_yaw = self.get_parameter('fixed_yaw').get_parameter_value().double_value

        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.subscription = self.create_subscription(Pose, pose_topic, self.pose_callback, 10)
        self.current_pose = None

        self.trajectory = trajectory
        self.traj_keys = sorted(trajectory.keys())
        self.dt = 1.0 / self.rate_hz

        # PID controllers (tune as needed)
        self.pid_x = PID(kp=0.1, ki=0.0, kd=0.01, limit=1.5)
        self.pid_y = PID(kp=0.1, ki=0.0, kd=0.01, limit=1.5)
        self.pid_z = PID(kp=0.05, ki=0.0, kd=0.005, limit=1.5)
        self.pid_yaw = PID(kp=0.015, ki=0.0, kd=0.1, limit=1.0)

        self.start_time = self.get_clock().now()

        self.counter = 0  # For controlled logging

        self.timer = self.create_timer(self.dt, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg

    def interpolate_point(self, current_time):
        traj = self.trajectory
        keys = self.traj_keys

        if current_time <= traj[keys[0]]['t']:
            return traj[keys[0]]
        if current_time >= traj[keys[-1]]['t']:
            return traj[keys[-1]]

        for i in range(len(keys) - 1):
            t0 = traj[keys[i]]['t']
            t1 = traj[keys[i + 1]]['t']
            if t0 <= current_time < t1:
                ratio = (current_time - t0) / (t1 - t0)
                p0 = traj[keys[i]]['position']
                p1 = traj[keys[i + 1]]['position']
                v0 = traj[keys[i]]['velocity']
                v1 = traj[keys[i + 1]]['velocity']

                interp_pos = {axis: p0[axis] + ratio * (p1[axis] - p0[axis]) for axis in ['x', 'y', 'z']}
                interp_vel = {axis: v0[axis] + ratio * (v1[axis] - v0[axis]) for axis in ['x', 'y', 'z']}
                return {'position': interp_pos, 'velocity': interp_vel}
        return traj[keys[-1]]  # fallback

    def control_loop(self):
        if self.current_pose is None:
            self.get_logger().warn("Waiting for /simple_drone/gt_pose...")
            return

        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds * 1e-9  # seconds float

        last_time = self.trajectory[self.traj_keys[-1]]['t']

        if elapsed_time >= last_time:
            # ถึงปลายทางแล้ว ให้หยุด
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)

            self.get_logger().info(f"Reached end of trajectory at {elapsed_time:.2f}s. Stopping.")
            return

        point = self.interpolate_point(elapsed_time)
        desired_pos = np.array([point['position']['x'], point['position']['y'], point['position']['z']])
        feedforward_vel = np.array([point['velocity']['x'], point['velocity']['y'], point['velocity']['z']])

        current_pos = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        ])

        error = desired_pos - current_pos

        # PID + feedforward velocity
        max_vel = 1.0
        vel_x = np.clip(self.pid_x.compute(error[0], self.dt) + feedforward_vel[0], -max_vel, max_vel)
        vel_y = np.clip(self.pid_y.compute(error[1], self.dt) + feedforward_vel[1], -max_vel, max_vel)
        vel_z = np.clip(self.pid_z.compute(error[2], self.dt) + feedforward_vel[2], -max_vel, max_vel)

        # Yaw control
        q = self.current_pose.orientation
        current_quat = [q.x, q.y, q.z, q.w]
        _, _, current_yaw = quat2euler(current_quat, axes='sxyz')
        yaw_error = self._normalize_angle(self.fixed_yaw - current_yaw)
        yaw_rate = self.pid_yaw.compute(yaw_error, self.dt)

        # Publish Twist cmd_vel
        msg = Twist()
        msg.linear.x = -float(vel_x)
        msg.linear.y = -float(vel_y)
        msg.linear.z = float(vel_z)
        msg.angular.z = float(yaw_rate)
        self.publisher.publish(msg)

        # Controlled logging every 10 cycles
        self.counter += 1
        if self.counter % 10 == 0:
            self.get_logger().info(
                f"time: {elapsed_time:.2f}s pos_err: {error.round(2)} yaw_err: {yaw_error:.2f} "
                f"vel: [{vel_x:.2f}, {vel_y:.2f}, {vel_z:.2f}] yaw_rate: {yaw_rate:.2f}"
            )

    @staticmethod
    def _normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

def load_trajectory():
    try:
        pkg_path = get_package_share_directory("sjtu_drone_control")
        yaml_path = os.path.join(pkg_path, "config", "aruco_gazebo", "gen_trajectory_output.yaml")
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading trajectory file: {e}")
        return None

def main():
    rclpy.init()
    trajectory = load_trajectory()
    if trajectory is None:
        print("Failed to load trajectory, exiting.")
        return

    node = CascadeController(trajectory)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
