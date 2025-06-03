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
        derivative = (error - self.prev_error) / dt if dt > 0. else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return np.clip(output, -self.limit, self.limit)

class CascadeController(Node):
    def __init__(self, trajectory, rate_hz=20.0):
        super().__init__('cascade_controller')
        self.publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/simple_drone/gt_pose', self.pose_callback, 10)
        self.current_pose = None

        self.trajectory = trajectory
        self.traj_keys = sorted(trajectory.keys())
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz

        # PID controllers - ปรับค่าให้ตอบสนองดีขึ้น
        self.pid_x = PID(kp=0.1, ki=0.0, kd=0.01, limit=1.5)
        self.pid_y = PID(kp=0.1, ki=0.0, kd=0.01, limit=1.5)
        self.pid_z = PID(kp=0.05, ki=0.0, kd=0.005, limit=1.5)
        self.pid_yaw = PID(kp=0.015, ki=0.0, kd=0.1, limit=1.0)

        self.start_time = self.get_clock().now()

        self.desired_yaw = -3.14  # fix orientation yaw

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
        yaw_error = self._normalize_angle(self.desired_yaw - current_yaw)
        yaw_rate = self.pid_yaw.compute(yaw_error, self.dt)

        # Publish Twist cmd_vel
        msg = Twist()
        msg.linear.x = -float(vel_x)
        msg.linear.y = -float(vel_y)
        msg.linear.z = float(vel_z)
        msg.angular.z = float(yaw_rate)
        self.publisher.publish(msg)

        self.get_logger().info(
            f"time: {elapsed_time:.2f}s pos_err: {error.round(2)} yaw_err: {yaw_error:.2f} vel: {[vel_x, vel_y, vel_z]}"
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
