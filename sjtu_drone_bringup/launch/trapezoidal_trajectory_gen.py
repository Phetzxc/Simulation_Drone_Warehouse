import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory

# Trapezoidal velocity profile for one segment (3D)
def trapezoidal_segment(p_start, p_end, v_max, a_max, dt, v_start=0.0, v_end=None):
    p_start = np.array(p_start)
    p_end = np.array(p_end)
    delta = p_end - p_start
    distance = np.linalg.norm(delta)
    direction = delta / distance if distance > 1e-6 else np.zeros(3)

    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc ** 2

    if distance < 2 * d_acc:
        # Triangular profile
        t_acc = np.sqrt(distance / a_max)
        t_total = 2 * t_acc
        t_cruise = 0.0
        d_cruise = 0.0
        v_peak = a_max * t_acc
    else:
        # Trapezoidal profile
        d_cruise = distance - 2 * d_acc
        t_cruise = d_cruise / v_max
        t_total = 2 * t_acc + t_cruise
        v_peak = v_max

    pos_list, vel_list, acc_list = [], [], []
    t = 0.0
    while t <= t_total + 1e-6:
        if t < t_acc:
            a = a_max
            v = a * t
            d = 0.5 * a * t ** 2
        elif t < t_acc + t_cruise:
            a = 0.0
            v = v_peak
            d = d_acc + v_peak * (t - t_acc)
        else:
            t_dec = t - t_acc - t_cruise
            a = -a_max
            v = v_peak + a * t_dec
            d = d_acc + d_cruise + v_peak * t_dec + 0.5 * a * t_dec ** 2

        pos = p_start + direction * d
        vel = direction * v
        acc = direction * a

        pos_list.append(pos)
        vel_list.append(vel)
        acc_list.append(acc)
        t += dt

    # Force final velocity and acceleration to 0 if needed
    if v_end == 0.0:
        vel_list[-1] = np.zeros(3)
        acc_list[-1] = np.zeros(3)

    return pos_list, vel_list, acc_list

# --- Input path ---
config_path = os.path.join(
    get_package_share_directory("sjtu_drone_control"),
    "config", "aruco_gazebo"
)
via_path = os.path.join(config_path, "gen_path_modified.yaml")
plot_path = os.path.join(config_path, "gen_trajectory_plot.jpg")

# --- Load via-points ---
with open(via_path, 'r') as f:
    via_data = yaml.safe_load(f)

via_points = [np.array([pt['x'], pt['y'], pt['z']]) for pt in via_data.values()]

# --- Generate full trajectory ---
all_pos, all_vel, all_acc, all_time = [], [], [], []
dt = 0.05
t_now = 0.0
v_max = 0.25
a_max = 1.5

for i in range(len(via_points) - 1):
    is_last = (i == len(via_points) - 2)
    v_end = 0.0 if is_last else None

    pos_seg, vel_seg, acc_seg = trapezoidal_segment(
        via_points[i], via_points[i + 1], v_max, a_max, dt,
        v_start=0.0, v_end=v_end
    )
    seg_len = len(pos_seg)
    time_seg = [t_now + j * dt for j in range(seg_len)]

    all_pos.extend(pos_seg)
    all_vel.extend(vel_seg)
    all_acc.extend(acc_seg)
    all_time.extend(time_seg)

    t_now = all_time[-1] + dt

# --- Convert to numpy ---
all_pos = np.array(all_pos)
all_vel = np.array(all_vel)
all_acc = np.array(all_acc)
all_time = np.array(all_time)

# --- Plot ---
fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# Position
axs[0].plot(all_time, all_pos[:, 0], label='X')
axs[0].plot(all_time, all_pos[:, 1], label='Y')
axs[0].plot(all_time, all_pos[:, 2], label='Z')
axs[0].set_ylabel("Position (m)")
axs[0].legend()
axs[0].grid(True)

# Velocity
axs[1].plot(all_time, all_vel[:, 0], label='X')
axs[1].plot(all_time, all_vel[:, 1], label='Y')
axs[1].plot(all_time, all_vel[:, 2], label='Z')
axs[1].set_ylabel("Linear Velocity (m/s)")
axs[1].legend()
axs[1].grid(True)

# Acceleration
axs[2].plot(all_time, all_acc[:, 0], label='X')
axs[2].plot(all_time, all_acc[:, 1], label='Y')
axs[2].plot(all_time, all_acc[:, 2], label='Z')
axs[2].set_ylabel("Linear Acceleration (m/s²)")
axs[2].set_xlabel("Time (s)")
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.savefig(plot_path)
plt.close()

# --- Log ---
print("\n✅ Trapezoidal trajectory generated from via-points.")
print(f"🖼️  Trajectory plot saved to:\n    {plot_path}")
