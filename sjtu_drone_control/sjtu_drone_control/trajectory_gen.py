import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory

def quintic_segment(p_start, p_end, v_start, v_end, T, dt):
    """
    Generate quintic polynomial trajectory segment from p_start to p_end with
    start velocity v_start and end velocity v_end in duration T.
    Returns lists of positions, velocities, accelerations.
    
    p_start, p_end, v_start, v_end: np.array shape (3,)
    T: float, total duration of segment
    dt: float, timestep
    """

    p0 = p_start
    pf = p_end
    v0 = v_start
    vf = v_end
    a0 = np.zeros(3)
    af = np.zeros(3)

    # Setup the time vector powers
    T2 = T**2
    T3 = T**3
    T4 = T**4
    T5 = T**5

    # Solve for polynomial coefficients for each dimension
    # position = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    # Using boundary conditions on pos, vel, acc at t=0 and t=T

    A = np.array([
        [   T3,    T4,     T5],
        [ 3*T2,  4*T3,   5*T4],
        [ 6*T,  12*T2,  20*T3]
    ])

    pos_list, vel_list, acc_list = [], [], []

    # Compute coefficients a3, a4, a5 for each axis
    coeffs = np.zeros((3,6))  # 3 axes, 6 coeffs each

    for i in range(3):
        b = np.array([
            pf[i] - (p0[i] + v0[i]*T + 0.5*a0[i]*T2),
            vf[i] - (v0[i] + a0[i]*T),
            af[i] - a0[i]
        ])
        x = np.linalg.solve(A, b)
        # a0, a1, a2 are known:
        coeffs[i,0] = p0[i]
        coeffs[i,1] = v0[i]
        coeffs[i,2] = 0.0
        coeffs[i,3] = x[0]
        coeffs[i,4] = x[1]
        coeffs[i,5] = x[2]

    # Generate trajectory
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

# --- Parameters ---
dt = 0.05
v_max = 0.4  # max velocity magnitude (m/s)

# --- Generate full trajectory with velocity continuity ---
all_pos, all_vel, all_acc, all_time = [], [], [], []

t_now = 0.0

# Initial velocity start with zero vector
v_prev_end = np.zeros(3)

for i in range(len(via_points) - 1):
    p_start = via_points[i]
    p_end = via_points[i + 1]

    dist = np.linalg.norm(p_end - p_start)
    # Estimate segment duration proportional to distance and max velocity
    T = max(dist / v_max, 0.1)

    # Determine v_end for this segment:
    # If last segment, velocity ends at zero (stop)
    # Else, estimate v_end for smoothness (can keep same direction and magnitude as v_prev_end)
    if i == len(via_points) - 2:
        v_end = np.zeros(3)
    else:
        # Simple heuristic: keep velocity direction towards next segment
        next_dir = via_points[i + 2] - p_end
        next_dir_norm = np.linalg.norm(next_dir)
        if next_dir_norm > 1e-6:
            next_dir /= next_dir_norm
        else:
            next_dir = np.zeros(3)

        # Velocity magnitude can be v_max scaled by angle between segments
        # Or just keep magnitude of v_prev_end for simplicity
        v_end_mag = np.linalg.norm(v_prev_end)
        if v_end_mag < 1e-3:
            v_end_mag = v_max * 0.8  # start with some velocity if previous zero

        v_end = next_dir * v_end_mag

    pos_seg, vel_seg, acc_seg = quintic_segment(p_start, p_end, v_prev_end, v_end, T, dt)

    # Update v_prev_end for next segment
    v_prev_end = vel_seg[-1]

    seg_len = len(pos_seg)
    time_seg = [t_now + j*dt for j in range(seg_len)]

    all_pos.extend(pos_seg)
    all_vel.extend(vel_seg)
    all_acc.extend(acc_seg)
    all_time.extend(time_seg)

    t_now = all_time[-1] + dt

# --- Convert to numpy arrays ---
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
# --- Save trajectory to YAML ---
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

# Define output path
output_yaml_path = os.path.join(config_path, "gen_trajectory_output.yaml")
with open(output_yaml_path, 'w') as f:
    yaml.dump(output_data, f, sort_keys=False)

print("\n✅ Quintic polynomial trajectory generated from via-points with velocity continuity.")
print(f"🖼️  Trajectory plot saved to:\n    {plot_path}")
print(f"📄  Trajectory data saved to:\n    {output_yaml_path}")
