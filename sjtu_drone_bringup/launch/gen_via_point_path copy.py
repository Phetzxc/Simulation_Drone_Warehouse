import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory

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

# Paths
pkg_path = get_package_share_directory("sjtu_drone_control")
input_path = os.path.join(pkg_path, "config", "aruco_gazebo", "saw_marker_from_drone.yaml")
config_path = os.path.join(pkg_path, "config", "aruco_gazebo")
os.makedirs(config_path, exist_ok=True)

output_path = os.path.join(config_path, "gen_path.yaml")
output_path_mod = os.path.join(config_path, "gen_path_modified.yaml")
image_path = os.path.join(config_path, "compare_path.jpg")

# Load and average marker positions
with open(input_path, 'r') as f:
    raw_data = yaml.safe_load(f)

marker_positions = []
for marker_id, poses in raw_data.items():
    pos = np.array([[p['position']['x'], p['position']['y'], p['position']['z']] for p in poses])
    avg_pos = np.mean(pos, axis=0)
    marker_positions.append({'id': marker_id, 'pos': avg_pos})

# Group markers into Z rows
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

# Snake path generation
path_points = []
direction = 1
for row in rows:
    sorted_row = sorted(row, key=lambda m: m['pos'][1], reverse=(direction == -1))
    path_points.extend([m['pos'] for m in sorted_row])
    direction *= -1

# Interpolation between points
via_points = []
for i in range(len(path_points) - 1):
    seg = interpolate_points(path_points[i], path_points[i + 1], step=0.2)
    via_points.extend(seg)
via_points.append(path_points[-1])

# Save original path with quaternion
yaml_data = {}
for i, pt in enumerate(via_points):
    if i < len(via_points) - 1:
        yaw = compute_yaw(pt, via_points[i+1])
    else:
        yaw = compute_yaw(via_points[i-1], pt)
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    yaml_data[f"via_point_{i}"] = {
        'x': float(pt[0]),
        'y': float(pt[1]),
        'z': float(pt[2]),
        'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}
    }

with open(output_path, 'w') as f:
    yaml.dump(yaml_data, f, sort_keys=False)

# Generate modified path (offset)
via_points_mod = [np.array([pt[0]+1.3, pt[1], pt[2]+0.83]) for pt in via_points]

# Save modified path with quaternion
yaml_data_mod = {}
for i, pt in enumerate(via_points_mod):
    if i < len(via_points_mod) - 1:
        yaw = compute_yaw(pt, via_points_mod[i+1])
    else:
        yaw = compute_yaw(via_points_mod[i-1], pt)
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    yaml_data_mod[f"via_point_{i}"] = {
        'x': float(pt[0]),
        'y': float(pt[1]),
        'z': float(pt[2]),
        'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}
    }

with open(output_path_mod, 'w') as f:
    yaml.dump(yaml_data_mod, f, sort_keys=False)

# --- Plot comparison ---
y1, z1 = [pt[1] for pt in via_points], [pt[2] for pt in via_points]
y2, z2 = [pt[1] for pt in via_points_mod], [pt[2] for pt in via_points_mod]

plt.figure(figsize=(10, 7))
plt.plot(y1, z1, '-o', label="Original Path", color='blue', markersize=3)
plt.plot(y2, z2, '-o', label="Modified Path (Offset)", color='green', markersize=3)

# Arrows for direction
for i in range(len(y1) - 1):
    plt.arrow(y1[i], z1[i], y1[i+1]-y1[i], z1[i+1]-z1[i], color='blue',
              head_width=0.05, head_length=0.05, length_includes_head=True)
for i in range(len(y2) - 1):
    plt.arrow(y2[i], z2[i], y2[i+1]-y2[i], z2[i+1]-z2[i], color='green',
              head_width=0.05, head_length=0.05, length_includes_head=True)

# Plot red dots for original marker positions with ID
for marker in marker_positions:
    y, z = marker['pos'][1], marker['pos'][2]
    plt.plot(y, z, 'ro')  # red dot
    plt.text(y + 0.02, z + 0.02, f"id: {marker['id']}", fontsize=8, color='red')

plt.title("Comparison of Original and Offset Paths (Y-Z plane)")
plt.xlabel("Y (m)")
plt.ylabel("Z (m)")
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.tight_layout()
plt.savefig(image_path)
plt.close()

# --- Done ---
print("\n✅ Snake path (original + offset) with quaternion orientation saved.")
print(f"📄 Original YAML:   {output_path}")
print(f"📄 Modified YAML:   {output_path_mod}")
print(f"🖼️  Path plot:      {image_path}")
