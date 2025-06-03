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
    # คำนวณ yaw angle จากเวกเตอร์ p_from -> p_to ในระนาบ XY
    dx = p_to[0] - p_from[0]
    dy = p_to[1] - p_from[1]
    yaw = np.arctan2(dy, dx)  # radians
    return yaw

# --- Input ---
input_path = os.path.join(
    get_package_share_directory("sjtu_drone_control"),
    "config", "aruco_gazebo", "saw_marker_from_drone.yaml"
)

config_path = os.path.join(
    get_package_share_directory("sjtu_drone_control"),
    "config", "aruco_gazebo"
)
os.makedirs(config_path, exist_ok=True)
output_path = os.path.join(config_path, "gen_path.yaml")
image_path = os.path.join(config_path, "gen_path.jpg")

# --- Load and average marker positions ---
with open(input_path, 'r') as f:
    raw_data = yaml.safe_load(f)

marker_positions = []
for marker_id, poses in raw_data.items():
    pos = np.array([[p['position']['x'], p['position']['y'], p['position']['z']] for p in poses])
    avg_pos = np.mean(pos, axis=0)
    marker_positions.append({
        'id': marker_id,
        'pos': avg_pos
    })

# --- Group markers into Z rows (within 0.5m) ---
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

# --- Sort within rows and build path (snake pattern) ---
path_points = []
direction = 1
for row in rows:
    sorted_row = sorted(row, key=lambda m: m['pos'][1], reverse=(direction == -1))
    path_points.extend([m['pos'] for m in sorted_row])
    direction *= -1

# --- Interpolate via-points between each marker ---
via_points = []
for i in range(len(path_points) - 1):
    seg = interpolate_points(path_points[i], path_points[i + 1], step=0.2)
    via_points.extend(seg)
via_points.append(path_points[-1])  # include last point

# --- Save YAML with orientation (yaw) ---
yaml_data = {}
for i, pt in enumerate(via_points):
    if i < len(via_points) - 1:
        yaw = compute_yaw(pt, via_points[i+1])
    else:
        yaw = compute_yaw(via_points[i-1], pt)  # ทิศทางจุดสุดท้าย ใช้จากจุดก่อนหน้า
    yaml_data[f"via_point_{i}"] = {
        'x': float(pt[0]),
        'y': float(pt[1]),
        'z': float(pt[2]),
        'yaw': float(yaw)  # radians
    }

with open(output_path, 'w') as f:
    yaml.dump(yaml_data, f, sort_keys=False)

# --- Plot (Y vs Z) ---
y_list = [pt[1] for pt in via_points]
z_list = [pt[2] for pt in via_points]

plt.figure(figsize=(10, 7))
plt.plot(y_list, z_list, '-o', markersize=3, color='blue', label='Interpolated Path')

# --- Add arrows for movement direction ---
for i in range(len(y_list) - 1):
    dy = y_list[i + 1] - y_list[i]
    dz = z_list[i + 1] - z_list[i]
    plt.arrow(y_list[i], z_list[i], dy, dz,
              head_width=0.05, head_length=0.05,
              length_includes_head=True, color='orange')

# --- Plot original marker positions as red dots with marker ID ---
for row in rows:
    for m in row:
        y, z = m['pos'][1], m['pos'][2]
        plt.scatter(y, z, color='red', s=50, zorder=5)
        plt.text(y, z + 0.15, m['id'], fontsize=9, ha='center', va='bottom',
                 bbox=dict(facecolor='white', alpha=0.7, edgecolor='gray'))

plt.title("Interpolated Snake Path (Y-Z)")
plt.xlabel("Y (m)")
plt.ylabel("Z (m)")
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.tight_layout()
plt.savefig(image_path)
plt.close()

print("\n✅ Snake path with interpolated via-points and orientation (yaw) saved.")
print(f"📄 YAML saved to:\n    {output_path}")
print(f"🖼️  Path plot saved to:\n    {image_path}")

# --- Generate modified via_points (offset x=1.1, z=z-0.7) ---
via_points_modified = []
for pt in via_points:
    modified = np.array([pt[0] + 1.3, pt[1], pt[2] + 0.83])
    via_points_modified.append(modified)

# --- Save modified YAML ---
output_path_mod = os.path.join(config_path, "gen_path_modified.yaml")
yaml_data_mod = {}
for i, pt in enumerate(via_points_modified):
    yaml_data_mod[f"via_point_{i}"] = {
        'x': float(pt[0]),
        'y': float(pt[1]),
        'z': float(pt[2])
    }

with open(output_path_mod, 'w') as f:
    yaml.dump(yaml_data_mod, f, sort_keys=False)

# --- Plot comparison ---
y1 = [pt[1] for pt in via_points]
z1 = [pt[2] for pt in via_points]
y2 = [pt[1] for pt in via_points_modified]
z2 = [pt[2] for pt in via_points_modified]

plt.figure(figsize=(10, 7))
plt.plot(y1, z1, '-o', markersize=3, color='blue', label='Original Path')
plt.plot(y2, z2, '-o', markersize=3, color='green', label='Modified Path')

# --- Arrows for both paths ---
for i in range(len(y1) - 1):
    dy, dz = y1[i+1] - y1[i], z1[i+1] - z1[i]
    plt.arrow(y1[i], z1[i], dy, dz, head_width=0.05, head_length=0.05,
              length_includes_head=True, color='orange')

for i in range(len(y2) - 1):
    dy, dz = y2[i+1] - y2[i], z2[i+1] - z2[i]
    plt.arrow(y2[i], z2[i], dy, dz, head_width=0.05, head_length=0.05,
              length_includes_head=True, color='purple')

# --- Replot markers for context ---
for row in rows:
    for m in row:
        y, z = m['pos'][1], m['pos'][2]
        plt.scatter(y, z, color='red', s=50, zorder=5)
        plt.text(y, z + 0.15, m['id'], fontsize=9, ha='center', va='bottom',
                 bbox=dict(facecolor='white', alpha=0.7, edgecolor='gray'))

plt.title("Original vs Modified Path (Y-Z)")
plt.xlabel("Y (m)")
plt.ylabel("Z (m)")
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.tight_layout()

image_path_compare = os.path.join(config_path, "gen_path_compare.jpg")
plt.savefig(image_path_compare)
plt.close()

# --- Final log ---
print("\n✅ Modified path (offset x=1.1, z=z-0.7) generated.")
print(f"📄 YAML saved to:\n    {output_path_mod}")
print(f"🖼️  Comparison plot saved to:\n    {image_path_compare}")
