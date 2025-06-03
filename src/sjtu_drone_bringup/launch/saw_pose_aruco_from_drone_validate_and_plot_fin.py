import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory

# --- Input path setup (original YAML with marker_i entries) ---
input_path = os.path.join(
    get_package_share_directory("sjtu_drone_control"),
    "config", "aruco_gazebo", "saw_marker_from_drone.yaml"
)

# --- Output path setup (averaged result YAML + JPG image) ---
config_path = os.path.join(
    get_package_share_directory("sjtu_drone_control"),
    "config", "aruco_gazebo"
)
os.makedirs(config_path, exist_ok=True)
output_path = os.path.join(config_path, "estimate_marker_from_drone.yaml")
image_path = os.path.join(config_path, "estimate_marker_from_drone.jpg")

# --- Load original marker observations ---
with open(input_path, 'r') as f:
    raw_data = yaml.safe_load(f)

# --- Compute average pose per marker ---
avg_data = {}
x_list, y_list, z_list, id_list = [], [], [], []

for marker_id, poses in raw_data.items():
    pos = np.array([[p['position']['x'], p['position']['y'], p['position']['z']] for p in poses])
    ori = np.array([[p['orientation']['x'], p['orientation']['y'], p['orientation']['z'], p['orientation']['w']] for p in poses])

    avg_pos = np.mean(pos, axis=0)
    avg_ori = np.mean(ori, axis=0)  # simple average of quaternion components

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

# --- Save to output YAML ---
with open(output_path, 'w') as f:
    yaml.dump(avg_data, f, sort_keys=False)

# --- Plot marker positions (XY and YZ) ---
plt.figure(figsize=(14, 6))

# Subplot เดียว: Y vs Z
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
plt.savefig(image_path)
plt.close()


# --- Print results ---
print("\n✅ Done!")
print(f"📄 Averaged YAML saved to:\n    {output_path}")
print(f"🖼️  Marker plot saved to:\n    {image_path}")
