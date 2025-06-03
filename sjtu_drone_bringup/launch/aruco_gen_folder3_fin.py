#!/usr/bin/env python3

import os
import shutil
from shutil import copyfile

# พาธ template
template_dir = "/home/phet/drone_ws/src/sjtu_drone/sjtu_drone_description/models/aruco_gazebo/marker_0"
base_path = "/home/phet/drone_ws/src/sjtu_drone/sjtu_drone_description/models/aruco_gazebo"
source_image_dir = "/home/phet/drone_ws/src/sjtu_drone/sjtu_drone_description/models/aruco_6x6"

# ตั้ง ID เริ่ม-สิ้นสุด
start_id = 0
end_id = 60

for i in range(start_id, end_id + 1):
    marker_name = f"marker_{i:02d}"
    new_dir = os.path.join(base_path, marker_name)
    mesh_dir = os.path.join(new_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)

    # คัดลอก model.config และแก้ไขชื่อ
    config_src = os.path.join(template_dir, "model.config")
    config_dst = os.path.join(new_dir, "model.config")
    copyfile(config_src, config_dst)
    with open(config_dst, "r") as f:
        config_data = f.read()
    config_data = config_data.replace("<name>Marker 0</name>", f"<name>Marker {i}</name>")
    config_data = config_data.replace("marker_0", marker_name)
    with open(config_dst, "w") as f:
        f.write(config_data)

    # คัดลอก model.sdf และแก้ชื่อ model + mesh path
    sdf_src = os.path.join(template_dir, "model.sdf")
    sdf_dst = os.path.join(new_dir, "model.sdf")
    copyfile(sdf_src, sdf_dst)
    with open(sdf_dst, "r") as f:
        sdf_data = f.read()
    dae_name = f"{i:02d}.dae"
    sdf_data = sdf_data.replace("marker_0", marker_name)
    sdf_data = sdf_data.replace("meshes/6x6.dae", f"meshes/{dae_name}")
    sdf_data = sdf_data.replace("<model name='Marker_0'>", f"<model name='Marker_{i}'>")
    with open(sdf_dst, "w") as f:
        f.write(sdf_data)

    # คัดลอก .dae และแก้ชื่อ texture + model name
    dae_src = os.path.join(template_dir, "meshes", "6x6.dae")
    dae_dst = os.path.join(mesh_dir, dae_name)
    copyfile(dae_src, dae_dst)
    with open(dae_dst, "r") as f:
        dae_data = f.read()
    dae_data = dae_data.replace("6x6_1000-0.png", f"6x6_1000-{i}.png")
    dae_data = dae_data.replace("<model name='Marker_0'>", f"<model name='Marker_{i}'>")
    with open(dae_dst, "w") as f:
        f.write(dae_data)

    # คัดลอก PNG texture และ rename
    src_image = os.path.join(source_image_dir, f"6x6_1000-{i}.png")
    dst_image = os.path.join(mesh_dir, f"6x6_1000-{i}.png")
    if os.path.exists(src_image):
        shutil.copy(src_image, dst_image)
        print(f"[✔] marker_{i:02d}: image copied -> {dst_image}")
    else:
        print(f"[✘] marker_{i:02d}: image not found -> {src_image}")

print("✅ All ArUco marker models, configs, and textures generated.")
