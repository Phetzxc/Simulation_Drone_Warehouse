#!/usr/bin/env python3

import os
import shutil

# Path ตั้งต้น
base_dir = '/home/phet/drone_ws/src/sjtu_drone/sjtu_drone_description/models/aruco_gazebo'
source_image_dir = '/home/phet/drone_ws/src/sjtu_drone/sjtu_drone_description/models/aruco_4x4'

# สร้าง marker_0 ถึง marker_60
for i in range(61):
    marker_name = f"marker_{i}"
    marker_path = os.path.join(base_dir, marker_name)
    mesh_path = os.path.join(marker_path, "meshes")

    # สร้างโฟลเดอร์ marker_{i} และ meshes
    os.makedirs(mesh_path, exist_ok=True)

    # ตั้งชื่อไฟล์ภาพต้นทาง
    src_image = os.path.join(source_image_dir, f"4x4_1000-{i}.png")
    dst_image = os.path.join(mesh_path, f"6x6_1000-{i}.png")  # เปลี่ยนชื่อเป็น 6x6_...

    # ตรวจสอบว่าภาพต้นทางมีอยู่จริงก่อนคัดลอก
    if os.path.exists(src_image):
        shutil.copy(src_image, dst_image)
        print(f"[✔] Copied and Renamed: {src_image} -> {dst_image}")
    else:
        print(f"[✘] Missing: {src_image}")
