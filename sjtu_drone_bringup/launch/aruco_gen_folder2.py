import os
from shutil import copyfile

# พาธ template
template_dir = "/home/phet/drone_ws/src/sjtu_drone/sjtu_drone_description/models/aruco_gazebo/marker_0"
base_path = "/home/phet/drone_ws/src/sjtu_drone/sjtu_drone_description/models/aruco_gazebo"

# ต้องการสร้าง marker ID 1-10
for marker_id in range(1, 11):
    marker_name = f"marker_{marker_id:02d}"
    new_dir = os.path.join(base_path, marker_name)
    
    if not os.path.exists(new_dir):
        os.makedirs(new_dir)
    
    # คัดลอกไฟล์ทั้งหมดจาก marker_00 ไป marker_X
    copyfile(os.path.join(template_dir, "model.config"), os.path.join(new_dir, "model.config"))
    copyfile(os.path.join(template_dir, "model.sdf"), os.path.join(new_dir, "model.sdf"))
    
    # คัดลอก mesh และเปลี่ยนชื่อ
    mesh_dir = os.path.join(new_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    new_mesh_name = f"{marker_id:02d}.dae"
    copyfile(
        os.path.join(template_dir, "meshes", "6x6.dae"),
        os.path.join(mesh_dir, new_mesh_name)
    )

    # แก้ path ภายใน model.sdf
    with open(os.path.join(new_dir, "model.sdf"), "r") as f:
        sdf_data = f.read()
    
    # เปลี่ยนชื่อ <mesh><uri> ให้ตรงกับ marker ใหม่
    sdf_data = sdf_data.replace("meshes/6x6.dae", f"meshes/{new_mesh_name}")
    sdf_data = sdf_data.replace("marker_0", marker_name)  # ปรับชื่อ model ถ้ามีใน sdf
    
    with open(os.path.join(new_dir, "model.sdf"), "w") as f:
        f.write(sdf_data)

    # แก้ model.config
    with open(os.path.join(new_dir, "model.config"), "r") as f:
        config_data = f.read()
    config_data = config_data.replace("marker_0", marker_name)
    with open(os.path.join(new_dir, "model.config"), "w") as f:
        f.write(config_data)

print("✅ ArUco marker models generated successfully.")
