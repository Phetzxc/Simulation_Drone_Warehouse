import os
from PIL import Image, ImageDraw, ImageFont

# Path ปัจจุบันของสคริปต์
script_dir = os.path.dirname(os.path.abspath(__file__))
output_dir = os.path.join(script_dir, "number_labels_71x31")
os.makedirs(output_dir, exist_ok=True)

# ฟอนต์ที่ใช้ใน Linux
font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

# ขนาดภาพที่ต้องการ
canvas_width, canvas_height = 71, 31

def get_max_font_size(text, max_width, max_height, font_path):
    size = 1
    last_valid_size = 1
    while size < 300:
        try:
            font = ImageFont.truetype(font_path, size)
        except IOError:
            return 20
        bbox = font.getbbox(text)
        width = bbox[2] - bbox[0]
        height = bbox[3] - bbox[1]
        if width >= max_width or height >= max_height:
            break
        last_valid_size = size
        size += 1
    return max(last_valid_size, 1)

for number in range(61):
    label_text = f"#{number}"
    best_size = get_max_font_size(label_text, canvas_width, canvas_height, font_path)

    try:
        font = ImageFont.truetype(font_path, size=best_size)
    except IOError:
        font = ImageFont.load_default()

    bbox = font.getbbox(label_text)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]
    text_x = (canvas_width - text_width) // 2
    text_y = (canvas_height - text_height) // 2

    canvas = Image.new("L", (canvas_width, canvas_height), 255)
    draw = ImageDraw.Draw(canvas)
    draw.text((text_x, text_y), label_text, fill=0, font=font)

    canvas.save(os.path.join(output_dir, f"aruco_mark_{number}_num.png"))

print("สร้างเสร็จแล้วในโฟลเดอร์:", output_dir)
