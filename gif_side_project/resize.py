from PIL import Image
import os

input_folder = "frames"  # Folder containing BMP images
output_folder = "frames_resized"  # New folder for resized images
os.makedirs(output_folder, exist_ok=True)

for filename in os.listdir(input_folder):
    if filename.endswith(".bmp"):
        img_path = os.path.join(input_folder, filename)
        img = Image.open(img_path).convert("1")  # Convert to 1-bit
        img_resized = img.resize((128, 32))  # Resize to match your OLED

        output_path = os.path.join(output_folder, filename)
        img_resized.save(output_path)
        print(f"Resized and saved: {output_path}")

print("✅ All frames resized to 128x32!")
