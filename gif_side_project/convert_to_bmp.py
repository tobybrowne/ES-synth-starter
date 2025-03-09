from PIL import Image, ImageSequence

gif_path = "dolphin.gif"
output_folder = "frames"

# Convert GIF frames to monochrome bitmaps
im = Image.open(gif_path)
for i, frame in enumerate(ImageSequence.Iterator(im)):
    frame = frame.convert("1")  # Convert to 1-bit
    frame = frame.resize((128, 64))  # Resize to OLED resolution
    frame.save(f"{output_folder}/frame_{i}.bmp")
