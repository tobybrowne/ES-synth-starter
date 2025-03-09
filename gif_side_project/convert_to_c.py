import os

def bmp_to_c_array(file_path):
    with open(file_path, "rb") as f:
        bmp_data = f.read()

    # Convert to a comma-separated hex format
    hex_data = ", ".join(f"0x{byte:02X}" for byte in bmp_data)

    # Create a valid C array name
    array_name = os.path.basename(file_path).replace(".", "_")

    return f"const unsigned char {array_name}[] PROGMEM = {{ {hex_data} }};\n"

# Path to your BMP frames
frames_folder = "frames_resized"
output_file = "frames.c"

# Get all BMP files
bmp_files = sorted([f for f in os.listdir(frames_folder) if f.endswith(".bmp")])

# Convert all BMPs to C arrays
c_code = "#include <avr/pgmspace.h>\n\n"  # Add necessary includes
for bmp in bmp_files:
    c_code += bmp_to_c_array(os.path.join(frames_folder, bmp))

# Save to a .c file
with open(output_file, "w") as f:
    f.write(c_code)

print(f"Conversion complete! C arrays saved in {output_file}")
