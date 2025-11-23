import os
import shutil

# --- CONFIGURATION ---
SOURCE_DIR = "../dataset/images/TUM/sequence_49/images"          # Your original folder
DEST_DIR = "../dataset/images/TUM/sequence_49/images_downsampled"  # New folder
STEP = 3                       # Keep 1, skip 2 (Indices: 0, 3, 6, 9...)
# ---------------------

def decimate_images():
    # 1. Setup
    if not os.path.exists(SOURCE_DIR):
        print(f"Error: Source folder '{SOURCE_DIR}' not found.")
        return

    if os.path.exists(DEST_DIR):
        print(f"Warning: Destination '{DEST_DIR}' already exists. Merging/Overwriting.")
    else:
        os.makedirs(DEST_DIR)

    # 2. Get sorted list of images
    # Sorting is critical so we skip the correct chronological frames
    valid_exts = ('.png', '.jpg', '.jpeg', '.bmp', '.tif')
    all_files = sorted([f for f in os.listdir(SOURCE_DIR) if f.lower().endswith(valid_exts)])
    
    if not all_files:
        print("No images found.")
        return

    print(f"Found {len(all_files)} images.")
    print(f"Keeping every {STEP}rd frame (Decimation).")

    # 3. Process
    count = 0
    # The slice [::STEP] handles the math for us (0, 3, 6, 9...)
    for i, filename in enumerate(all_files[::STEP]):
        src_path = os.path.join(SOURCE_DIR, filename)
        
        # Determine extension
        ext = os.path.splitext(filename)[1]
        
        # Rename strictly sequentially: 0000, 0001, 0002...
        # Most VO pipelines (like KITTI) fail if numbers skip (e.g. 0000, 0003)
        new_name = f"{count:06d}{ext}" 
        dst_path = os.path.join(DEST_DIR, new_name)
        
        shutil.copy2(src_path, dst_path)
        count += 1

    print(f"Done! Reduced from {len(all_files)} to {count} images.")
    print(f"Saved in folder: {DEST_DIR}")

if __name__ == "__main__":
    decimate_images()