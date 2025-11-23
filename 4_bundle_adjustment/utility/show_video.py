import cv2
import glob
import os

def view_frames_interactively(image_folder, starting_offset=0, n_frames=None):
    """
    Opens a separate window to view a specific range of images with keyboard navigation.
    
    Args:
        image_folder (str): Path to images.
        starting_offset (int): Index to start viewing from (default 0).
        n_frames (int or None): Number of frames to view. If None or if 
                                (offset + n) > total, shows until the end.
    """
    
    # 1. Find and Sort Images
    search_pattern = os.path.join(image_folder, "*")
    exts = ('.png', '.jpg', '.jpeg', '.bmp', '.tif')
    all_files = sorted([f for f in glob.glob(search_pattern) if f.lower().endswith(exts)])
    
    total_available = len(all_files)
    if total_available == 0:
        print(f"No images found in folder: {image_folder}")
        return

    # 2. Apply Logic for Offset and N_Frames
    # Ensure offset is valid
    if starting_offset < 0: 
        starting_offset = 0
    if starting_offset >= total_available:
        print(f"Error: Starting offset {starting_offset} exceeds total images ({total_available}).")
        return

    # Calculate End Index
    if n_frames is None:
        end_index = total_available
    else:
        calculated_end = starting_offset + n_frames
        if calculated_end > total_available:
            print(f"Requested range exceeds total. Showing from {starting_offset} to End.")
            end_index = total_available
        else:
            end_index = calculated_end

    # Slice the file list
    selected_files = all_files[starting_offset:end_index]
    num_selected = len(selected_files)

    print(f"Total images in folder: {total_available}")
    print(f"Viewing range: {starting_offset} to {end_index} ({num_selected} frames)")
    print("Controls: [Right Arrow/n]=Next, [Left Arrow/p]=Prev, [q]=Quit")

    # 3. Setup Window
    window_name = "Frame Viewer"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    # cv2.resizeWindow(window_name, 1280, 720) # Uncomment to force specific size

    current_idx = 0 # Index relative to the slice (0 to n_frames)

    while True:
        # Get file path from the sliced list
        img_path = selected_files[current_idx]
        img = cv2.imread(img_path)
        
        if img is None:
            print(f"Failed to read {img_path}")
            break

        # Calculate Global Index for display
        global_idx = starting_offset + current_idx

        # Overlay Text info
        # Line 1: Relative progress in the selection
        text_local = f"View: {current_idx + 1} / {num_selected}"
        # Line 2: Absolute index in the folder (matching your dataset IDs)
        text_global = f"Global Index: {global_idx}"

        cv2.putText(img, text_local, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, text_global, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.8, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow(window_name, img)

        # 4. Wait for Input
        key = cv2.waitKey(0)

        # Navigation
        if key == ord('q') or key == 27: # Quit
            break
        
        elif key == 83 or key == ord('n') or key == 32: # Next
            if current_idx < num_selected - 1:
                current_idx += 1
            else:
                print("End of selected range.")
                
        elif key == 81 or key == ord('p'): # Prev
            if current_idx > 0:
                current_idx -= 1
            else:
                print("Start of selected range.")

    cv2.destroyAllWindows()

# --- Usage Examples ---
if __name__ == "__main__":
    view_frames_interactively('/home/filippo/Desktop/università/secondo_anno/primo_semestre/ADS/assignments/Autonomous_Driving_Systems/4_bundle_adjustment/dataset/images/TUM/sequence_49/images_downsampled', 180 , 100)