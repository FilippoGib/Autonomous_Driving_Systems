import cv2
import glob
import os

def view_frames_interactively(image_folder, starting_offset=0, n_frames=None):

    search_pattern = os.path.join(image_folder, "*")
    exts = ('.png', '.jpg', '.jpeg', '.bmp', '.tif')
    all_files = sorted([f for f in glob.glob(search_pattern) if f.lower().endswith(exts)])
    
    total_available = len(all_files)
    if total_available == 0:
        print(f"No images found in folder: {image_folder}")
        return

    if starting_offset < 0: 
        starting_offset = 0
    if starting_offset >= total_available:
        print(f"Error: Starting offset {starting_offset} exceeds total images ({total_available}).")
        return

    if n_frames is None:
        end_index = total_available
    else:
        calculated_end = starting_offset + n_frames
        if calculated_end > total_available:
            print(f"Requested range exceeds total. Showing from {starting_offset} to End.")
            end_index = total_available
        else:
            end_index = calculated_end

    selected_files = all_files[starting_offset:end_index]
    num_selected = len(selected_files)

    print(f"Total images in folder: {total_available}")
    print(f"Viewing range: {starting_offset} to {end_index} ({num_selected} frames)")
    print("Controls: [Right Arrow/n]=Next, [Left Arrow/p]=Prev, [q]=Quit")

    window_name = "Frame Viewer"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    current_idx = 0

    while True:
        img_path = selected_files[current_idx]
        img = cv2.imread(img_path)
        
        if img is None:
            print(f"Failed to read {img_path}")
            break

        global_idx = starting_offset + current_idx

        text_local = f"View: {current_idx + 1} / {num_selected}"
        text_global = f"Global Index: {global_idx}"

        cv2.putText(img, text_local, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, text_global, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.8, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow(window_name, img)

        key = cv2.waitKey(0)

        if key == ord('q') or key == 27:
            break
        
        elif key == 83 or key == ord('n') or key == 32:
            if current_idx < num_selected - 1:
                current_idx += 1
                
        elif key == 81 or key == ord('p'):
            if current_idx > 0:
                current_idx -= 1

    cv2.destroyAllWindows()

if __name__ == "__main__":
    view_frames_interactively('/home/filippo/Desktop/università/secondo_anno/primo_semestre/ADS/assignments/Autonomous_Driving_Systems/4_bundle_adjustment/dataset/images/TUM/sequence_49/images_downsampled', 180 , 100)