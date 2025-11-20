import cv2
from PIL import Image

def frames_extraction(video_path):
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"ERROR: could not open file {video_path}")

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    for frame_id in range(0, total_frames):

        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)

        success, frame = cap.read()

        if success:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(frame_rgb)
            # save frame to device as idx.png where idx is always 6 digits, pad 0 to the left
            
        else:
            print(f"something went wrong")
            exit()
    

    cap.release()
    print(f" # saved {total_frames} to device")
    

video_path = ""
