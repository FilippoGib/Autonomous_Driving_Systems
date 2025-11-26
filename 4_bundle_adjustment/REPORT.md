## 1.  Bare minimum implementation
First I decided to change the dataset to [TUM Monocular Vision Odometry Dataset](https://cvg.cit.tum.de/data/datasets/mono-dataset). This dataset includes fisheye frames that follow the FOV camera model, therefore the available camera parameters are the following: fx, fy, cx, cy, omega. \
Since opencv does not natively handle a single distorion parameter I had to implement the function by hand. \
I wanted to use 100 frames to reconstuct the pose therefore I needed to implement the second point of the bare minimum assignment which was to implement the logic to refurnish the tracked features if they drop below a certain threshold. \
However this was not enough as I also had to refurnish the 3D point in order to be always able to perform the PnP.

## 2. Additional steps

### 1. Try a different feature detector
I decided to try to exploit a different feature detector and looking on the web I came across [cv2.goodFeaturesToTrack()](https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga1d6bb77486c8f92d79c8793ad995d541), which is a opencv function that implements the Shi-Tomasi corner detector.

### 2. Different dataset
I completed the "bare minimum" requirements using a fisheye model, I decided to try to use a stereo dataset that uses a *equidistant* distorion model: [TUM Visual-Inertial Dataset](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset)