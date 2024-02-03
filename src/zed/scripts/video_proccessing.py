"""
TODO:
- multi-object tracking ???
- disparity parameters tuning using depth_tune_stereo
- computers communication
"""

import time
from tools import *
from matplotlib import pyplot as plt


def main(video_name, start_time=0, is_stereo=False):
    if is_stereo:
        n_column = 2
        video_path = 'stereo_video/' + video_name
    else:
        n_column = 1
        video_path = 'video/' + video_name

    # Create a VideoCapture object
    cap = cv2.VideoCapture(video_path)

    # Check if the video file was successfully opened
    if not cap.isOpened():
        print("Error: Could not open video file.")
        exit()

    fps = int(cap.get(cv2.CAP_PROP_FPS))
    source_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    source_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) / n_column)

    # Define colors in BGR
    color_of = (255, 0, 0)  # BLUE
    color_grad = (0, 0, 255)  # RED
    color_horizon = (0, 255, 0)  # GREEN
    color_box = (255, 0, 255)  # PURPLE

    # Define the downscale factor
    downscale_factor = 2

    # Calculate the new size based on the scale factor
    width = int(source_width / downscale_factor)
    height = int(source_height / downscale_factor)

    horizon_estimator = HorizonEstimator()
    saliency_estimator = SaliencyEstimator()
    tracking_window = TrackingWindow(height, width)

    # Erosion & dilatation are applied on active pixel -> used for computing the saliency mask
    connection_kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3, 3))
    erosion_kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(5, 5))
    dilatation_kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(5, 5))

    # Parameters for corner detection
    features_params = {
        'maxCorners': 10000,
        'qualityLevel': 0.01,
        'minDistance': 0.1
    }

    # Parameters for pyramidal Lucas-Kanade optical flow
    lk_params = {
        'winSize': (15, 15),
        'maxLevel': 2,
        'criteria': (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
    }

    # Set duration (in seconds)
    frame_skip = 4
    duration = 40
    dt = frame_skip / fps

    # Calculate the corresponding frame numbers
    n_frame = duration * fps
    start_frame = int(start_time * fps)

    # Set the initial frame position
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    # Define variables to initialize later
    old_frame = None
    points_old_frame = []

    # Read and display frames within the specified time range
    for k_frame in range(n_frame):
        if k_frame % frame_skip == 0:
            pass

        t_loop = time.time()

        # Read a frame from the video
        ret, read_frame = cap.read()

        # Check if the video has ended
        if not ret:
            break

        # Only consider the left camera flow
        source_frame = read_frame[:, :source_width]

        # Downscale and convert to grayscale
        frame_bgr = cv2.resize(source_frame, (width, height))
        frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        # Draw points and line on a separate image
        canvas = source_frame.copy()

        # Estimate a line for the horizon
        hpx, hpy, border_y, horizontal_mask = horizon_estimator.compute(frame)

        # Scatter gradient points on the image
        for (x, y) in zip(hpx, hpy):
            cv2.circle(canvas, (int(downscale_factor * x), int(downscale_factor * y)), 3, color_grad, -1)

        # Draw the horizontal line on the image
        border_x = np.array([[[0.]], [[width - 1.]]])
        ransac_points = np.concatenate((border_x, border_y), axis=2).astype(np.int32)
        cv2.polylines(canvas, [ransac_points * downscale_factor], isClosed=False, color=color_horizon, thickness=2)

        if (old_frame is not None) and (points_old_frame is not None):
            points_new_frame, status, err = cv2.calcOpticalFlowPyrLK(old_frame, frame,
                                                                     points_old_frame,
                                                                     None, **lk_params)

            for p0, p1 in zip(points_old_frame[status == 1], points_new_frame[status == 1]):
                x0, y0 = (downscale_factor * p0).ravel()
                x1, y1 = (downscale_factor * p1).ravel()
                cv2.line(canvas, (int(x1), int(y1)), (int(x0), int(y0)), color_of, 3)

        # Current frame becomes the old one
        old_frame = frame.copy()

        # Compute the saliency map (where to look)
        saliency_mask = saliency_estimator.compute(frame_bgr, horizontal_mask)

        # Compute good feature point to keep track of
        mask = horizontal_mask & saliency_mask

        mask = cv2.dilate(mask, connection_kernel)
        mask = cv2.erode(mask, erosion_kernel)
        mask = cv2.dilate(mask, dilatation_kernel)

        points_old_frame = cv2.goodFeaturesToTrack(frame, mask=mask, **features_params)

        wx, wy, wsx, wsy = tracking_window.update(points_old_frame, downscale_factor)
        cv2.rectangle(canvas, (int(wx-wsx), int(wy-wsy)), (int(wx+wsx), int(wy+wsy)), color_box, thickness=2)

        debug = cv2.cvtColor(mask * 255, cv2.COLOR_GRAY2BGR)
        debug = cv2.resize(debug, (source_width, source_height))

        # Stack images vertically or horizontally
        layout = np.vstack((canvas, debug)) if height < width else np.hstack((canvas, debug))

        # Show lower part of layout
        cv2.imshow('Canvas & Debug', layout)  # [int(source_height / 3):])

        # Pause the loop if SPACE is pressed
        paused = False
        if (cv2.waitKey(25) & 0xFF) == ord(' '):
            paused = True
            while (cv2.waitKey(25) & 0xFF) != ord(' '):
                continue

        t_remaining = EPSILON if paused else dt - (time.time() - t_loop)
        if t_remaining > 0:
            print("Time left:", t_remaining, 'seconds.')
            plt.pause(t_remaining)
        else:
            print("Out of time of", -t_remaining, 'seconds.')
            plt.pause(EPSILON)

    # Release the VideoCapture object and close the window
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # main('duck_close.avi', is_stereo=True)
    main('goodbye.mp4', start_time=10)
    # main('skyfall.mp4', start_time=10)  # oblique horizon
    # main('sunny.mp4', start_time=20)  # too many things over the horizon
    # main('forest.mp4', start_time=10)  # saliency issue -> sun reflection
    # main('double.mp4', start_time=10)  # won't happen -> too far from water surface
