#!/usr/bin/env python3

import cv2
import numpy as np

from skimage.filters import sobel_h
from sklearn.linear_model import RANSACRegressor

EPSILON = 1e-5

# Define colors in BGR
COLOR_HP = (0, 0, 255)  # RED
COLOR_BOX = (255, 0, 255)  # PURPLE
COLOR_HORIZON = (0, 255, 0)  # GREEN


def take_top_n(indices, n_top, top_distance=5):
    # top_distance corresponds to the minimum distance in pixels between 2 tops

    n_horizontal, n_vertical = indices.shape
    top_n = np.full((n_top, n_vertical), -top_distance)

    for x in range(n_vertical):
        k_top = 0
        y = 0  # start with smallest vertical position
        while k_top < n_top and y < n_horizontal:
            top = indices[y, x]
            if np.min(np.abs(top - top_n[:, x])) > top_distance:
                top_n[k_top, x] = top
                k_top += 1
            y += 1

    return top_n


class HorizonEstimator:
    def __init__(self, n_vertical=8, n_top=2, alpha=0.2, max_residual=8., max_slope_change=8e-2, max_bias_change=32.):
        self.n_vertical = n_vertical
        self.n_top = n_top
        self.alpha = alpha

        self.max_residual = max_residual
        self.max_slope_change = max_slope_change  # no units
        self.max_bias_change = max_bias_change  # in pixels

        self.slope = 0.  # looking for a horizontal line
        self.bias = None  # no ideas what this value should be

    def compute(self, image):
        def is_data_valid(X, Y):
            dx = X[-1, 0] - X[0, 0]
            dy = Y[-1, 0] - Y[0, 0]

            if self.bias is None:
                return True
            else:
                if abs(dx) > EPSILON:
                    a = (dy / dx)
                    b = Y[0, 0] - a * X[0, 0]
                    return (abs(self.slope - a) < self.max_slope_change) \
                           and (abs(self.bias - b) < self.max_bias_change)
                else:
                    return False

        # Compute some useful values
        height, width, *_ = image.shape
        horizontal_half_step = int(0.5 * width / self.n_vertical)

        # Compute the horizontal gradient -> smallest = from far to close -> Guerledan hypothesis
        horizontal_grad = sobel_h(image)

        # Filter top n_top largest gradient points
        ks_row = np.tile(np.arange(height)[:, np.newaxis], self.n_vertical)
        ks_row += (horizontal_grad[:, horizontal_half_step::2 * horizontal_half_step] < EPSILON) * height
        indices = np.sort(ks_row, axis=0)

        x = horizontal_half_step + (np.arange(self.n_vertical * self.n_top) // self.n_top) * (2 * horizontal_half_step)
        y = np.ravel(take_top_n(indices, self.n_top), order='F')
        overflow_mask = (y < height)

        # Reshape for visualization purposes
        horizontal_point_x = x[overflow_mask].reshape(-1, 1)
        horizontal_point_y = y[overflow_mask].reshape(-1, 1)

        # Compute the best line fitting our point using RANSAC
        ransac = RANSACRegressor(residual_threshold=self.max_residual, is_data_valid=is_data_valid)

        try:
            ransac.fit(horizontal_point_x, horizontal_point_y)
        except ValueError:
            print('RANSAC value error!')
            self.slope = 0.
            self.bias = None
            return horizontal_point_x, horizontal_point_y, np.zeros(shape=(2, 1, 1), dtype=float), np.ones(shape=(height, width), dtype=np.uint8)
        else:
            # Update slope
            ransac_pred_y = ransac.predict(np.array([[0.], [width - 1.]]))
            bias, y_w = ransac_pred_y.flatten()
            slope = (y_w - bias) / width

            # Average things throught time to avoid oscillation
            self.slope = (1. - self.alpha) * self.slope + self.alpha * slope
            self.bias = bias if self.bias is None else (1. - self.alpha) * self.bias + self.alpha * bias

            # Compute horizon mask
            mask = (np.arange(height).reshape(-1, 1) > ransac.predict(np.arange(width).reshape(-1, 1)).T)

            return horizontal_point_x, horizontal_point_y, ransac_pred_y.reshape(-1, 1, 1), mask.astype(np.uint8)


class SaliencyEstimator:
    def __init__(self, n_sigma=4.):
        self.n_sigma = n_sigma

    def compute(self, img_bgr, mask):
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        mask = mask.astype(bool)

        mean_hsv = np.mean(img_hsv[mask], axis=0).reshape((1, 1, 3))
        std_hsv = np.std(img_hsv[mask], axis=0).reshape((1, 1, 3))

        saliency = np.linalg.norm((img_hsv - mean_hsv) / (EPSILON + std_hsv), axis=2)
        saliency_mask = (saliency > self.n_sigma).astype(np.uint8)

        return saliency_mask


def bounding_box_from_mask(mask):
    # Find indices of True values
    true_indices = np.argwhere(mask)

    # Calculate bounding box coordinates
    y0, x0 = np.min(true_indices, axis=0)
    y1, x1 = np.max(true_indices, axis=0)

    return x0, y0, x1, y1


class Obstacle:
    def __init__(self, keypoints, label):
        self.keypoints = keypoints
        self.label = label

        self.age = 0
    
    def update(self, new_keypoints, label):
        self.keypoints = new_keypoints
        self.label = label
        self.age += 1


class SIFTracker:
    def __init__(self, top_two_ratio=0.64, keeping_ratio=0.32, pegi=1):
        self.top_two_ratio = top_two_ratio
        self.keeping_ratio = keeping_ratio
        self.pegi = pegi

        # Initialize SIFT detector
        self.sift = cv2.SIFT_create()

        # Initialize FLANN matcher
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        self.frame = None
        self.keypoints = None
        self.descriptors = None

        self.obstacles = []

    def compute(self, frame, mask):
        # Find keypoints and descriptors in the current frame
        keypoints, descriptors = self.sift.detectAndCompute(frame, mask)

        if len(keypoints) < 2:
            self.frame = None
            return []

        # Perform connected component labeling
        num_labels, labels = cv2.connectedComponents(mask, connectivity=8)

        if self.frame is None:
            remainding_keypoints = keypoints
        else:
            # Match descriptors between the old and the new frame : take best 2
            matches = self.flann.knnMatch(self.descriptors, descriptors, k=2)

            # Apply ratio test
            good_matches = []
            for m,n in matches:
                if m.distance < self.top_two_ratio * n.distance:
                    good_matches.append(m)
            
            # Extract matched keypoints
            n_matched_keypoints = len(good_matches)
            matched_keypoints_old = [self.keypoints[m.queryIdx] for m in good_matches]
            matched_keypoints_new = [keypoints[m.trainIdx] for m in good_matches]

            new_obstacles = []
            matched_keypoints_flag = [True for _ in range(n_matched_keypoints)]
            for obstacle in self.obstacles:
                keypoint_clustering = [[] for _ in range(1 + num_labels)]
                for old_keypoint in obstacle.keypoints:
                    try:
                        k = matched_keypoints_old.index(old_keypoint)
                    except ValueError:
                        continue
                    else:
                        matched_keypoints_flag[k] = False
                        new_keypoint = matched_keypoints_new[k]
                        x, y = new_keypoint.pt
                        label = labels[int(y),int(x)]
                        keypoint_clustering[label].append(new_keypoint)
                new_label, new_keypoints = max(enumerate(keypoint_clustering), key=lambda tup: len(tup[1]))
                if new_label > 0 and len(new_keypoints) > self.keeping_ratio * len(obstacle.keypoints):
                    obstacle.update(new_keypoints, new_label)
                    new_obstacles.append(obstacle)
            
            self.obstacles = new_obstacles
            remainding_keypoints = [matched_keypoints_new[k] for k in range(n_matched_keypoints) if matched_keypoints_flag[k]]         
        
        # Add remainding keypoints
        obstacles = [[] for _ in range(1 + num_labels)]
        for keypoint in remainding_keypoints:
            x, y = keypoint.pt
            label = labels[int(y),int(x)]
            obstacles[label].append(keypoint)
        
        for (label, obs_keypoints) in enumerate(obstacles):  # iterate through assigned keypoints
            if label > 0 and len(obs_keypoints) > 0:  # only consider non empty components
                obstacle = Obstacle(obs_keypoints, label)
                self.obstacles.append(obstacle)

        # The new becomes the old
        self.frame = frame.copy()
        self.keypoints = keypoints
        self.descriptors = descriptors

        verified_obstacles = []
        for obstacle in self.obstacles:
            if obstacle.age >= self.pegi:
                obstacle_mask = (labels == obstacle.label)
                if np.any(obstacle_mask):
                    x0, y0, x1, y1 = bounding_box_from_mask(obstacle_mask)
                    verified_obstacles.append((x0, y0, x1, y1))
        
        return verified_obstacles


class VideoProcessor:
    def __init__(self):
        # Define the downscale factor
        self.downscale_factor = 4

        self.horizon_estimator = HorizonEstimator()
        self.saliency_estimator = SaliencyEstimator()
        self.obstacle_tracker = SIFTracker()

        # Erosion & dilatation are applied on active pixel -> used for computing the saliency mask
        self.dilate_depth = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(11, 11))
        self.erode_depth = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(5, 5))
        
        self.connection_kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3, 3))
        self.erosion_kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(5, 5))
        self.dilatation_kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(11, 11))

        # Define variables to initialize later
        self.left_crop = None
    
    def process(self, source_frame, source_depth, return_stategy=False):
        # Crop left side of the image (where left & right POV doesn't overlapped)
        if self.left_crop is None:  # compute it once and for all
            self.left_crop = np.argmin(np.all(source_depth > 30e3, axis=0))  # look for too far area
        source_frame = source_frame[:, self.left_crop:]
        source_depth = source_depth[:, self.left_crop:]

        # Get image dimensions
        source_height, source_width, *_ = source_frame.shape
        height = source_height // self.downscale_factor
        width = source_width // self.downscale_factor

        # Downscale
        frame_bgr = cv2.resize(source_frame, (width, height))
        depth_map = cv2.resize(source_depth, (width, height))

        # Draw points and line on a separate image (for debugging only)
        frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        # Estimate a line for the horizon
        depth_mask = (depth_map < 20e3).astype(np.uint8)
        dilated_depth_mask = cv2.dilate(depth_mask, self.dilate_depth)
        hpx, hpy, border_y, horizontal_mask = self.horizon_estimator.compute(dilated_depth_mask)
        
        # Compute the saliency map (where to look)
        bottom_mask = horizontal_mask & depth_mask
        saliency_mask = self.saliency_estimator.compute(frame_bgr, bottom_mask)

        eroded_depth_mask = cv2.erode(depth_mask, self.erode_depth)
        mask = saliency_mask & horizontal_mask & eroded_depth_mask

        mask = cv2.dilate(mask, self.connection_kernel)
        mask = cv2.erode(mask, self.erosion_kernel)
        mask = cv2.dilate(mask, self.dilatation_kernel)

        obstacles = self.obstacle_tracker.compute(frame, mask)

        canvas = None
        if return_stategy:  # DRAW horizon information, tracking window & show layout
            canvas = source_frame.copy()

            # Scatter gradient points on the image
            for (x, y) in zip(hpx, hpy):
                cv2.circle(canvas, (int(self.downscale_factor * x), int(self.downscale_factor * y)), 3, COLOR_HP, -1)

            # Draw the horizontal line on the image
            border_x = np.array([[[0.]], [[width - 1.]]])
            ransac_points = np.concatenate((border_x, border_y), axis=2).astype(np.int32)
            cv2.polylines(canvas, [ransac_points * self.downscale_factor],
                          isClosed=False, color=COLOR_HORIZON, thickness=2)
            
            for box in obstacles:
                x0, y0, x1, y1 = np.array([*box]) * self.downscale_factor
                cv2.rectangle(canvas, (int(x0), int(y0)), (int(x1), int(y1)), COLOR_BOX, thickness=2)
            
            # # Create the depth debug image
            # debug = cv2.cvtColor(mask * 255, cv2.COLOR_GRAY2BGR)
            # debug = cv2.resize(debug, (source_width, source_height))

            # # Create the depth debug image
            # debug_depth = cv2.cvtColor(eroded_depth_mask * 255, cv2.COLOR_GRAY2BGR)
            # debug_depth = cv2.resize(debug_depth, (source_width, source_height))

            # # Create the depth debug image
            # debug_horizon = cv2.cvtColor(horizontal_mask * 255, cv2.COLOR_GRAY2BGR)
            # debug_horizon = cv2.resize(debug_horizon, (source_width, source_height))

            # # Create the depth debug image
            # debug_saliency = cv2.cvtColor(saliency_mask * 255, cv2.COLOR_GRAY2BGR)
            # debug_saliency = cv2.resize(debug_saliency, (source_width, source_height))

            # # Show pre-processes
            # imgs = (debug_depth, debug_horizon, debug_saliency)
            # layout = np.hstack(imgs)
            # cv2.imshow('Pre-process', layout)

        return obstacles, canvas
