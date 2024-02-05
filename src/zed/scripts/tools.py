#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.ndimage import sobel
from sklearn.linear_model import RANSACRegressor

EPSILON = 1e-5

# Define colors in BGR
COLOR_OF = (255, 0, 0)  # BLUE
COLOR_HP = (0, 0, 255)  # RED
COLOR_HORIZON = (0, 255, 0)  # GREEN
COLOR_WINDOW = (255, 0, 255)  # PURPLE


def take_top_n(indices, n_top, top_distance=5):
    # top_distance corresponds to the minimum distance in pixels between 2 tops

    n_horizontal, n_vertical = indices.shape
    top_n = np.full((n_top, n_vertical), -top_distance)

    for x in range(n_vertical):
        k_top = 0
        y = n_horizontal - 1  # start to the end (with the largest gradients)
        while k_top < n_top and y >= 0:
            top = indices[y, x]
            if np.min(np.abs(top - top_n[:, x])) > top_distance:
                top_n[k_top, x] = top
                k_top += 1
            y -= 1

    return top_n


class HorizonEstimator:
    def __init__(self, n_vertical=8, n_top=3, max_residual=4., max_slope_change=6e-2, max_bias_change=10.):
        self.n_vertical = n_vertical
        self.n_top = n_top

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

        # Normalize between 0 and 1
        image = image / 255.

        # Compute some useful values
        height, width, *_ = image.shape
        horizontal_half_step = int(0.5 * width / self.n_vertical)

        # Compute the horizontal gradient -> higher = from dark to bright -> Guerledan hypothesis
        # horizontal_grad = sobel(image, axis=0)
        horizontal_grad = np.abs(sobel(image, axis=0))

        # Filter top n_top largest gradient points
        indices = np.argsort(horizontal_grad[:, horizontal_half_step:][:, ::2 * horizontal_half_step], axis=0)
        x = horizontal_half_step + (np.arange(self.n_vertical * self.n_top) // self.n_top) * (2 * horizontal_half_step)
        y = np.ravel(take_top_n(indices, self.n_top), order='F')

        # Reshape for visualization purposes
        horizontal_point_x = x.reshape(-1, 1)
        horizontal_point_y = y.reshape(-1, 1)

        # Compute the best line fitting our point using RANSAC
        ransac = RANSACRegressor(residual_threshold=self.max_residual, is_data_valid=is_data_valid)

        try:
            ransac.fit(horizontal_point_x, horizontal_point_y)
        except ValueError:
            print('RANSAC value error!')
            self.slope = 0.
            self.bias = None
            return self.compute(image)
        else:
            # Update slope
            ransac_pred_y = ransac.predict(np.array([[0.], [width - 1.]]))
            self.bias, y_w = ransac_pred_y.flatten()
            self.slope = (y_w - self.bias) / width

            # Compute horizon mask
            mask = (np.arange(height).reshape(-1, 1) > ransac.predict(np.arange(width).reshape(-1, 1)).T)

            return horizontal_point_x, horizontal_point_y, ransac_pred_y.reshape(-1, 1, 1), mask.astype(np.uint8)


class SaliencyEstimator:
    def __init__(self, n_sigma=3.5):
        self.n_sigma = n_sigma

    def compute(self, img_bgr, mask):
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        mask = mask.astype(bool)

        mean_hsv = np.mean(img_hsv[mask], axis=0).reshape((1, 1, 3))
        std_hsv = np.std(img_hsv[mask], axis=0).reshape((1, 1, 3))

        saliency = np.linalg.norm((img_hsv - mean_hsv) / std_hsv, axis=2)
        saliency_mask = (saliency > self.n_sigma).astype(np.uint8)

        return saliency_mask


class Buffer:
    def __init__(self, n_buffer, shape):
        self.k_buffer = 0
        self.n_buffer = n_buffer
        self.is_buffer_full = False
        self.memory = np.empty(shape=(n_buffer, *shape))

    def push(self, values):
        n_values = values.shape[0]

        if n_values > self.n_buffer:
            n_values = self.n_buffer
            values = values[:n_values]

        if self.k_buffer + n_values < self.n_buffer:
            self.memory[self.k_buffer:self.k_buffer + n_values] = values
        else:
            delta = self.n_buffer - self.k_buffer
            self.memory[self.k_buffer:self.n_buffer] = values[:delta]
            self.memory[:n_values-delta] = values[delta:]

        self.k_buffer += n_values
        if self.k_buffer >= self.n_buffer:
            self.is_buffer_full = True
            self.k_buffer -= self.n_buffer

        if self.is_buffer_full:
            return self.memory
        else:
            return self.memory[:self.k_buffer]


class TrackingWindow:
    def __init__(self, height, width, alpha=0.5, size_scale=3., size_search_factor=1.25):
        self.alpha = alpha
        self.size_scale = size_scale
        self.size_search_factor = size_search_factor

        self.size_min = np.array([width, height]).reshape((1, 1, 2)) / 64.

        self.center = np.array([width, height]).reshape((1, 1, 2)) / 2.
        self.size = np.array([width, height]).reshape((1, 1, 2)) / 2.

        self.buffer = Buffer(n_buffer=(width // 6), shape=(1, 2))

    def update(self, ps, output_scale=1.):
        if ps is not None:
            mask = (np.abs(ps - self.center) < self.size)
            mask = mask[:, 0, 0] & mask[:, 0, 1]

            points_in_window = ps[mask]
            if points_in_window.size > 0:
                self.center = (1. - self.alpha) * self.center + self.alpha * np.mean(points_in_window, axis=0)

                buffered_ps = self.buffer.push(points_in_window)
                buffer_size = np.std(buffered_ps, axis=0) / 2.  # convert into radius
                self.size = self.size_min + buffer_size * self.size_scale
            else:
                self.size *= self.size_search_factor

        return *(self.center * output_scale).flatten(), *(self.size * output_scale).flatten()


class VideoProcessor:
    def __init__(self):
        # Define the downscale factor
        self.downscale_factor = 2

        self.horizon_estimator = HorizonEstimator()
        self.saliency_estimator = SaliencyEstimator()
        self.tracking_window = None  # need image dimensions to be initialized

        # Erosion & dilatation are applied on active pixel -> used for computing the saliency mask
        self.connection_kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3, 3))
        self.erosion_kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(5, 5))
        self.dilatation_kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(5, 5))

        # Parameters for corner detection
        self.features_params = {
            'maxCorners': 10000,
            'qualityLevel': 0.01,
            'minDistance': 0.1
        }

        # Parameters for pyramidal Lucas-Kanade optical flow
        self.lk_params = {
            'winSize': (15, 15),
            'maxLevel': 2,
            'criteria': (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        }

        # Define variables to initialize later
        self.old_frame = None
        self.points_old_frame = []

    def process(self, source_frame, draw_strategy=False):
        # Get image dimensions
        source_height, source_width, *_ = source_frame.shape
        height = source_height // self.downscale_factor
        width = source_width // self.downscale_factor

        # Initialize the tracking_window if necessary
        if self.tracking_window is None:
            self.tracking_window = TrackingWindow(height, width)

        # Downscale and convert to grayscale
        frame_bgr = cv2.resize(source_frame, (width, height))
        frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        # Draw points and line on a separate image (for debugging only)
        canvas = source_frame.copy()

        # Estimate a line for the horizon
        hpx, hpy, border_y, horizontal_mask = self.horizon_estimator.compute(frame)

        if draw_strategy:  # DRAW gradients points and horizon
            # Scatter gradient points on the image
            for (x, y) in zip(hpx, hpy):
                cv2.circle(canvas, (int(self.downscale_factor * x), int(self.downscale_factor * y)), 3, COLOR_HP, -1)
            # Draw the horizontal line on the image
            border_x = np.array([[[0.]], [[width - 1.]]])
            ransac_points = np.concatenate((border_x, border_y), axis=2).astype(np.int32)
            cv2.polylines(canvas, [ransac_points * self.downscale_factor],
                          isClosed=False, color=COLOR_HORIZON, thickness=2)

        if (self.old_frame is not None) and (self.points_old_frame is not None):
            points_new_frame, status, err = cv2.calcOpticalFlowPyrLK(self.old_frame, frame,
                                                                     self.points_old_frame,
                                                                     None, **self.lk_params)

            if draw_strategy:  # DRAW optical flow
                for p0, p1 in zip(self.points_old_frame[status == 1], points_new_frame[status == 1]):
                    x0, y0 = (self.downscale_factor * p0).ravel()
                    x1, y1 = (self.downscale_factor * p1).ravel()
                    cv2.line(canvas, (int(x1), int(y1)), (int(x0), int(y0)), COLOR_OF, 3)

        # Current frame becomes the old one
        self.old_frame = frame.copy()

        # Compute the saliency map (where to look)
        saliency_mask = self.saliency_estimator.compute(frame_bgr, horizontal_mask)

        # Compute good feature point to keep track of
        mask = horizontal_mask & saliency_mask

        mask = cv2.dilate(mask, self.connection_kernel)
        mask = cv2.erode(mask, self.erosion_kernel)
        mask = cv2.dilate(mask, self.dilatation_kernel)

        self.points_old_frame = cv2.goodFeaturesToTrack(frame, mask=mask, **self.features_params)

        wx, wy, wsx, wsy = self.tracking_window.update(self.points_old_frame, self.downscale_factor)

        x0 = max(0, int(wx - wsx))
        x1 = min(int(wx + wsx), source_width)
        y0 = max(0, int(wy - wsy))
        y1 = min(int(wy + wsy), source_height)

        if draw_strategy:  # DRAW tracking window and show layout
            cv2.rectangle(canvas, (x0, y0), (x1, y1), COLOR_WINDOW, thickness=2)

            debug = cv2.cvtColor(mask * 255, cv2.COLOR_GRAY2BGR)
            debug = cv2.resize(debug, (source_width, source_height))

            # Stack images vertically or horizontally
            layout = np.vstack((canvas, debug)) if height < width else np.hstack((canvas, debug))

            # Show lower part of layout
            cv2.imshow('Canvas & Debug', layout)
            cv2.waitKey(1)

        return x0, x1, y0, y1
