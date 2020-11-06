"""
Example script to detect objects by color from an image using OpenCV.
"""
import numpy as np
import cv2
from utils.ecore_utils import image_to_center_points
from utils.select_video_source import select_video_source


# Select the camera source by setting this
#VIDEO_SOURCE = "ffmpeg"  # Options: 'gstreamer', 'webcam' or 'opencv'
#VIDEO_SOURCE = 'webcam'
VIDEO_SOURCE = 'ffmpeg'

# Low and High values in HSV-colorspace for detecting color range.
# See the these articles to understand more about HSV-colorspace:
# https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
# https://en.wikipedia.org/wiki/HSL_and_HSV
POS_ECORE_LOW_COLOR = np.array([120, 80, 100], dtype=np.float32)
POS_ECORE_HIGH_COLOR = np.array([175, 255, 255], dtype=np.float32)
NEG_ECORE_LOW_COLOR = np.array([25, 80, 100], dtype=np.float32)
NEG_ECORE_HIGH_COLOR = np.array([40, 255, 255], dtype=np.float32)


def print_core_positions(pos_ecore_positions, neg_ecore_positions):
    """
    Function to pretty print the X and Y coordinates for energy cores
    """
    if pos_ecore_positions:
        for i, core in enumerate(pos_ecore_positions):
            print(f'Positive Core {i}: X: {core[0]:.2f}, Y: {core[1]:.2f}')
    if neg_ecore_positions:
        for i, core in enumerate(neg_ecore_positions):
            print(f'Negative Core {i}: X: {core[0]:.2f}, Y: {core[1]:.2f}')
    if not pos_ecore_positions and not neg_ecore_positions:
        print('No Energy Cores detected')
    print('=== Done\n')


def main():
    """
    Get an image from the chosen video source and then detect the energy
    cores from the image. Finally print the coordinates of the found
    energy cores.
    """
    get_image_func = select_video_source(VIDEO_SOURCE)

    while True:
        frame = get_image_func()
        if frame is None:
            continue

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        pos_ecore_positions = image_to_center_points(
            frame,
            POS_ECORE_LOW_COLOR,
            POS_ECORE_HIGH_COLOR,
            'Positive Energy Cores')
        neg_ecore_positions = image_to_center_points(
            frame,
            NEG_ECORE_LOW_COLOR,
            NEG_ECORE_HIGH_COLOR,
            'Negative Energy Cores')
        print_core_positions(pos_ecore_positions, neg_ecore_positions)


if __name__ == '__main__':
    main()
