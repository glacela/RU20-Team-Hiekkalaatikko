import numpy as np
import cv2
import imutils


ITERATIONS = 2
MIN_AREA_TO_DETECT = 1000


def blur_and_hsv(image):
    blurred_frame = cv2.GaussianBlur(image, (5, 5), 0)
    hsv_image = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
    return hsv_image


def image_to_center_points(
        orig_image,
        low_color,
        high_color,
        debug_name=False):
    hsv_image = blur_and_hsv(orig_image)
    ecore_image, ecore_mask = find_ecores_by_color(
        hsv_image, orig_image, low_color, high_color)
    ecore_coordinates = find_center_points(
        ecore_mask, MIN_AREA_TO_DETECT)

    # Show ball mask to see in detail the ball detection
    if debug_name:
        cv2.imshow(f'{debug_name}_mask', ecore_mask)
        cv2.imshow(f'{debug_name}_image', ecore_image)
        cv2.waitKey(1)

    return ecore_coordinates


def find_ecores_by_color(
        hsv_image,
        orig_image,
        low_color,
        high_color):
    """
    """
    color_mask = cv2.inRange(hsv_image, low_color, high_color)
    color_mask = cv2.erode(color_mask, None, iterations=ITERATIONS)
    color_mask = cv2.dilate(color_mask, None, iterations=ITERATIONS)
    color_image = cv2.bitwise_and(orig_image, orig_image, mask=color_mask)
    return color_image, color_mask


def find_center_points(color_mask, min_ball_area_to_detect):
    """
    """
    center_points = []
    contours = cv2.findContours(
        color_mask, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    for contour in contours:
        # Uncomment the below print to see the right
        # value for min_ball_area_to_detect
        # print("Ball area detected: {}".format(cv2.contourArea(contour)))
        if cv2.contourArea(contour) < min_ball_area_to_detect:
            continue

        # compute the center of the contour
        moments = cv2.moments(contour)
        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
        center_points.append([center_x, center_y])

    return center_points
