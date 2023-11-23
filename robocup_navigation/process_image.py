#!/usr/bin/env python3

import cv2
import numpy as np

MARKER_LENGTH = 0.13
camMat = np.matrix([[528.43375656, 0., 320.5],[0., 528.43375656, 240.5],[0., 0., 1.]])
distCoef = np.array([0., 0., 0., 0., 0. ])

def find_marker(image):
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    arucoParams = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image,arucoDict,parameters=arucoParams)
    if np.all(ids is not None):
        print(ids)
        image_marker = draw_marker(image, corners, ids)
        image_axis = marker_pos(image_marker,corners)
        return image_axis
    else:
        return 0

def draw_marker(image, corners, ids):
    colour = (0,255,0)
    return cv2.aruco.drawDetectedMarkers(image, corners, ids, colour)

def grayscale_image(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def marker_pos(image,corners):
    rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, camMat, distCoef)
    print(tvecs)
    image_axis = cv2.drawFrameAxes(image, camMat, distCoef, rvecs, tvecs, 0.1, 3)
    return image_axis, tvecs

def wait_on_gui():
    cv2.waitKey(2)