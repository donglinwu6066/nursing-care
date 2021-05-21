#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import numpy as np
import rospy
import math
from rospy.client import init_node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from vision.srv import Rotation, RotationResponse
from vision.msg import Floats
import logging
logging.basicConfig()
DEPTH_IMAGE_TOPIC = "/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"

CAMERA_RESOLUTION = (480, 640)

intrinsic_matrix = None
cvbridge = CvBridge()
depth_array = None


def rotation_server():

    rospy.init_node('rotation_server')
    get_intrinsic_matrix()
    srv = rospy.Service('rotation', Rotation, handle_rotation)
    print("Ready to generate rotation.")
    rospy.spin()

def get_angle(a, b):
    c = math.sqrt(a*a+b*b - math.sqrt(3)*a*b)
    # print("c = ", c)
    theta = (a*a+c*c-b*b) / (2*a*c)
    # print("theta = ", theta)
    arccos = math.acos(theta) / math.pi * 180
    # print(arccos)
    delta = 90 - arccos
    # print(delta)
    return delta

def handle_rotation(req):
    # if req.request is 1:
    print("UNDER CONSTRUCTION OF ANGLE")
    depth()

    # get left and right point on image
    # left = pixel2cam(CAMERA_RESOLUTION[0]/2, CAMERA_RESOLUTION[0]/3)
    # right = pixel2cam(CAMERA_RESOLUTION[0]/2, CAMERA_RESOLUTION[0]*2/3)
    left = depth_array[CAMERA_RESOLUTION[0]/2, CAMERA_RESOLUTION[0]/3]
    right = depth_array[CAMERA_RESOLUTION[0]/2, CAMERA_RESOLUTION[0]*2/3]
    print(left)
    print(right)
    print(left - right)
    angle = get_angle(left, right)
    if (angle > 18):
        rotate = str(angle - 15)
    elif (angle < 12):
        rotate = str(angle - 15)
    elif (angle < 18 and angle > 15.5):
        rotate = "1"
    elif (angle < 14.5 and angle > 12):
        rotate = "-1"
    else:
        rotate = "0"
    return RotationResponse(rotate)


def get_intrinsic_matrix():
    global intrinsic_matrix
    # Get camera intrinsic matrix
    try:
        print("hi")
        info_msg = rospy.wait_for_message(
            CAMERA_INFO_TOPIC, CameraInfo, timeout=1)
        intrinsic_matrix = np.array(info_msg.K).reshape((3, 3))
    except rospy.ROSException as e:
        rospy.logerr("Time out for camera info!")
        exit(-1)


def depth():
    global depth_array
    try:
        depth_msg = rospy.wait_for_message(
            DEPTH_IMAGE_TOPIC, Image, timeout=0.5)
        depth_image = cvbridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=Floats) * 0.001
    except rospy.ROSInitException as e:
        rospy.logerr("Timeout for depth image")
        exit(-1)


def pixel2cam(x, y):
    # get depth array
    depth()

    if np.isnan(depth_array[x, y]):
        rospy.logerr("depth_array is nan")
        # continue

    # Vector in pixel coordinate s[u, v, 1]*T
    vector_pixel = np.array([y, x, 1.0]).transpose(
    ) * depth_array[x, y]
    # Vector in camera coordinate
    vector_cam = np.dot(np.linalg.inv(intrinsic_matrix), vector_pixel)
    vector_cam = np.append(vector_cam, [1], 0).transpose()
    print(vector_cam)
    return vector_cam[2]


if __name__ == '__main__':

    rotation_server()
