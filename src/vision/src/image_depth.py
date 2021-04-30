#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import numpy as np
# import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import tf

DEPTH_IMAGE_TOPIC = "/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
# CAMERA_RESOLUTION = (480, 640)


def image_depth():
    # CVand ROSImage brige
    cvbridge = CvBridge()

    # Create a panal
    # fig = plt.figure(figsize=(8,6))
    # colorbar = None

    # Get camera intrinsic matrix
    intrinsic_matrix = None

    try:
        info_msg = rospy.wait_for_message(
            CAMERA_INFO_TOPIC, CameraInfo, timeout=0.5)
        intrinsic_matrix = np.array(info_msg.K).reshape((3, 3))
        # print(intrinsic_matrix)
    except rospy.ROSException as e:
        rospy.logerr("Time out for camera info!")
        exit(-1)

    # rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        try:
            depth_msg = rospy.wait_for_message(
                DEPTH_IMAGE_TOPIC, Image, timeout=0.5)
            depth_image = cvbridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32) * 0.001

            center_idx = np.array(depth_array.shape) / 2
            if np.isnan(depth_array[center_idx[0], center_idx[1]]):
                rospy.logerr("depth_array is nan")
                continue

            # Vector in pixel coordinate s[u, v, 1]*T
            vector_pixel = np.array([center_idx[1], center_idx[0], 1.0]
                                    ).transpose() * depth_array[center_idx[0], center_idx[1]]
            # Vector in camera coordinate
            vector_cam = np.dot(np.linalg.inv(intrinsic_matrix), vector_pixel)
            vector_cam = np.append(vector_cam, [1], 0).transpose()

            return vector_cam
            # print(vector_cam)

            # ax = plt.imshow(depth_array)
            # if colorbar is not None:
            #     colorbar.remove()
            #     colorbar = fig.colorbar(ax)
            # else:
            #     colorbar = fig.colorbar(ax)
            # plt.pause(0.01)

        except rospy.ROSInitException as e:
            rospy.logerr("Timeout for depth image")
            exit(-1)
    # rate.sleep()


if __name__ == '__main__':
    rospy.init_node('depth_image_node', anonymous=False)

    try:
        vector_cam = image_depth()
        print(vector_cam)
    except rospy.ROSException as e:
        rospy.logerr("Time out for camera info!")
        exit(-1)
