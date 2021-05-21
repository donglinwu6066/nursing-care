#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import numpy as np
# import matplotlib.pyplot as plt
import rospy
from rospy.client import init_node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from geometry_msgs.msg import Point
import tf
from vision.msg import Floats

DEPTH_IMAGE_TOPIC = "/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
TARGET_CLASS = 'doorknob'
CAMERA_RESOLUTION = (480, 640)


class Cam2Target(object):

    def __init__(self):

        rospy.init_node('cam2target', anonymous=True)
        # distance from objects to camera
        self.pub_doorknob = rospy.Publisher(
            "cam2target", data_class=Floats, queue_size=5)

        # publish frequency
        self.__rate = rospy.Rate(2)

        # CVand ROSImage brige
        self.__cvbridge = CvBridge()

        # original depth data
        self.__depths = None

        # set intrinsic_matrix for camera
        self.__get_intrinsic_matrix()

    def cam2target(self):
        print("Ready to generate cam2target")
        while not rospy.is_shutdown():
            self.__get_depths()
            sub_bbox = rospy.Subscriber(
                "darknet_ros/bounding_boxes", BoundingBoxes, callback=self.__get_bbox, queue_size=1)

    def __get_bbox(self, boxes_msg):
        ### depth is x axis ###
        ### left is y axis ###
        ### up is z axis ###

        # Get box info
        for bbox in boxes_msg.bounding_boxes:
            if bbox.Class == TARGET_CLASS:

                xref = (bbox.xmin + bbox.xmax) / 2
                yref = (bbox.ymin + bbox.ymax) / 2
                p = Point(xref, yref, self.__depths[yref, xref])
                # print(p)
                vector_cam = self.pixel2cam(p)

                # if depth_array is not None:
                if vector_cam[2] is not None and not 0:
                    # swap dimension to fit our camera orientation
                    temp = vector_cam[0]
                    vector_cam[0] = vector_cam[2]
                    vector_cam[2] = vector_cam[1]
                    vector_cam[1] = temp

                elif vector_cam[2] is 0:
                    vector_cam = np.array([-1.0, -1.0, -1.0])

        # publish [x, y, z, 1]
        self.pub_doorknob.publish(vector_cam)

    def __get_intrinsic_matrix(self):
        # Get camera intrinsic matrix
        try:
            info_msg = rospy.wait_for_message(
                CAMERA_INFO_TOPIC, CameraInfo, timeout=1)
            self.__intrinsic_matrix = np.array(info_msg.K).reshape((3, 3))
        except rospy.ROSException as e:
            rospy.logerr("Time out for camera info!")
            exit(-1)

    # original depth data
    def __get_depths(self):
        try:
            depth_msg = rospy.wait_for_message(
                DEPTH_IMAGE_TOPIC, Image, timeout=0.5)
            depth_image = self.__cvbridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough")
            self.__depths = np.array(depth_image, dtype=Floats) * 0.001
        except rospy.ROSInitException as e:
            rospy.logerr("Timeout for depth image")
            exit(-1)

        self.__rate.sleep()

    def set_rate(self, freq):
        self.__rate = freq

    def pixel2cam(self, p):
        # Vector in pixel coordinate s[u, v, 1]*T
        vector_pixel = np.array([p.x, p.y, 1.0]).transpose() * p.z

        # Vector in camera coordinate
        vector_cam = np.dot(np.linalg.inv(
            self.__intrinsic_matrix), vector_pixel)
        vector_cam = np.append(vector_cam, [1], 0).transpose()

        return vector_cam


if __name__ == '__main__':
    try:
        object = Cam2Target()
        object.cam2target()
    except rospy.ROSException as e:
        rospy.logerr("Time out for camera info!")
        exit(-1)
