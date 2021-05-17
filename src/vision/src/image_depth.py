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
from vision.srv import Rotation

DEPTH_IMAGE_TOPIC = "/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
TARGET_CLASS = 'doorknob'
CAMERA_RESOLUTION = (480, 640)


class Image_depth(object):

    def __init__(self):

        rospy.init_node('image_depth', anonymous=True)
        # distance from objects to camera
        self.pub_doorknob = rospy.Publisher(
            "nursing/doorknob_depth", data_class=Floats, queue_size=5)
        # rotate degree to parallel with wall
        # self.pub_rotation = rospy.Publisher(
        #     "nursing/rotation_radian", data_class=Floats, queue_size=5)
        # self.srv_rotation = rospy.Service(
        #     'rotation', service_class=Rotation, handler=self.get_rotation)

        # publish frequency
        self.rate = rospy.Rate(2)

        # CVand ROSImage brige
        self.cvbridge = CvBridge()

        # depth array
        self.depth_array = None

        # set intrinsic_matrix for camera
        self.intrinsic_matrix = None
        self.get_intrinsic_matrix()

    def get_intrinsic_matrix(self):
        # Get camera intrinsic matrix
        try:
            info_msg = rospy.wait_for_message(
                CAMERA_INFO_TOPIC, CameraInfo, timeout=1)
            self.intrinsic_matrix = np.array(info_msg.K).reshape((3, 3))
            # print(intrinsic_matrix)
        except rospy.ROSException as e:
            rospy.logerr("Time out for camera info!")
            exit(-1)

    def depth(self):
        try:
            depth_msg = rospy.wait_for_message(
                DEPTH_IMAGE_TOPIC, Image, timeout=0.5)
            depth_image = self.cvbridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough")
            self.depth_array = np.array(depth_image, dtype=Floats) * 0.001
        except rospy.ROSInitException as e:
            rospy.logerr("Timeout for depth image")
            exit(-1)

    def get_bbox(self, boxes_msg):
        ### depth is x axis ###
        ### left is y axis ###
        ### up is z axis ###

        # Get box info
        for bbox in boxes_msg.bounding_boxes:
            if bbox.Class == TARGET_CLASS:
                xmid = (bbox.xmin + bbox.xmax) / 2
                ymid = (bbox.ymin + bbox.ymax) / 2

                # if depth_array is not None:
                if self.depth_array[ymid, xmid] is not None:
                    print("depth of box center: {:.2f}mm".format(
                        self.depth_array[ymid, xmid]))
                    p = Point(xmid, ymid, self.depth_array[ymid, xmid])
                #  if depth_array is None, find its neighbors:
                else:
                    for i in range(-2, 2):
                        for j in range(-2, 2):
                            # originally, self.depth_array[ymid+i, xmid+j]
                            if self.depth_array[ymid+i, xmid+j] is not None:
                                p = Point(ymid+i, xmid+j,
                                          self.depth_array[ymid+i, xmid+j])

                                # double loop breaker
                                i = j = 2

        # get horizontal_y and vertical_z from 2D
        vector_cam = self.pixel2cam(p)

        # swap dimension to fit our camera orientation,
        horizontal_y = vector_cam[0]
        vertical_z = vector_cam[1]
        vector_cam[0] = self.depth_array[ymid, xmid]
        vector_cam[1] = horizontal_y
        vector_cam[2] = vertical_z

        # publish [x, y, z, 1]
        self.pub_doorknob.publish(vector_cam)

    def get_rotation(self, req):

        print("UNDER CONSTRUCTION")
        # get left and right point on image
        left = self.depth_array[CAMERA_RESOLUTION[0]/2, CAMERA_RESOLUTION[0]/3]
        right = self.depth_array[CAMERA_RESOLUTION[0] /
                                 2, CAMERA_RESOLUTION[0]*2/3]

        if (left - right) > 0.1:
            rotate = "counterclockwise"
        elif (left - right) > 0.1:
            rotate = "clockwise"
        else:
            rotate = "balanced"

        return vision.srv.RotationResponse(rotate)

    def pixel2cam(self, p):
        # Vector in pixel coordinate s[u, v, 1]*T
        vector_pixel = np.array([p.x, p.y, 1.0]).transpose() * p.z

        # Vector in camera coordinate
        vector_cam = np.dot(np.linalg.inv(self.intrinsic_matrix), vector_pixel)
        vector_cam = np.append(vector_cam, [1], 0).transpose()
        return vector_cam

    def get_depth_array(self):
        self.depth()

        center_idx = np.array(self.depth_array.shape) / 2
        if np.isnan(self.depth_array[center_idx[0], center_idx[1]]):
            rospy.logerr("depth_array is nan")
            # continue

        # Vector in pixel coordinate s[u, v, 1]*T
        vector_pixel = np.array([center_idx[1], center_idx[0], 1.0]).transpose(
        ) * self.depth_array[center_idx[0], center_idx[1]]
        # Vector in camera coordinate
        vector_cam = np.dot(np.linalg.inv(self.intrinsic_matrix), vector_pixel)
        vector_cam = np.append(vector_cam, [1], 0).transpose()
        # rospy.loginfo(vector_cam)

        # rospy.loginfo(self.depth_array)
        # self.pub_doorknob.publish(self.depth_array)
        self.rate.sleep()
        # print(vector_cam)

    def image_depth(self):
        while not rospy.is_shutdown():
            self.get_depth_array()

            # self.get_rotation()
            # sub_tf = rospy.Subscriber(
            # "nursing/mid_depth", data_class=Floats, callback=self.get_depth_array())
            sub_bbox = rospy.Subscriber(
                "darknet_ros/bounding_boxes", BoundingBoxes, callback=self.get_bbox, queue_size=1)


if __name__ == '__main__':
    try:
        image = Image_depth()
        image.image_depth()
    except rospy.ROSException as e:
        rospy.logerr("Time out for camera info!")
        exit(-1)
