#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import rospy
import tf

PARENT_LINK = "gripper_base"
TARGET_LINK = "base_link"


def realsense_tf():
    tflistener = tf.TransformListener()
    # create tf listener
    tflistener.waitForTransform(
        PARENT_LINK, TARGET_LINK, rospy.Time(0), rospy.Duration(1.0))
    (translation, rotation_q) = tflistener.lookupTransform(
        PARENT_LINK, TARGET_LINK, rospy.Time(0))
    # print("translation:", translation)
    # print("rotation_q:", rotation_q)

    translation = np.diag(translation)
    translation = np.append(translation, [[0, 0, 0]], 0)
    rotation_q = np.transpose(rotation_q)
    tf_cam2base = np.column_stack((translation, rotation_q))

    #translation = np.append(translation, rotation_q[0], 1)
    # translation = np.append(translation, [[rotation_q.index(
    #    1)], [rotation_q.index(2)], [rotation_q.index(3)]], 1)

    return tf_cam2base


if __name__ == '__main__':
    rospy.init_node('tf_example_node', anonymous=False)

    try:
        tf_cam2base = realsense_tf()
        print(tf_cam2base)
    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("tf error!")
