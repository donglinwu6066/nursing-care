#!/usr/bin/env python
import cv_bridge
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
import numpy as np


TARGET_CLASS = 'doorknob'
cvbridge = None


def box_cb(boxes_msg):
    # Get depth image
    depth_array = None
    flag = False
    try:
        depth_msg = rospy.wait_for_message(
            '/camera/aligned_depth_to_color/image_raw', Image, timeout=0.5)
        depth_image = cvbridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough")
        # Convert the depth image to a Numpy array
        depth_array = np.array(depth_image, dtype=np.float32)
    except rospy.ROSException as e:
        rospy.logerr("Timeout for depth image!")
    except CvBridgeError, e:
        rospy.logerr(e)

    # Get box info
    for bbox in boxes_msg.bounding_boxes:
        if bbox.Class == TARGET_CLASS:
            xmid = (bbox.xmin + bbox.xmax) / 2
            ymid = (bbox.ymin + bbox.ymax) / 2

            # if depth_array is not None:
            if depth_array[ymid, xmid] is not None:
                print("depth of box center: {:.2f}mm".format(
                    depth_array[ymid, xmid]))
            else:
                for i in range(-2, 2):
                    for j in range(-2, 2):
                        if depth_array[ymid+i, xmid+j] is not None:
                            print("depth of box center: {:.2f}mm".format(
                                depth_array[ymid+i, xmid+j]))
                            flag = True
                            break
                    if flag:
                        break


if __name__ == '__main__':
    rospy.init_node('get_box_depth_node', anonymous=False)
    rospy.Subscriber("darknet_ros/bounding_boxes",
                     BoundingBoxes, callback=box_cb, queue_size=1)

    cvbridge = CvBridge()

    rospy.spin()