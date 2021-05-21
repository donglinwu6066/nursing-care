#!/usr/bin/env python

import sys
import rospy
from vision.srv import *

def rotaion_client(x):
    print("paper ring")
    rospy.wait_for_service('rotation')
    try:
        rotation = rospy.ServiceProxy('rotation', Rotation)
        resp1 = rotation(x)
        return resp1.rotate
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# def usage():
#     return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = sys.argv[1]
        print(x)
        # y = int(sys.argv[2])
    else:
        # print(usage())
        sys.exit(1)
    # print("Requesting %s+%s"%(x, y))
    # print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
    starlight = rotaion_client(x)
    print(starlight)