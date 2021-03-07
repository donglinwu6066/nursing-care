#!/usr/bin/env python

'''
This file can open doorknob
'''

import rospy

def opendoor():
    print("open~")

if __name__ == '__main__':
    try:
        # Testing our function
        print("opendoor")
	opendoor();
    except rospy.ROSInterruptException: pass
