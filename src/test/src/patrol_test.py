#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseActionGoal


def move(x, y, z, w):

    # Starts a new node
    #rospy.init_node('plat_move', anonymous=True)
    velocity_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    cmd = MoveBaseActionGoal()
    #rate = rospy.Rate(0.0636) # 0.1hz

    #Receiveing the user's input
    print("Let's move your marslite lol")

#    while not rospy.is_shutdown():

    rospy.sleep(1.)
    raw_input("Press Enter to continue...")
    cmd.goal.target_pose.header.frame_id = 'map'
    cmd.goal.target_pose.pose.position.x = x
    cmd.goal.target_pose.pose.position.y = y
    cmd.goal.target_pose.pose.orientation.z = z
    cmd.goal.target_pose.pose.orientation.w = w
    velocity_publisher.publish(cmd)
    #raw_input("Press Enter to continue...")


       


#################################################################

if __name__ == '__main__':
    #try:
        #Testing our function

        print("warehouse")
        #move(2.35550308228, -3.18725371361, -0.503290874917, 0.864117061066)
        print("warehouse-mid1")
        move(4.61152219772, -5.15782785416, -0.37051665948, 0.928825820619)
    #except rospy.ROSInterruptException: pass
