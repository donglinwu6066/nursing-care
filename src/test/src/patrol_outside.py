#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseActionGoal

#outside 632 for open door

def move():

    # Starts a new node
    rospy.init_node('plat_move', anonymous=True)
    velocity_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    cmd = MoveBaseActionGoal()
    #rate = rospy.Rate(0.0636) # 0.1hz

    #Receiveing the user's input
    print("Let's move your marslite lol")

    while not rospy.is_shutdown():

        rospy.sleep(2.)
        raw_input("Press Enter to continue...")
        print("position_1")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 0.685518325424
        cmd.goal.target_pose.pose.position.y = -1.6039554297
        cmd.goal.target_pose.pose.orientation.z = 0.599524672035
        cmd.goal.target_pose.pose.orientation.w = 0.80035627543
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("position_2")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 1.2257013637
        cmd.goal.target_pose.pose.position.y = -0.443885547
        cmd.goal.target_pose.pose.orientation.z = 0.996684258638
        cmd.goal.target_pose.pose.orientation.w = 0.0813663848464
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("position_3")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 0.122825026602
        cmd.goal.target_pose.pose.position.y = -0.258469543458
        cmd.goal.target_pose.pose.orientation.z = 0.998810284783
        cmd.goal.target_pose.pose.orientation.w = 0.0487648952853
        velocity_publisher.publish(cmd)

    #    print("opening_1")
    #    cmd.goal.target_pose.header.frame_id = 'map'
    #    cmd.goal.target_pose.pose.position.x = 1.11925713146
    #    cmd.goal.target_pose.pose.position.y = -0.386072942585
    #    cmd.goal.target_pose.pose.orientation.z = 0.993983685072
    #    cmd.goal.target_pose.pose.orientation.w = 0.109528232936
    #    velocity_publisher.publish(cmd)
    #    raw_input("Press Enter to continue...")

#################################################################

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
