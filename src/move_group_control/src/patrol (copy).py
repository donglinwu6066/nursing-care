#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseActionGoal


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
        print("outside button")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 0.699
        cmd.goal.target_pose.pose.position.y = -0.217
        cmd.goal.target_pose.pose.orientation.z = -0.338
        cmd.goal.target_pose.pose.orientation.w = 0.941
        velocity_publisher.publish(cmd)

	rospy.sleep(2.)
        raw_input("Press Enter to continue...")
        print("outside door") 
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 1.075
        cmd.goal.target_pose.pose.position.y = 0.199
        cmd.goal.target_pose.pose.orientation.z = -0.368
        cmd.goal.target_pose.pose.orientation.w = 0.930
        velocity_publisher.publish(cmd)


    



#################################################################

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
