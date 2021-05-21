#!/usr/bin/env python

'''
REMIND: you should roslaunch some of the files before using this code.
This code can control MARS to go to a desired position.
First, you have to set rviz to make MARS realize its position in the room.
By moving MARS to the position and getting its coordinate from "rostopic echo /amcl_pose", then, setting the x, y, z, w to move to the place.  
'''

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
        print("warehouse")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 2.35550308228
        cmd.goal.target_pose.pose.position.y = -3.18725371361
        cmd.goal.target_pose.pose.orientation.z = -0.503290874917
        cmd.goal.target_pose.pose.orientation.w = 0.864117061066
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("warehouse-mid1")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 4.61152219772
        cmd.goal.target_pose.pose.position.y = -5.15782785416
        cmd.goal.target_pose.pose.orientation.z = -0.37051665948
        cmd.goal.target_pose.pose.orientation.w = 0.928825820619
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("warehouse-mid2")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 6.1186914444
        cmd.goal.target_pose.pose.position.y = -6.61669445038
        cmd.goal.target_pose.pose.orientation.z = 0.225592839728
        cmd.goal.target_pose.pose.orientation.w = 0.974221674294
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("Shelf1")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 8.91937828064
        cmd.goal.target_pose.pose.position.y = -4.61103630066
        cmd.goal.target_pose.pose.orientation.z = 0.858489360502
        cmd.goal.target_pose.pose.orientation.w = 0.512831373752
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("Shelf2")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 8.01776981354
        cmd.goal.target_pose.pose.position.y = -2.75765752792
        cmd.goal.target_pose.pose.orientation.z = 0.854279780185
        cmd.goal.target_pose.pose.orientation.w = 0.519813483057
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("Shelf3")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 7.11867427826
        cmd.goal.target_pose.pose.position.y = -1.20243740082
        cmd.goal.target_pose.pose.orientation.z = 0.858969132663
        cmd.goal.target_pose.pose.orientation.w = 0.512027371468
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("Shelf4")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 6.3447060585
        cmd.goal.target_pose.pose.position.y = 0.515312314034
        cmd.goal.target_pose.pose.orientation.z = 0.851896252711
        cmd.goal.target_pose.pose.orientation.w = 0.523710582877
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("Shelf4-mid1")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 4.20419597626
        cmd.goal.target_pose.pose.position.y = 1.77392196655
        cmd.goal.target_pose.pose.orientation.z = 0.999993987759
        cmd.goal.target_pose.pose.orientation.w = 0.00346762836293
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("Shelf4-mid2")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 1.90743637085
        cmd.goal.target_pose.pose.position.y = 1.10563850403
        cmd.goal.target_pose.pose.orientation.z = -0.624642194607
        cmd.goal.target_pose.pose.orientation.w = 0.780911088868
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("Shelf4-mid3")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 2.78451395035
        cmd.goal.target_pose.pose.position.y = -1.95036220551
        cmd.goal.target_pose.pose.orientation.z = -0.633823887486
        cmd.goal.target_pose.pose.orientation.w = 0.773477394403
        velocity_publisher.publish(cmd)



#################################################################

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
