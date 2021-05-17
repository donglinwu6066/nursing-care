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
        print("position_1")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 5.76386527087
        cmd.goal.target_pose.pose.position.y = 0.52827135555
        cmd.goal.target_pose.pose.orientation.z = 0.780875071381
        cmd.goal.target_pose.pose.orientation.w = 0.624687220051
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("position_2")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 6.35159115396
        cmd.goal.target_pose.pose.position.y = 1.4817120434
        cmd.goal.target_pose.pose.orientation.z = 0.352876129763
        cmd.goal.target_pose.pose.orientation.w = 0.935670047102
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("position_3")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 6.80329398193
        cmd.goal.target_pose.pose.position.y = 2.03871081275
        cmd.goal.target_pose.pose.orientation.z = 0.627085371245
        cmd.goal.target_pose.pose.orientation.w = 0.778950535767
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("position_4")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 6.96393890019
        cmd.goal.target_pose.pose.position.y = 2.41214431309
        cmd.goal.target_pose.pose.orientation.z = 0.678204516696
        cmd.goal.target_pose.pose.orientation.w = 0.734873209155
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

        print("position_5")
        cmd.goal.target_pose.header.frame_id = 'map'
        cmd.goal.target_pose.pose.position.x = 6.90064350091
        cmd.goal.target_pose.pose.position.y = 3.24971623032
        cmd.goal.target_pose.pose.orientation.z = 0.470270082984
        cmd.goal.target_pose.pose.orientation.w = 0.882522548749
        velocity_publisher.publish(cmd)
        raw_input("Press Enter to continue...")

#################################################################

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
