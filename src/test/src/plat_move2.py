#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def rotate(x):
    #Starts a new node
    rospy.init_node('plat_move', anonymous=True)
    velocity_publisher = rospy.Publisher('/mob_plat/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    print("Let's rotate your robot")
    speed = 15 #input("Input your speed (degrees/sec):")
    angle = abs(x) #input("Type your distance (degrees):")
    print("angle = ", angle)
    if(x >= 0):
        print("CLOCKWISE")
        clockwise = True #input("Clockwise?: ") #True or false
    else:
        print("COUNTERCLOCKWISE")
        clockwise = False

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #We wont use linear components
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)


    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    # rospy.spin()

def move():
    # Starts a new node
    rospy.init_node('plat_move', anonymous=True)
    velocity_publisher = rospy.Publisher('/mob_plat/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    #Receiveing the user's input
    print("Let's move your marslite lol")
    speed = 0.1 # input("Input your speed:")
    distance = 0.25#input("Type your distance:")
    isForward = True#input("Foward?: ")#True or False

    #Checking if the movement is forward or backwards
    if(isForward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    #Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    #Loop to move the turtle in an specified distance
    while(current_distance < distance):
        #Publish the velocity
        velocity_publisher.publish(vel_msg)
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance= speed*(t1-t0)
    #After the loop, stops the robot
    vel_msg.linear.x = 0
    #Force the robot to stop
    velocity_publisher.publish( vel_msg)
    # rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        rot = input("Rotate or not?")
        if(rot):
            rotate()
        else: 
            move()
    except rospy.ROSInterruptException: pass
