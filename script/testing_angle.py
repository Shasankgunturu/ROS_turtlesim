#!/home/shasank/miniconda3/bin/python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

global pose
pose = Pose()

def getDistance(data):

    pose.x = data.x
    pose.y = data.y

def heart():

    rospy.init_node('heart', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)

    init_x = pose.x
    init_y = pose.y

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    velocity_msg.linear.z = 0
    velocity_msg.angular.x = 0
    velocity_msg.angular.y = 0
    velocity_msg.angular.z = 0

    goal_location_x = 3 + pose.x
    goal_location_y = 3 + pose.y

    kp = 0.1
    error_last = 0
    i_max = 1
    i_min = -1
    kd = 1
    ki = 0
    Integrator=0

    while not rospy.is_shutdown():
        velocity_msg.linear.y = 1
        velocity_msg.angular.z = 1  
        velocity_publisher.publish(velocity_msg)

if __name__ == '__main__':
    try:
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, getDistance)
        # side = int(input("Enter the side of the square: "))
        heart()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")