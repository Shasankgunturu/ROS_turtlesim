#!/bin/python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

global pose
pose = Pose()

def getDistance(data):

    pose.x = data.x
    pose.y = data.y

def circle(rad):
    rospy.init_node('circle', anonymous=True)
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

    while not rospy.is_shutdown():
        velocity_msg.linear.x = rad
        velocity_msg.angular.z = 1
        velocity_publisher.publish(velocity_msg)


if __name__ == '__main__':
    try:
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, getDistance)
        rad = int(input("Enter the radius of the circle: "))
        circle(rad)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
