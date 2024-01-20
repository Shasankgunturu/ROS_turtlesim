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

def square(side):

    rospy.init_node('square', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
    # rospy.Rate()
    init_x = pose.x
    init_y = pose.y

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    velocity_msg.linear.z = 0
    velocity_msg.angular.x = 0
    velocity_msg.angular.y = 0
    velocity_msg.angular.z = 0

    goal_location_x = side/2 + pose.x
    goal_location_y = pose.y

    # print(goal_location_x, goal_location_y)
    kp = 1
    error_last = 0
    i_max = 1
    i_min = 0
    kd = 0
    ki = 0
    Integrator=0
    while not rospy.is_shutdown():

        while pose.x < goal_location_x:
            error = math.sqrt(pow(goal_location_x-pose.x, 2)+pow(goal_location_y-pose.y, 2))
            p_value = kp*error
            d_value = kd*(error-error_last)
            Integrator = Integrator + error
            if Integrator > i_max:
                Integrator = i_max
            elif Integrator < i_min:
                Integrator = i_min
            i_value = Integrator*ki
            error_last = error
            velocity_msg.linear.x = p_value + i_value + d_value
            velocity_publisher.publish(velocity_msg)

        velocity_msg.linear.x = 0
        velocity_publisher.publish(velocity_msg)
        distance_moved1 = math.sqrt(pow(init_x-pose.x, 2)+pow(init_y-pose.y, 2))
        print(distance_moved1)
        init_x = pose.x
        init_y = pose.y
        goal_location_x = pose.x
        goal_location_y = pose.y + side
        error_last = 0
        Integrator=0

        while pose.y < goal_location_y:
            error = math.sqrt(pow(goal_location_x-pose.x, 2)+pow(goal_location_y-pose.y, 2))
            p_value = kp*error
            d_value = kd*(error-error_last)
            Integrator = Integrator + error
            if Integrator > i_max:
                Integrator = i_max
            elif Integrator < i_min:
                Integrator = i_min
            i_value = Integrator*ki
            error_last = error
            velocity_msg.linear.y = p_value + i_value + d_value
            velocity_publisher.publish(velocity_msg)

        velocity_msg.linear.y = 0
        velocity_publisher.publish(velocity_msg)
        distance_moved2 = math.sqrt(pow(init_x-pose.x, 2)+pow(init_y-pose.y, 2))
        print(distance_moved2)
        init_x = pose.x
        init_y = pose.y
        goal_location_x = pose.x - side
        goal_location_y = pose.y
        error_last = 0
        Integrator=0
        
        while pose.x > goal_location_x:
            error = math.sqrt(pow(goal_location_x-pose.x, 2)+pow(goal_location_y-pose.y, 2))
            p_value = kp*error
            d_value = kd*(error-error_last)
            Integrator = Integrator + error
            if Integrator > i_max:
                Integrator = i_max
            elif Integrator < i_min:
                Integrator = i_min
            i_value = Integrator*ki
            error_last = error
            velocity_msg.linear.x = -(p_value + i_value + d_value)
            velocity_publisher.publish(velocity_msg)

        velocity_msg.linear.x = 0
        velocity_publisher.publish(velocity_msg)
        distance_moved3 = math.sqrt(pow(init_x-pose.x, 2)+pow(init_y-pose.y, 2))
        init_x = pose.x
        init_y = pose.y
        print(distance_moved3)
        goal_location_x = pose.x
        goal_location_y = pose.y - side
        error_last = 0
        Integrator=0

        while pose.y > goal_location_y:
            error = math.sqrt(pow(goal_location_x-pose.x, 2)+pow(goal_location_y-pose.y, 2))
            p_value = kp*error
            d_value = kd*(error-error_last)
            Integrator = Integrator + error
            if Integrator > i_max:
                Integrator = i_max
            elif Integrator < i_min:
                Integrator = i_min
            i_value = Integrator*ki
            error_last = error
            velocity_msg.linear.y = -(p_value + i_value + d_value)
            velocity_publisher.publish(velocity_msg)

        velocity_msg.linear.y = 0
        velocity_publisher.publish(velocity_msg)
        distance_moved4 = math.sqrt(pow(init_x-pose.x, 2)+pow(init_y-pose.y, 2))
        print(distance_moved4)
        init_x = pose.x
        init_y = pose.y
        goal_location_x = pose.x + side/2
        goal_location_y = pose.y
        error_last = 0
        Integrator=0

        while pose.x < goal_location_x:
            error = math.sqrt(pow(goal_location_x-pose.x, 2)+pow(goal_location_y-pose.y, 2))
            p_value = kp*error
            d_value = kd*(error-error_last)
            Integrator = Integrator + error
            if Integrator > i_max:
                Integrator = i_max
            elif Integrator < i_min:
                Integrator = i_min
            i_value = Integrator*ki
            error_last = error
            velocity_msg.linear.x = p_value + i_value + d_value
            velocity_publisher.publish(velocity_msg)

        velocity_msg.linear.x = 0
        velocity_publisher.publish(velocity_msg)
        distance_moved5 = math.sqrt(pow(init_x-pose.x, 2)+pow(init_y-pose.y, 2))
        print(distance_moved5)
        # rospy.spin()
        break


if __name__ == '__main__':
    try:
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, getDistance)
        side = int(input("Enter the side of the square: "))
        square(side)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
