#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import numpy as np

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

    x = np.linspace(0.1, 1.139028164, 100)
    goal_location_x = init_x + x

    y = 1 + 0.5 * (((x ** (2)) ** (1/3))  - ((x ** (4)) ** (1/3) + 4 * (1 - (x ** 2))) ** 0.5)
    goal_location_y = init_y + y
    
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    velocity_msg.linear.z = 0
    velocity_msg.angular.x = 0
    velocity_msg.angular.y = 0
    velocity_msg.angular.z = 0

    # distance_tolerance = 0.7
    
    kp = 8
    kp_v = 0.3
    kd = 100
    ki = 0
    error_last_x = 0
    error_last_y = 0
    error_last_v = 0
    i_max = 4
    i_min = 0
    Integrator=0
    Integrator_v=0

    while not rospy.is_shutdown():

        for location_x,location_y in zip(goal_location_x, goal_location_y):

            print(location_x, location_y)
            error_last_x = location_x-pose.x
            error_last_y = location_y-pose.y

            while pose.x<location_x:
                
                error_x = location_x-pose.x
                error_y = location_y-pose.y 
                error_v = math.atan2(goal_location_y[0] - pose.y, goal_location_x[0] - pose.x) - pose.theta

                p_value_x = kp*error_x
                p_value_y = kp*error_y
                p_value_ang = kp_v*error_v

                d_value_x = kd*(error_x-error_last_x)
                d_value_y = kd*(error_y-error_last_y)
                d_value_ang = kd*(error_v - error_last_v)

                # Integrator = Integrator + error
                # Integrator_v = Integrator_v + error_v
                # # print(Integrator)
                # if Integrator > i_max:
                #     Integrator = i_max
                # elif Integrator < i_min:
                #     Integrator = i_min
                # # print(Integrator)
                # i_value = Integrator*ki
                # i_value_ang = Integrator_v*ki

                error_last_x = error_x
                error_last_y = error_y
                error_last_v = error_v

                velocity_msg.linear.x = p_value_x + d_value_y #+ i_value
                velocity_msg.linear.y = p_value_y + d_value_y
                # velocity_msg.angular.z = math.sqrt(pow(p_value_ang + d_value_ang, 2))  #
                velocity_publisher.publish(velocity_msg)
            velocity_msg.linear.x = 0
            velocity_msg.linear.y = 0
            # velocity_msg.angular.z = 0
            velocity_publisher.publish(velocity_msg)
            
            print(pose.x, pose.y)
            print("Going to next point bitch")

        
        distance_moved1 = math.sqrt(pow(init_x-pose.x, 2)+pow(init_y-pose.y, 2))
        print(distance_moved1)
        print("next command")


        x = np.linspace(1.137028164, 0, 100)
        new_goal_location_x = init_x + x

        y = 1 + 0.5 * (((x ** (2)) ** (1/3)) + ((x ** (4)) ** (1/3) + 4 * (1 - (x ** 2))) ** 0.5)
        new_goal_location_y = init_y + y

        kp = 10
        kp_v = 0.3
        kd = 100
        ki = 0
        error_last_x = 0
        error_last_y = 0
        error_last_v = 0
        i_max = 4
        i_min = 0
        Integrator=0
        Integrator_v=0

        for location_x,location_y in zip(new_goal_location_x, new_goal_location_y):

            print(location_x, location_y)
            error_last_x = location_x-pose.x
            error_last_y = location_y-pose.y

            while pose.x>location_x:
                
                error_x = location_x-pose.x
                error_y = location_y-pose.y
                error_v = math.atan2(goal_location_y[0] - pose.y, goal_location_x[0] - pose.x) - pose.theta

                p_value_x = kp*error_x
                p_value_y = kp*error_y
                p_value_ang = kp_v*error_v

                d_value_x = kd*(error_x-error_last_x)
                d_value_y = kd*(error_y-error_last_y)
                d_value_ang = kd*(error_v - error_last_v)

                # Integrator = Integrator + error
                # Integrator_v = Integrator_v + error_v
                # # print(Integrator)
                # if Integrator > i_max:
                #     Integrator = i_max
                # elif Integrator < i_min:
                #     Integrator = i_min
                # # print(Integrator)
                # i_value = Integrator*ki
                # i_value_ang = Integrator_v*ki

                error_last_x = error_x
                error_last_y = error_y
                error_last_v = error_v

                velocity_msg.linear.x = p_value_x + d_value_y #+ i_value
                velocity_msg.linear.y = p_value_y + d_value_y
                # velocity_msg.angular.z = math.sqrt(pow(p_value_ang + d_value_ang, 2))  #
                velocity_publisher.publish(velocity_msg)
            velocity_msg.linear.x = 0
            velocity_msg.linear.y = 0
            # velocity_msg.angular.z = 0
            velocity_publisher.publish(velocity_msg)
            
            print(pose.x, pose.y)
            print("Going to next point bitch")

        
        distance_moved1 = math.sqrt(pow(init_x-pose.x, 2)+pow(init_y-pose.y, 2))
        print(distance_moved1)
        print("next command")


        x = np.linspace(-0.1, -1.139028164, 100)
        new_goal_location_x = init_x + x

        y = 1 + 0.5 * (((x ** (2)) ** (1/3)) + ((x ** (4)) ** (1/3) + 4 * (1 - (x ** 2))) ** 0.5)
        new_goal_location_y = init_y + y

        kp = 10
        kp_v = 0.3
        kd = 100
        ki = 0
        error_last_x = 0
        error_last_y = 0
        error_last_v = 0
        i_max = 4
        i_min = 0
        Integrator=0
        Integrator_v=0

        for location_x,location_y in zip(new_goal_location_x, new_goal_location_y):

            print(location_x, location_y)
            error_last_x = location_x-pose.x
            error_last_y = location_y-pose.y

            while pose.x>location_x:
                
                error_x = location_x-pose.x
                error_y = location_y-pose.y
                error_v = math.atan2(goal_location_y[0] - pose.y, goal_location_x[0] - pose.x) - pose.theta

                p_value_x = kp*error_x
                p_value_y = kp*error_y
                p_value_ang = kp_v*error_v

                d_value_x = kd*(error_x-error_last_x)
                d_value_y = kd*(error_y-error_last_y)
                d_value_ang = kd*(error_v - error_last_v)

                # Integrator = Integrator + error
                # Integrator_v = Integrator_v + error_v
                # # print(Integrator)
                # if Integrator > i_max:
                #     Integrator = i_max
                # elif Integrator < i_min:
                #     Integrator = i_min
                # # print(Integrator)
                # i_value = Integrator*ki
                # i_value_ang = Integrator_v*ki

                error_last_x = error_x
                error_last_y = error_y
                error_last_v = error_v

                velocity_msg.linear.x = p_value_x + d_value_y #+ i_value
                velocity_msg.linear.y = p_value_y + d_value_y
                # velocity_msg.angular.z = math.sqrt(pow(p_value_ang + d_value_ang, 2))  #
                velocity_publisher.publish(velocity_msg)
            velocity_msg.linear.x = 0
            velocity_msg.linear.y = 0
            # velocity_msg.angular.z = 0
            velocity_publisher.publish(velocity_msg)
            
            print(pose.x, pose.y)
            print("Going to next point bitch")

        
        distance_moved1 = math.sqrt(pow(init_x-pose.x, 2)+pow(init_y-pose.y, 2))
        print(distance_moved1)
        print("next command")

        x = np.linspace(-1.137028164, 0, 100)
        new_goal_location_x = init_x + x

        y = 1 + 0.5 * (((x ** (2)) ** (1/3)) - ((x ** (4)) ** (1/3) + 4 * (1 - (x ** 2))) ** 0.5)
        new_goal_location_y = init_y + y

        kp = 10
        kp_v = 0.3
        kd = 100
        ki = 0
        error_last_x = 0
        error_last_y = 0
        error_last_v = 0
        i_max = 4
        i_min = 0
        Integrator=0
        Integrator_v=0

        for location_x,location_y in zip(new_goal_location_x, new_goal_location_y):

            print(location_x, location_y)
            error_last_x = location_x-pose.x
            error_last_y = location_y-pose.y

            while pose.x<location_x:
                
                error_x = location_x-pose.x
                error_y = location_y-pose.y
                error_v = math.atan2(goal_location_y[0] - pose.y, goal_location_x[0] - pose.x) - pose.theta

                p_value_x = kp*error_x
                p_value_y = kp*error_y
                p_value_ang = kp_v*error_v

                d_value_x = kd*(error_x-error_last_x)
                d_value_y = kd*(error_y-error_last_y)
                d_value_ang = kd*(error_v - error_last_v)

                # Integrator = Integrator + error
                # Integrator_v = Integrator_v + error_v
                # # print(Integrator)
                # if Integrator > i_max:
                #     Integrator = i_max
                # elif Integrator < i_min:
                #     Integrator = i_min
                # # print(Integrator)
                # i_value = Integrator*ki
                # i_value_ang = Integrator_v*ki

                error_last_x = error_x
                error_last_y = error_y
                error_last_v = error_v

                velocity_msg.linear.x = p_value_x + d_value_y #+ i_value
                velocity_msg.linear.y = p_value_y + d_value_y
                # velocity_msg.angular.z = math.sqrt(pow(p_value_ang + d_value_ang, 2))  #
                velocity_publisher.publish(velocity_msg)
            velocity_msg.linear.x = 0
            velocity_msg.linear.y = 0
            # velocity_msg.angular.z = 0
            velocity_publisher.publish(velocity_msg)
            
            print(pose.x, pose.y)
            print("Going to next point bitch")

        
        distance_moved1 = math.sqrt(pow(init_x-pose.x, 2)+pow(init_y-pose.y, 2))
        print(distance_moved1)
        print("next command")
      
        break

if __name__ == '__main__':
    try:
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, getDistance)
        # side = int(input("Enter the side of the square: "))
        heart()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")

