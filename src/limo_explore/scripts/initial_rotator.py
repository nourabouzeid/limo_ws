#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def initial_rotator():
    rospy.init_node('initial_rotator', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    angular_speed = 1
    duration = 5.0       

    rospy.loginfo("Initial Rotator: Waiting for ROS time to start...")
    while rospy.Time.now() == rospy.Time(0):
        rospy.Rate(10).sleep()
    
    start_time = rospy.Time.now()
    rospy.loginfo("Initial Rotator: ROS time is available. Starting rotation.")
    
    twist_cmd = Twist()
    twist_cmd.angular.z = angular_speed

    rospy.loginfo("Initial Rotator: Starting %s second rotation at %s rad/s.", 
                  duration, angular_speed)

    while rospy.Time.now() < start_time + rospy.Duration(duration):
        if rospy.is_shutdown():
            break
        
        cmd_vel_pub.publish(twist_cmd)
        rospy.Rate(10).sleep() 

    rospy.loginfo("Initial Rotator: Rotation complete. Stopping robot.")
    twist_cmd.angular.z = 0.0
    cmd_vel_pub.publish(twist_cmd)

    rospy.signal_shutdown("Initial rotation finished.")

if __name__ == '__main__':
    try:
        initial_rotator()
    except rospy.ROSInterruptException:
        pass
