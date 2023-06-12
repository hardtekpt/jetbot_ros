#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
import qwiic_scmd


motors = qwiic_scmd.QwiicScmd()

wheel_separation = 0.1016
wheel_diameter = 0.060325
max_rpm = 200

def set_speed(left, right):

    max_pwm = 255.0

    motors.set_drive(0, 0, int(left * max_pwm))
    motors.set_drive(1, 0, int(right * max_pwm))
        
def stop():
    set_speed(0,0)
		
def twist_listener(msg):

    x = msg.linear.x
    rot = msg.angular.z
    
    # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/231a7219b36b8a6cdd100b59f66a3df2955df787/gazebo_plugins/src/gazebo_ros_diff_drive.cpp#L331
    left = x - rot * wheel_separation / 2.0
    right = x + rot * wheel_separation / 2.0
    
    # convert velocities to [-1,1]
    max_speed = (max_rpm / 60.0) * 2.0 * math.pi * (wheel_diameter * 0.5)

    left = max(min(left, max_speed), -max_speed) / max_speed
    right = max(min(right, max_speed), -max_speed) / max_speed
    
    set_speed(left, right)


# initialization
if __name__ == '__main__':
	
    rospy.init_node('jetbot_motors')

    rospy.loginfo("Motor connection status " + str(motors.connected))

    motors.begin()
    rospy.sleep(.250)
    motors.enable()
    rospy.sleep(.250)

    # stop the motors as precaution
    stop()

    # setup ros node	
    ns = rospy.get_namespace()
    rospy.Subscriber(ns+'cmd_vel', Twist, twist_listener)


    # start running
    rospy.spin()
        
    # stop motors before exiting
    stop()
    motors.disable()
    