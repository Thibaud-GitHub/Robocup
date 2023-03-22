#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def wheel_control_node():
    rospy.init_node('wheel_control_node')

    # Subscribe to the command topics for each wheel
    wheel1_sub = rospy.Subscriber('wheel1_command', Float64, wheel1_command_callback)
    wheel2_sub = rospy.Subscriber('wheel2_command', Float64, wheel2_command_callback)
    wheel3_sub = rospy.Subscriber('wheel3_command', Float64, wheel3_command_callback)
    wheel4_sub = rospy.Subscriber('wheel4_command', Float64, wheel4_command_callback)

    # Publish to the control topics for each wheel
    wheel1_pub = rospy.Publisher('wheel1_control', Float64, queue_size=1)
    wheel2_pub = rospy.Publisher('wheel2_control', Float64, queue_size=1)
    wheel3_pub = rospy.Publisher('wheel3_control', Float64, queue_size=1)
    wheel4_pub = rospy.Publisher('wheel4_control', Float64, queue_size=1)

    rospy.spin()

def wheel1_command_callback(data):
    wheel1_pub = rospy.Publisher('wheel1_command', Float64, queue_size=1)
    wheel1_pub.publish(1.0)

def wheel2_command_callback(data):
    wheel2_pub = rospy.Publisher('wheel2_command', Float64, queue_size=1)
    wheel2_pub.publish(1.0) 

def wheel3_command_callback(data):
    wheel3_pub = rospy.Publisher('wheel3_command', Float64, queue_size=1)
    wheel3_pub.publish(1.0)

def wheel4_command_callback(data):
    wheel4_pub = rospy.Publisher('wheel4_command', Float64, queue_size=1)
    wheel4_pub.publish(1.0) 

if __name__ == '__main__':
    try:
        wheel_control_node()
    except rospy.ROSInterruptException:
        pass