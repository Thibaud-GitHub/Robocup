#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def wheel_control_node():
    rospy.init_node('wheel_control_node')

    # Subscribe to the command topics for each wheel
    wheel1_sub = rospy.Subscriber('wheel1_command', Float64, wheel1_command_callback)

    # Publish to the control topics for each wheel
    wheel1_pub = rospy.Publisher('wheel1_control', Float64, queue_size=1)

    rospy.spin()

def wheel1_command_callback(data):
    wheel1_pub = rospy.Publisher('wheel1_command', Float64, queue_size=1)
    wheel1_pub.publish(1.0)

if __name__ == '__main__':
    try:
        wheel_control_node()
    except rospy.ROSInterruptException:
        pass