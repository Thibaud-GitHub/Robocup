#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def wheel_cmd_node():
    rospy.init_node('wheel_cmd_node')

    # Subscribe to the direction command topics
    cmd_forward_sub = rospy.Subscriber('cmd_forward', Float64, cmd_forward_callback)
    cmd_right_sub = rospy.Subscriber('cmd_right', Float64, cmd_right_callback)
    cmd_left_sub = rospy.Subscriber('cmd_left', Float64, cmd_left_callback)
    cmd_back_sub = rospy.Subscriber('cmd_back', Float64, cmd_back_callback)

    # Publish to the speed command topics for each wheel
    wheel1_speed_pub = rospy.Publisher('wheel1_speed_cmd', Float64, queue_size=1)
    wheel2_speed_pub = rospy.Publisher('wheel2_speed_cmd', Float64, queue_size=1)
    wheel3_speed_pub = rospy.Publisher('wheel3_speed_cmd', Float64, queue_size=1)
    wheel4_speed_pub = rospy.Publisher('wheel4_speed_cmd', Float64, queue_size=1)

    rospy.spin()

def cmd_forward_callback(data):
    # TODO: Convert direction command to speed commands
    # and publish to the appropriate topics
    pass

def cmd_right_callback(data):
    # TODO: Convert direction command to speed commands
    # and publish to the appropriate topics
    pass

def cmd_left_callback(data):
    # TODO: Convert direction command to speed commands
    # and publish to the appropriate topics
    pass

def cmd_back_callback(data):
    # TODO: Convert direction command to speed commands
    # and publish to the appropriate topics
    pass

if __name__ == '__main__':
    try:
        wheel_cmd_node()
    except rospy.ROSInterruptException:
        pass