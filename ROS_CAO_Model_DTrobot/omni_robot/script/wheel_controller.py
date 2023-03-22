#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def wheel_controller():

    # initialize the node
    rospy.init_node('wheel_controller', anonymous=True)

    # create publishers for each wheel joint
    wheel_joints = ['joint_wfr', 'joint_wfl', 'joint_wbl', 'joint_wbr']
    wheel_pubs = [rospy.Publisher(joint_name + '_velocity_controller/command', Float64, queue_size=10) for joint_name in wheel_joints]

    # set the velocity for each wheel
    velocity = 100
    velocities = [velocity] * 4

    # create message with the velocity for each wheel
    vel_msg = Float64()
    vel_msg.data = velocity

    # publish the velocity for each wheel
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for pub in wheel_pubs:
            pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        wheel_controller()
    except rospy.ROSInterruptException:
        pass
