#!/usr/bin/env python

import rospy
from control_msgs.msg  import JointControllerState
from std_msgs.msg import Float64
from math import pi

class JointControllerPublisher:
    def __init__(self):
        rospy.init_node('joint_controller_publisher')

        # Define joint names and PID values
        self.joint_names = ['joint_wfr', 'joint_wfl', 'joint_wbl', 'joint_wbr']
        self.joint_p = 100.0
        self.joint_i = 0.0
        self.joint_d = 0.0

        # Initialize publishers for each joint
        self.joint_publishers = []
        for name in self.joint_names:
            pub = rospy.Publisher('/{}/command'.format(name), Float64, queue_size=10)
            self.joint_publishers.append(pub)

        # Initialize subscribers for each joint
        self.joint_subscribers = []
        for name in self.joint_names:
            sub = rospy.Subscriber('/{}/state'.format(name), JointControllerState, self.joint_state_callback)
            self.joint_subscribers.append(sub)

    def joint_state_callback(self, msg):
        # Get joint index from joint name
        joint_index = self.joint_names.index(msg.joint_name)

        # Compute error and command joint effort
        error = msg.set_point - msg.process_value
        command_effort = self.joint_p * error + self.joint_i * msg.i_clamp + self.joint_d * (msg.d * -1.0)

        # Publish command effort
        self.joint_publishers[joint_index].publish(command_effort)

if __name__ == '__main__':
    jcp = JointControllerPublisher()
    rospy.spin()
