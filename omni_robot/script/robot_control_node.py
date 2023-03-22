#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import JointControllerState
from dynamic_reconfigure.server import Server
from my_robot.cfg import MyRobotConfig

class RobotControlNode:
    def __init__(self):
        rospy.init_node('robot_control_node')
        
        # Subscribers
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('wheel_speed_cmd', Float64MultiArray, self.wheel_speed_cmd_callback)

        # Publishers
        self.joint1_controller_pub = rospy.Publisher('/wheel_1_joint_controller/command', Float64, queue_size=10)
        self.joint2_controller_pub = rospy.Publisher('/wheel_2_joint_controller/command', Float64, queue_size=10)
        self.joint3_controller_pub = rospy.Publisher('/wheel_3_joint_controller/command', Float64, queue_size=10)
        self.joint4_controller_pub = rospy.Publisher('/wheel_4_joint_controller/command', Float64, queue_size=10)

        # Dynamic reconfigure server
        self.dyn_reconf_server = Server(MyRobotConfig, self.dyn_reconf_callback)

        self.joint_states = None
        self.wheel_speed_cmd = [0.0, 0.0, 0.0, 0.0]
        self.wheel_pid_gains = [10.0, 0.1, 0.0, 10.0]
        self.wheel_effort_cmd = [0.0, 0.0, 0.0, 0.0]
        self.wheel_effort_limits = [10.0, 10.0, 10.0, 10.0]

    def joint_states_callback(self, msg):
        self.joint_states = msg

    def wheel_speed_cmd_callback(self, msg):
        self.wheel_speed_cmd = msg.data

    def dyn_reconf_callback(self, config, level):
        self.wheel_pid_gains = [config.p_gain, config.i_gain, config.d_gain, config.i_clamp]
        self.wheel_effort_limits = [config.effort_limit] * 4
        return config

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.joint_states is not None:
                # Calcul des commandes d'effort en fonction des consignes de vitesse
                for i in range(4):
                    joint_name = 'wheel_{}_joint'.format(i+1)
                    joint_idx = self.joint_states.name.index(joint_name)
                    wheel_velocity = self.joint_states.velocity[joint_idx]
                    wheel_error = self.wheel_speed_cmd[i] - wheel_velocity
                    self.wheel_effort_cmd[i] = (
                        self.wheel_pid_gains[0]*wheel_error +
                        self.wheel_pid_gains[1]*wheel_error*self.joint_states.effort[joint_idx] +
                        self.wheel_pid_gains[2]*(wheel_error - self.joint_states.velocity[joint_idx-1]) +
                        self.wheel_pid_gains[3]*self.joint_states.effort[joint_idx]
                    )
                    self.wheel_effort_cmd[i] = max(min(self.wheel_effort_cmd[i], self.wheel_effort_limits[i]), -self.wheel_effort_limits[i])

                # Envoi des commandes d'effort aux contrôleurs de jointure
                self.joint1_controller_pub.publish(self.wheel_effort_cmd[0])
                self.joint2_controller_pub.publish(self.wheel_effort_cmd[1])
                self.joint3_controller_pub.publish(self.wheel_effort_cmd[2])
                self.joint4_controller_pub.publish(self.wheel_effort_cmd[3])

            rate.sleep()

if __name__ == '__main__':
    try:
        node = RobotControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
