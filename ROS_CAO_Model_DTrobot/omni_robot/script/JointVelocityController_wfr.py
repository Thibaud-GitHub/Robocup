#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class JointVelocityController:

    def __init__(self):
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        self.joint_velocities = JointState()
        self.joint_velocities.name = ['joint_wfr']
        self.joint_velocities.velocity = [0.0]

    def cmd_vel_callback(self, cmd_vel):
        # Convertir la commande de vitesse linéaire en une vitesse angulaire pour le joint
        joint_vel = cmd_vel.linear.x / 0.1 * 1.0
        # Convertir la commande de vitesse angulaire en une commande de vitesse pour le joint
        self.joint_velocities.velocity[0] = joint_vel
        # Publier la commande de vitesse pour le joint
        self.joint_pub.publish(self.joint_velocities)

    def run(self):
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        while not rospy.is_shutdown():
            self.joint_pub.publish(self.joint_velocities)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('joint_velocity_controller', anonymous=True)
        jvc = JointVelocityController()
        jvc.run()
    except rospy.ROSInterruptException:
        pass
