#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def move_robot():
    # Créer un publisher pour envoyer les commandes de mouvement
    cmd_vel_pub = rospy.Publisher('/omni_robot/cmd_vel', Twist, queue_size=10)
    # Initialiser le noeud ROS
    rospy.init_node('move_robot_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # Créer un message Twist pour envoyer une commande de mouvement
        cmd_vel = Twist()
        cmd_vel.linear.x = 10.0 # Vitesse linéaire en x (m/s)
        cmd_vel.angular.z = 0.0 # Vitesse angulaire en z (rad/s)
        # Publier la commande sur le topic cmd_vel
        cmd_vel_pub.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
