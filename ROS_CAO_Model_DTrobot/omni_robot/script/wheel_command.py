#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import std_msgs
import geometry_msgs
from geometry_msgs.msg import Twist

def wheel_front_right_controller():
    # Initialise le noeud
    rospy.init_node('wheel_front_right_controller', anonymous=True)

    # Abonnez-vous au topic cmd_vel pour recevoir des commandes de vitesse et de rotation
    rospy.Subscriber("wheel_front_right_controller/cmd_vel", Twist, cmd_vel_callback)

    # Boucle d'exécution principale
    rospy.spin()

def cmd_vel_callback(msg):
    # Récupérez les vitesses linéaire et angulaire à partir de la commande Twist
    linear_vel_x = msg.linear.x
    angular_vel_z = msg.angular.z

    # Ici, vous pouvez utiliser ces vitesses pour contrôler la roue
    # Par exemple, vous pouvez appliquer une force proportionnelle à la vitesse linéaire

if __name__ == '__main__':
    try:
        wheel_front_right_controller()
    except rospy.ROSInterruptException:
        pass
