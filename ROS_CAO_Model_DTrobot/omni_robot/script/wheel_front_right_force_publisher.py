#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64

def wheel_front_right_force_publisher():
    # Initialise le noeud
    rospy.init_node('wheel_front_right_force_publisher', anonymous=True)

    # Publie sur le topic wheel_front_right_controller/cmd_vel
    wheel_front_right_publisher = rospy.Publisher('wheel_front_right_controller/cmd_vel', Float64, queue_size=10)

    # Boucle d'envoi de la commande à intervalles réguliers
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        # Publie une force linéaire sur x
        wheel_front_right_publisher.publish(Float64(data=100.0))

        # Attend un intervalle de temps
        rate.sleep()

if __name__ == '__main__':
    try:
        wheel_front_right_force_publisher()
    except rospy.ROSInterruptException:
        pass
