#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from pid_controller import PIDController

# Création du contrôleur PID
pid_controller = PIDController(Kp=1.0, Ki=0.02, Kd=0.1)

# Fonction de rappel qui reçoit les commandes de vitesse
def wheel_command_callback(data):
    wheel_speed = pid_controller.update(data.data)
    wheel_speed_publisher.publish(wheel_speed)

# Initialisation du noeud
rospy.init_node('wheel_speed_controller')

# Abonnement au topic de commande
rospy.Subscriber('wheel_command', Float32, wheel_command_callback)

# Publication de la vitesse de la roue
wheel_speed_publisher = rospy.Publisher('wheel_speed', Float32, queue_size=10)

# Boucle principale
rospy.spin()