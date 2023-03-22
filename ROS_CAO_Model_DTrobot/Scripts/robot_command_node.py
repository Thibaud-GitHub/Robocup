#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class RobotCommandNode:
    def __init__(self):
        rospy.init_node('robot_command_node')
        
        # Subscribers
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        # Publishers
        self.wheel_speed_cmd_pub = rospy.Publisher('wheel_speed_cmd', Float64MultiArray, queue_size=10)

        self.linear_speed = 0.0
        self.angular_speed = 0.0

    def cmd_vel_callback(self, msg):
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # Calcul des vitesses de chaque roue en fonction de la commande de l'utilisateur
            wheel_speeds = [
                self.linear_speed - self.angular,
                self.angular_speed,
                self.linear_speed + self.angular_speed,
                self.angular_speed,
                self.linear_speed - self.angular_speed,
            ]

        # Publication des consignes de vitesse sur le topic wheel_speed_cmd
        msg = Float64MultiArray()
        msg.data = wheel_speeds
        self.wheel_speed_cmd_pub.publish(msg)

        rate.sleep()
if name == 'main':
    try:
        node = RobotCommandNode()
        node.run()
    except rospy.ROSInterruptException:
        pass