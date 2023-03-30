#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

def set_model_state():
    # Initialise le noeud
    rospy.init_node('set_model_state', anonymous=True)

    # Publie sur le topic /gazebo/set_model_state
    model_state_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    # Crée un message ModelState pour définir la position et l'orientation du modèle
    model_state_msg = ModelState()
    model_state_msg.model_name = 'omni_robot' # Remplacer par le nom de votre modèle
    model_state_msg.pose = Pose()
    model_state_msg.pose.position.x = 0.0 # Remplacer par la position x souhaitée
    model_state_msg.pose.position.y = 0.0 # Remplacer par la position y souhaitée
    model_state_msg.pose.position.z = 0.0 # Remplacer par la position z souhaitée
    model_state_msg.pose.orientation.x = 0.0
    model_state_msg.pose.orientation.y = 0.0
    model_state_msg.pose.orientation.z = 0.0
    model_state_msg.pose.orientation.w = 1.0

    # Crée un message Twist pour définir la vitesse et l'orientation angulaire du modèle
    model_state_msg.twist = Twist()
    model_state_msg.twist.linear.x = 0.0
    model_state_msg.twist.linear.y = 0.0
    model_state_msg.twist.linear.z = 0.0
    model_state_msg.twist.angular.x = 0.0
    model_state_msg.twist.angular.y = 0.0
    model_state_msg.twist.angular.z = 0.0

    # Publie le message en boucle
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        model_state_publisher.publish(model_state_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        set_model_state()
    except rospy.ROSInterruptException:
        pass
