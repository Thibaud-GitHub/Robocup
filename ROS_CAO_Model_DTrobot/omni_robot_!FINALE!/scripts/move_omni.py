#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class move_omni:
	global x

	def callback(joint_state):
		wheel_velocities = joint_state.velocity[-6:]  # récupérer les 4 dernières valeurs de la liste velocity
		rospy.loginfo("Vitesse des roues : %s", wheel_velocities)

	def stop():
		move_omni.x=0
		rate=rospy.Rate(50)
		move_omni.velocity_publisher()

	def velocity_publisher():
		#---------------------- publisher setting ------------------------------------------------------------#
		pub1=rospy.Publisher("/omni_robot/wfl_joint_velocity_controller/command",Float64,queue_size=10) 
		pub2=rospy.Publisher("/omni_robot/wfr_joint_velocity_controller/command",Float64,queue_size=10)
		pub3=rospy.Publisher("/omni_robot/wbl_joint_velocity_controller/command",Float64,queue_size=10)
		pub4=rospy.Publisher("/omni_robot/wbr_joint_velocity_controller/command",Float64,queue_size=10)

		sub=rospy.Subscriber("/omni_robot/joint_states",JointState,move_omni.callback,queue_size=10)

		pub1.publish(move_omni.x)                                         #forawards 10 | -10 | 10 | -10  
		pub2.publish(move_omni.x)
		pub3.publish(move_omni.x) 
		pub4.publish(move_omni.x)
		

	def main():
		rospy.init_node("move_omni",anonymous=False)
		rate=rospy.Rate(50)
		move_omni.x=3

		while not rospy.is_shutdown():
			move_omni.velocity_publisher()                         
			rate.sleep()
			rospy.on_shutdown(move_omni.stop)

if __name__ == '__main__':
    move_omni.main()