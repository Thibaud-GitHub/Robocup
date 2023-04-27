#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import sys
import math
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np 

class Ball_tracker:
    global velr
    global velt
    
    def __init__(self):
        self.cx=0
        self.cy=0
        self.velx=3 #max vel linear x
        self.velz=2.5 #max vel angular z

        self.bridge = CvBridge()
        self.image_received = False
        img_topic = "/final/camera1/image_raw"
        
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)
        self.aim_bot = 0
        self.dpos = [0,0]

        # Allow up to one second to connection
        rospy.sleep(1)
        
    def callback(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")      
        self.image_received = True
        self.image = cv_image
        self.find_object(cv_image)

    def find_object(self,img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([5, 50, 50])
        upper = np.array([15, 255, 255])
        flag = False
        mask_frame=cv2.inRange(hsv_frame,lower,upper)
        cv2.imshow("mask",mask_frame)
        contours,hierarchy= cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        X,Y,W,H=0,0,0,0
        point=(int(self.image.shape[0]/2),int(self.image.shape[1]*3/5))
        for pic, contour in enumerate(contours):
            flag = True
            area = cv2.contourArea(contour)
            if(area > 30):
                x, y, w, h = cv2.boundingRect(contour)
                if(w*h>W*H):
                    X, Y, W, H= x, y, w, h 
            cx = X + W/2
            cy = Y + H/2
            self.dpos[1] = -(cy - point[1])

        img = cv2.rectangle(img, (X, Y),(X +W, Y + H),(0, 0, 255), 2)
        cv2.circle(img,point,4,(0,255,0),-1)
        cv2.imshow("window", img)
        cv2.waitKey(3)
        
    def move_to_object(self, flag):
        margin = 5
        if flag:
            if self.dpos[0] < -margin or self.dpos[0] > margin:
                ndif = abs(self.dpos[0] - margin)/(self.image.shape[0]/2)
                prop = math.log(1 + 5*ndif)/math.e

                Ball_tracker.velr = self.velz*prop
                if self.dpos[0] < -margin:
                    Ball_tracker.rot_anticlock()
                    rospy.loginfo("Rotating left")
                elif self.dpos[0] > margin:
                    Ball_tracker.rot_clock()
                    rospy.loginfo("Rotating right")
            else:
                Ball_tracker.velr = 0
                Ball_tracker.rot_anticlock()
                rospy.loginfo("Not rotating")

            if self.dpos[1] < -margin or self.dpos[1] > margin:
                ndif = abs(self.dpos[1] - margin)/(self.image.shape[1]/2)
                prop = math.log(1 + 100*ndif)/math.e

                Ball_tracker.velt = self.velx*prop
                if self.dpos[1] < -margin:
                    Ball_tracker.backward()
                    rospy.loginfo("Translating backwards")
                elif self.dpos[1] > margin:
                    Ball_tracker.forward()
                    rospy.loginfo("Translating forward")

            else:
                Ball_tracker.velt = 0
                Ball_tracker.forward()
                rospy.loginfo("Not translating")

        else:
            Ball_tracker.velr = self.velz
            Ball_tracker.velt = 0

            Ball_tracker.rot_anticlock()
            rospy.loginfo(".........Looking for the ball.........")
    
	def callback_state_joint(joint_state):
		wheel_velocities = joint_state.velocity[-4:]  # récupérer les 4 dernières valeurs de la liste velocity
		rospy.loginfo("Vitesse des roues : %s", wheel_velocities)

    def velocity_publisher():
		#---------------------- publisher setting ------------------------------------------------------------#
		pub1=rospy.Publisher("/omni_robot/wfl_joint_velocity_controller/command",Float64,queue_size=10) 
		pub2=rospy.Publisher("/omni_robot/wfr_joint_velocity_controller/command",Float64,queue_size=10)
		pub3=rospy.Publisher("/omni_robot/wbl_joint_velocity_controller/command",Float64,queue_size=10)
		pub4=rospy.Publisher("/omni_robot/wbr_joint_velocity_controller/command",Float64,queue_size=10)

		pub1.publish(Ball_tracker.x)                              #forawards 10 | -10 | 10 | -10  
		pub2.publish(Ball_tracker.x)
		pub3.publish(Ball_tracker.x) 
		pub4.publish(Ball_tracker.x)

	def rot_anticlock():
		#---------------------- publisher setting ------------------------------------------------------------#
		pub1=rospy.Publisher("/omni_robot/wfl_joint_velocity_controller/command",Float64,queue_size=10) 
		pub2=rospy.Publisher("/omni_robot/wfr_joint_velocity_controller/command",Float64,queue_size=10)
		pub3=rospy.Publisher("/omni_robot/wbl_joint_velocity_controller/command",Float64,queue_size=10)
		pub4=rospy.Publisher("/omni_robot/wbr_joint_velocity_controller/command",Float64,queue_size=10)

		pub1.publish(Ball_tracker.velr)                     
		pub2.publish(Ball_tracker.velr)
		pub3.publish(Ball_tracker.velr) 
		pub4.publish(Ball_tracker.velr)			

	def rot_clock():
		#---------------------- publisher setting ------------------------------------------------------------#
		pub1=rospy.Publisher("/omni_robot/wfl_joint_velocity_controller/command",Float64,queue_size=10) 
		pub2=rospy.Publisher("/omni_robot/wfr_joint_velocity_controller/command",Float64,queue_size=10)
		pub3=rospy.Publisher("/omni_robot/wbl_joint_velocity_controller/command",Float64,queue_size=10)
		pub4=rospy.Publisher("/omni_robot/wbr_joint_velocity_controller/command",Float64,queue_size=10)

		pub1.publish(-Ball_tracker.velr)                                           
		pub2.publish(-Ball_tracker.velr)
		pub3.publish(-Ball_tracker.velr) 
		pub4.publish(-Ball_tracker.velr)

	def forward():
		#---------------------- publisher setting ------------------------------------------------------------#
		pub1=rospy.Publisher("/omni_robot/wfl_joint_velocity_controller/command",Float64,queue_size=10) 
		pub2=rospy.Publisher("/omni_robot/wfr_joint_velocity_controller/command",Float64,queue_size=10)
		pub3=rospy.Publisher("/omni_robot/wbl_joint_velocity_controller/command",Float64,queue_size=10)
		pub4=rospy.Publisher("/omni_robot/wbr_joint_velocity_controller/command",Float64,queue_size=10)

		pub1.publish(Ball_tracker.velt)                    
		pub2.publish(-Ball_tracker.velt)
		pub3.publish(-Ball_tracker.velt) 
		pub4.publish(Ball_tracker.velt)

	def backward():
		#---------------------- publisher setting ------------------------------------------------------------#
		pub1=rospy.Publisher("/omni_robot/wfl_joint_velocity_controller/command",Float64,queue_size=10) 
		pub2=rospy.Publisher("/omni_robot/wfr_joint_velocity_controller/command",Float64,queue_size=10)
		pub3=rospy.Publisher("/omni_robot/wbl_joint_velocity_controller/command",Float64,queue_size=10)
		pub4=rospy.Publisher("/omni_robot/wbr_joint_velocity_controller/command",Float64,queue_size=10)

		pub1.publish(-Ball_tracker.velt)                                         
		pub2.publish(Ball_tracker.velt)
		pub3.publish(Ball_tracker.velt) 
		pub4.publish(-Ball_tracker.velt)			

	def stop():
        cv2.destroyAllWindows()
        Ball_tracker.x=0
        rate=rospy.Rate(50)
        Ball_tracker.velocity_publisher()

    def main():
        rospy.init_node("Ball_tracker",anonymous=False)
        sub=rospy.Subscriber("/omni_robot/joint_states",JointState,Ball_tracker.callback_state_joint,queue_size=10)
        rate=rospy.Rate(50)

        Ball_tracker.velr=0
        Ball_tracker.velt=0

        rospy.on_shutdown(Ball_tracker.stop)

        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            rospy.spin()

if __name__ == '__main__':
    Ball_tracker.main()