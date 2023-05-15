#!/usr/bin/env python3

import sys
import math
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float64

class Ball_tracker:
    global vel
    def __init__(self):
        #publisher to velocity topic 
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.rate=rospy.Rate(1)
        self.rot=Twist()

        self.cx=0
        self.cy=0
        self.velx=3 #max vel linear x
        self.velz=2.5 #max vel angular z

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/omni_robot/camera_bot/image_raw"
        
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

            self.dpos[0] =  (cx - point[0])
            self.dpos[1] = -(cy - point[1])
        self.move_to_object(flag)
        img = cv2.rectangle(img, (X, Y),(X +W, Y + H),(0, 0, 255), 2)
        cv2.circle(img,point,4,(0,255,0),-1)
        cv2.imshow("window", img)
        cv2.waitKey(3)

    def move_to_object(self, flag):
        pub1=rospy.Publisher("/omni_robot/wfl_joint_velocity_controller/command",Float64,queue_size=10) 
        pub2=rospy.Publisher("/omni_robot/wfr_joint_velocity_controller/command",Float64,queue_size=10)
        pub3=rospy.Publisher("/omni_robot/wbl_joint_velocity_controller/command",Float64,queue_size=10)
        pub4=rospy.Publisher("/omni_robot/wbr_joint_velocity_controller/command",Float64,queue_size=10)

        margin_ang = 3
        margin = 1
        
        if flag:
            if self.dpos[0] < -margin_ang or self.dpos[0] > margin_ang:
                ndif = abs(self.dpos[0] - margin_ang)/(self.image.shape[0]/2)
                prop = math.log(1 + 5*ndif)/math.e 
                #vel = self.velz*prop+1
                vel = 3
                print("rotation speed: %5.3f" % vel)
                
                if self.dpos[0] < -margin_ang:
                    text="rotating left"
                    pub1.publish(-vel)
                    pub2.publish(-vel)
                    pub3.publish(-vel)
                    pub4.publish(-vel)

                elif self.dpos[0] > margin_ang:
                    text="rotating right"
                    pub1.publish(vel*0.1)
                    pub2.publish(vel*0.1)
                    pub3.publish(vel*0.1) 
                    pub4.publish(vel*0.1)  
            else:
                text="not rotating"
                pub1.publish(0)
                pub2.publish(0)
                pub3.publish(0) 
                pub4.publish(0)

            if self.dpos[1] < -margin or self.dpos[1] > margin:
                ndif = abs(self.dpos[1] - margin)/(self.image.shape[1]/2)
                prop = math.log(1 + 100*ndif)/math.e
                #vel = self.velx*prop+1
                vel = 3.5
                print("translation speed: %5.3f" % vel)

                if self.dpos[1] < -margin:
                    text+=", translating backwards"
                    pub1.publish(-vel)
                    pub2.publish(-vel)
                    pub3.publish(-vel) 
                    pub4.publish(-vel)
                elif self.dpos[1] > margin:
                    text+=", translating forward"
                    pub1.publish(-vel) 
                    pub2.publish(vel*1.1)
                    pub3.publish(-vel) 
                    pub4.publish(vel*1.1)
                
            else:
                text+=", not translating"
                pub1.publish(0)
                pub2.publish(0)
                pub3.publish(0) 
                pub4.publish(0)

        else:
            text="seeking ball........."
            vel = 4
            pub1.publish(vel)
            pub2.publish(vel)
            pub3.publish(vel) 
            pub4.publish(vel)
         
    def stop(self):
        pub1=rospy.Publisher("/omni_robot/wfl_joint_velocity_controller/command",Float64,queue_size=10) 
        pub2=rospy.Publisher("/omni_robot/wfr_joint_velocity_controller/command",Float64,queue_size=10)
        pub3=rospy.Publisher("/omni_robot/wbl_joint_velocity_controller/command",Float64,queue_size=10)
        pub4=rospy.Publisher("/omni_robot/wbr_joint_velocity_controller/command",Float64,queue_size=10)
        vel = 0
        pub1.publish(-vel)
        pub2.publish(vel)
        pub3.publish(-vel) 
        pub4.publish(vel)

        cv2.destroyAllWindows()
        print('######## stop #########')

    def velocity_publisher():
		#---------------------- publisher setting ------------------------------------------------------------#
        pub1=rospy.Publisher("/omni_robot/wfl_joint_velocity_controller/command",Float64,queue_size=10) 
        pub2=rospy.Publisher("/omni_robot/wfr_joint_velocity_controller/command",Float64,queue_size=10)
        pub3=rospy.Publisher("/omni_robot/wbl_joint_velocity_controller/command",Float64,queue_size=10)
        pub4=rospy.Publisher("/omni_robot/wbr_joint_velocity_controller/command",Float64,queue_size=10)

        pub1.publish(Ball_tracker.move_to_object.vel)                                         #forawards 10 | -10 | 10 | -10  
        pub2.publish(Ball_tracker.move_to_object.vel)
        pub3.publish(Ball_tracker.move_to_object.vel) 
        pub4.publish(Ball_tracker.move_to_object.vel)

def main():
    rospy.init_node('ball_tracker', anonymous=False)
    script = Ball_tracker()

    rospy.on_shutdown(script.stop)

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        rospy.spin()


if __name__ == '__main__':

    main()