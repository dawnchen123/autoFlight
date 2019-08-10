# -*- coding: utf-8 -*- 
'''
    可视化颜色阈值调参软件
'''

import cv2
import numpy as np
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 颜色阈值下界(HSV) lower boudnary
lowerb = (0, 24, 121) 
# 颜色阈值上界(HSV) upper boundary
upperb = (255, 255, 255)

pub = rospy.Publisher('/circle/position', Image, queue_size=1)
pub_Img = rospy.Publisher('circle/image', Image, queue_size=1) 
#rospy.init_node('detectCircle', anonymous=True)
bridge = CvBridge()
circleCenter = np.ones((3,1),dtype=np.uint8)



def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_hsv, lowerb, upperb)
    #cv2.imshow('mask', mask)
    kernel = np.ones((8,8),np.uint8)
    dilation = cv2.dilate(mask,kernel,iterations = 1)
    #gauss_img = cv2.GaussianBlur(dilation,(5,5),0)
    #canny_img = cv2.Canny(gauss_img, 50, 150)
    contours = cv2.findContours(dilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[1]  
    #print(len(contours))
    for i in range(len(contours)):
        if(cv2.contourArea(contours[i])>17000 and cv2.contourArea(contours[i])<110000):
            #最小外接圆
            (x,y),radius  = cv2.minEnclosingCircle(contours[i])
            center = (int(x),int(y))
            radius = int(radius)
            #img = cv2.circle(img,center,radius,(0,255,0),2)
            print(radius)
            
            if(3.1415*radius*radius/cv2.contourArea(contours[i])<1.3):
                cv2.drawContours(img,contours[i],-1,(0,0,255),3) 
                #图像矩
                #M=cv2.moments(contours[i])                    
                #cx=int(M['m10']/M['m00'])
                #cy=int(M['m01']/M['m00'])
                # print(cv2.contourArea(contours[i]),cx,cy)
                # circleCenter[0,0]=int(cx/2)
                # circleCenter[1,0]=int(cy/2)
                print(cv2.contourArea(contours[i]),center[0],center[1])
                circleCenter[0,0]=int(center[0]/2)
                circleCenter[1,0]=int(center[1]/2)
                circleCenter[2,0]=int(radius)
                pub.publish(bridge.cv2_to_imgmsg(circleCenter, "mono8"))
    img2 = cv2.resize(img,(160,128))
    pub_Img.publish(bridge.cv2_to_imgmsg(img2, '8UC3'))


def listener():
    rospy.init_node("usb_cam", anonymous=True)
    zed_topic = "/usb_cam/image_raw"
    
    rospy.Subscriber(zed_topic, Image, image_callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()