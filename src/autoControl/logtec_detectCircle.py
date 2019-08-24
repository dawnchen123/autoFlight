# -*- coding: utf-8 -*- 
'''
    可视化颜色阈值调参软件
'''

import cv2
import numpy as np
import math
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan

pub_Img = rospy.Publisher('circle/image', Image, queue_size=1) 
pubScan = rospy.Publisher('/circle/scan', LaserScan, queue_size=1)

circleScan = LaserScan()
#rospy.init_node('detectCircle', anonymous=True)
bridge = CvBridge()



def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    #img = cv2.resize(img,(160,128))
    pub_Img.publish(bridge.cv2_to_imgmsg(img, '8UC3'))

def scan_callback(msg):
    circle_distance = []
    circle_angle = []
    #print(len(msg.ranges))   #1440 max
    for i in range(len(msg.ranges)):
        if(msg.ranges[i]<3 and msg.ranges[i]>0.4):
            # tmp_angle = math.pi*2*i/1440
            # tmp_radius = math.sin(tmp_angle)*msg.ranges[i]
            #print(i,msg.ranges[i])
            # if(abs(tmp_radius)>0.5 and abs(tmp_radius)<0.9):    #old:0.55,0.85
            circle_distance.append(msg.ranges[i])
                #print(abs(tmp_radius))
            circle_angle.append(360*i/1440)
    circleScan.ranges = circle_distance
    circleScan.intensities = circle_angle
    pubScan.publish(circleScan)
    #for j in range(len(circle_distance)):
    #    print(j,circle_distance[j],circle_angle[j])

def listener():
    rospy.init_node("usb_cam", anonymous=True)
    zed_topic = "/usb_cam/image_raw"
    scan_topic = "/scan"

    rospy.Subscriber(zed_topic, Image, image_callback)
    rospy.Subscriber(scan_topic, LaserScan, scan_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()