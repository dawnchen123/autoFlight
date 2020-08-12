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


def detectCircle():

    pub_Img = rospy.Publisher('circle/image', Image, queue_size=1) 
    rospy.init_node('detectCircle', anonymous=True)
    rate = rospy.Rate(30)
    bridge = CvBridge()
    # 读入视频流
    cap = cv2.VideoCapture(0)
  
    while(True):
        # 逐帧获取画面
        # ret ？ 画面是否获取成功
        ret, frame = cap.read()
        
        if ret:
            img = frame
            # img2 = cv2.resize(img,(160,120))
            pub_Img.publish(bridge.cv2_to_imgmsg(img, '8UC3'))
            # cv2.imshow('img',img)
            #cv2.imshow('canny',canny_img)

        else:
            print("视频读取完毕或者视频路径异常")
            break

        # 这里做一下适当的延迟，每帧延时0.1s钟
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    #cv2.destroyAllWindows()


if __name__ == "__main__":
    detectCircle()
