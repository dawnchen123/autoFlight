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
# 更新MASK图像，并且刷新windows
def updateMask():
    global img
    global lowerb
    global upperb
    global mask
    # 计算MASK
    mask = cv2.inRange(img_hsv, lowerb, upperb)

    cv2.imshow('mask', mask)

# 更新阈值
def updateThreshold(x):

    global lowerb
    global upperb

    minH = cv2.getTrackbarPos('minH','image')
    maxH = cv2.getTrackbarPos('maxH','image')
    minS = cv2.getTrackbarPos('minS','image')
    maxS = cv2.getTrackbarPos('maxS', 'image')
    minV = cv2.getTrackbarPos('minV', 'image')
    maxV = cv2.getTrackbarPos('maxV', 'image')
    
    lowerb = np.int32([minH, minS, minV])
    upperb = np.int32([maxH, maxS, maxV])
    
    print('更新阈值')
    print(lowerb)
    print(upperb)
    updateMask()

def detectCircle():
    '''
    色块识别测试样例2 从视频流中读取并且识别
    '''
    # 视频路径
    #video_path = 'demo-video.mkv'
    # 颜色阈值下界(HSV) lower boudnary
    lowerb = (0, 24, 121) 
    # 颜色阈值上界(HSV) upper boundary
    upperb = (255, 255, 255)

    pub = rospy.Publisher('/circle/position', Image, queue_size=1)
    pub_Img = rospy.Publisher('circle/image', Image, queue_size=1) 
    rospy.init_node('detectCircle', anonymous=True)
    rate = rospy.Rate(30)
    bridge = CvBridge()
    # 读入视频流
    cap = cv2.VideoCapture(0)

    circleCenter = np.ones((3,1),dtype=np.uint8)
  
    while(True):
        # 逐帧获取画面
        # ret ？ 画面是否获取成功
        ret, frame = cap.read()
        
        if ret:
            img = frame
            # 识别色块 获取矩形区域数组
            # 同时设定最小高度还有宽度，过滤噪声
            img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(img_hsv, lowerb, upperb)
            #cv2.imshow('mask', mask)
            kernel = np.ones((10,10),np.uint8)
            dilation = cv2.dilate(mask,kernel,iterations = 1)
            gauss_img = cv2.GaussianBlur(dilation,(5,5),0)
            canny_img = cv2.Canny(gauss_img, 50, 150)
            contours = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[1]  
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
                        # M=cv2.moments(contours[i])                    
                        # cx=int(M['m10']/M['m00'])
                        # cy=int(M['m01']/M['m00'])
                        # print(cv2.contourArea(contours[i]),cx,cy)
                        circleCenter[0,0]=int(center[0]/2)
                        circleCenter[1,0]=int(center[1]/2)
                        circleCenter[2,0]=int(radius)
                        pub.publish(bridge.cv2_to_imgmsg(circleCenter, "mono8"))
            img2 = cv2.resize(img,(320,180))
            pub_Img.publish(bridge.cv2_to_imgmsg(img2, '8UC3'))
            # cv2.imshow('img',img)
            #cv2.imshow('canny',canny_img)

        else:
            print("视频读取完毕或者视频路径异常")
            break

        # 这里做一下适当的延迟，每帧延时0.1s钟
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    #cv2.destroyAllWindows()


def main(img):
    global img_hsv
    global upperb
    global lowerb
    global mask
    # 将图片转换为HSV格式
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 颜色阈值 Upper
    upperb = None
    # 颜色阈值 Lower
    lowerb = None

    mask = None

    cv2.namedWindow('image', flags= cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
    # cv2.namedWindow('image')
    cv2.imshow('image', img)

    # cv2.namedWindow('mask')
    cv2.namedWindow('mask', flags= cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)

    # 红色阈值 Bar
    ## 红色阈值下界
    cv2.createTrackbar('minH','image',0,255,updateThreshold)
    ## 红色阈值上界
    cv2.createTrackbar('maxH','image',0,255,updateThreshold)
    ## 设定红色阈值上界滑条的值为255
    cv2.setTrackbarPos('maxH', 'image', 255)
    cv2.setTrackbarPos('minH', 'image', 0)
    # 绿色阈值 Bar
    cv2.createTrackbar('minS','image',0,255,updateThreshold)
    cv2.createTrackbar('maxS','image',0,255,updateThreshold)
    cv2.setTrackbarPos('maxS', 'image', 255)
    cv2.setTrackbarPos('minS', 'image', 0)
    # 蓝色阈值 Bar
    cv2.createTrackbar('minV','image',0,255,updateThreshold)
    cv2.createTrackbar('maxV','image',0,255,updateThreshold)
    cv2.setTrackbarPos('maxV', 'image', 255)
    cv2.setTrackbarPos('minV', 'image', 0)

    # 首次初始化窗口的色块
    # 后面的更新 都是由getTrackbarPos产生变化而触发
    updateThreshold(None)

    print("调试棋子的颜色阈值, 键盘摁e退出程序")
    while cv2.waitKey(0) != ord('e'):
        continue

    cv2.imwrite('tmp_bin.png', mask)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detectCircle()
