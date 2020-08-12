#!/usr/bin/env python
import rospy
import struct
import time
from nav_msgs.msg import Odometry


fo = open("vio.txt", "w") 

def callback_position(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    rospy.loginfo(str(x) + "," + str(y)+ "," + str(z))
    fo.write(str(x) + "," + str(y)+ "," + str(z)+ '\n')
    

    
def listener():
 
    rospy.init_node('realsense2_camera', anonymous=True)
    rtab_topic = "/camera/odom/sample"
    
    rospy.Subscriber(rtab_topic, Odometry, callback_position)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__ == '__main__':
    listener()

