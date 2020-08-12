
# -*- coding: utf-8 -*
#!/usr/bin/env python

import rospy
import struct
import time
from sensor_msgs.msg import NavSatFix

from math import radians, cos, sin, asin, sqrt
from geopy.distance import geodesic
fe = open("GPS.txt", "w")

get_start = bool(0)

start_lat = 0.0
start_long = 0.0

def geodistance(lng1,lat1,lng2,lat2):
    lng1, lat1, lng2, lat2 = map(radians, [float(lng1), float(lat1), float(lng2), float(lat2)])
    dlon=lng2-lng1
    dlat=lat2-lat1
    a=sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2 
    distance=2*asin(sqrt(a))*6371*1000 
    distance=round(distance/1000,3)
    return distance

def callback_groundtruth(msg):
    global get_start,start_lat,start_long
    if not get_start:
        start_lat = msg.latitude
        start_long = msg.longitude
        get_start = bool(1)

    # rospy.loginfo(start_lat)
    # rospy.loginfo(start_long)
    #rospy.loginfo(str(p))
    if get_start:
        dlat = geodesic((float(start_lat),float(start_long)),(float(msg.latitude),float(start_long))).m
        dlon = geodesic((float(start_lat),float(start_long)),(float(start_lat),float(msg.longitude))).m
        rospy.loginfo(dlat)
        rospy.loginfo(dlon)
        fe.write(str(dlat) + "," + str(dlon) + '\n')    

    
def listener():
 
    rospy.init_node('dji_sdk', anonymous=True)
    gps_topic = "/dji_sdk/gps_position"
    
    rospy.Subscriber(gps_topic, NavSatFix, callback_groundtruth)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__ == '__main__':
    listener()

