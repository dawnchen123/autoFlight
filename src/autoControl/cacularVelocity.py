#!/usr/bin/env python
import rospy
import struct
import time
import math
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray


global planner_points

planner_list = []

odom_x = 0
odom_y = 0
odom_z = 0
planner_points = Marker()
start_t = 0


rospy.init_node('vins_estimator', anonymous=True)
pub_currentVelocity = rospy.Publisher('/control/velocity', Float32MultiArray, queue_size=1)

def callback_position(msg):
    global odom_x
    global odom_y
    global odom_z
    global start_t
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y
    odom_z = msg.pose.pose.position.z
    start_t = time.time()




def callback_planning(msg):
    global planner_list
    
    for i in range(1,len(msg.points),10):
        planner_points.points.append(msg.points[i])
        planner_list.append([msg.points[i].x, msg.points[i].y, msg.points[i].z])

  

    
def listener():
 
    vins_topic = "/vins_estimator/odometry"
    plan_topic = "/planning_vis/trajectory"

    
    rospy.Subscriber(vins_topic, Odometry, callback_position)
    rospy.Subscriber(plan_topic, Marker, callback_planning)

 
if __name__ == '__main__':
    control_velocity = Float32MultiArray()


    listener()
    start_t = time.time()

    while(True):
        stop_t = time.time()
        time.sleep(0.1)
        if(stop_t - start_t < 0.5 and len(planner_list) != 0):
            cur_distance=0


            dx = planner_list[0][0] + odom_y     # odom_y: backward -> positive
            dy = planner_list[0][1] - odom_x
            dz = planner_list[0][2] - odom_z
            cur_distance = math.sqrt(dx*dx+dy*dy+dz*dz)
            

            if cur_distance > 0.5:
                control_velocity.data=[dx/cur_distance,dy/cur_distance,dz/cur_distance]

                rospy.loginfo(str(len(planner_list))+"\t"+str(planner_list[0]))
            else:
                planner_list.remove(planner_list[0])
                control_velocity.data=[0,0,0]

            # pub_currentVelocity.publish(control_velocity)
                
        else:
            control_velocity.data=[0,0,0]
        pub_currentVelocity.publish(control_velocity)

        control_velocity.data=[0,0,0]


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# def callback_position(msg):
#     global planner_list
#     cur_distance=0
#     control_velocity = Float32MultiArray()

#     odom_x = msg.pose.pose.position.x
#     odom_y = msg.pose.pose.position.y
#     odom_z = msg.pose.pose.position.z
#     for i in range(len(planner_points.points)):
#         dx= planner_points.points[i].x-odom_x
#         dy= planner_points.points[i].y-odom_y
#         dz= planner_points.points[i].z-odom_z
#         cur_distance = math.sqrt(dx*dx+dy*dy+dz*dz)
#         send_velocity = bool(0)
#         if cur_distance > 0 and not send_velocity:
#             if not send_velocity:
#                 control_velocity.data=[dx/cur_distance,dy/cur_distance,dz/cur_distance]
#                 pub_currentVelocity.publish(control_velocity)
#                 control_velocity.data=[0,0,0]
#                 send_velocity = bool(1)

#         if cur_distance<0.1:
#             send_velocity = bool(0)
#     rospy.loginfo("ffffffffffffffffffffffffff")
#     # rospy.loginfo(str(x) + "," + str(y)+ "," + str(z))
#     # fo.write(str(x) + "," + str(y)+ "," + str(z)+ '\n')
