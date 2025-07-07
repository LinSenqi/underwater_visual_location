#! /usr/bin/env python3
# coding:UTF-8

import os
import sys
path = os.path.abspath(".")
sys.path.insert(0,path + "/src/Visual_location/visual_location/scripts")

import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
import position
from std_msgs.msg import Float32

imu_data = [-500]*4
laser_data = -500
height_data = -500
data_pos = [[-500,-500,-500]]*10

def laser_callback(data):
    global laser_data
    laser_data = data.data
    
def height_callback(data):
    global height_data
    height_data = data.data

def imu_callback(data):
    imu_data[0]=data.x
    imu_data[1]=data.y
    imu_data[2]=data.z
    imu_data[3]=data.w

if __name__ == "__main__":
    rospy.init_node("pos_listener")
    
    #实例化 订阅者 对象
    laser_sub = rospy.Subscriber("cmd_laser",Float32,laser_callback,queue_size=10)
    height_sub = rospy.Subscriber("cmd_height",Float32,height_callback,queue_size=10)
    imu_sub = rospy.Subscriber("cmd_imu",Quaternion,imu_callback,queue_size=10)
    pos_pub = rospy.Publisher("cmd_pos",Point,queue_size=10)
    pos_msg = Point()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        i = imu_data
        h = height_data
        l = laser_data
        if h != -500 and l != -500 and i != [-500]*4:
            
            localizer = position.UnderwaterLocalizer(pool_dimensions=(8.0, 4.0, 5.0))
            pos = localizer.update_position(l, h, i[0:3])
            data_pos.pop(0)
            data_pos.append(pos)
            pos_msg.x = pos[0]
            pos_msg.y = pos[1]
            pos_msg.z = pos[2]
            pos_pub.publish(pos_msg)
            rospy.loginfo("%f,%f,%f",pos[0],pos[1],pos[2])
        else:
            pos_msg.x = -500
            pos_msg.y = -500
            pos_msg.z = -500
            data_pos.pop(0)
            data_pos.append(data_pos[8])
            pos_pub.publish(pos_msg)
            