#! /usr/bin/env python3
# coding:UTF-8

import os
import sys
path = os.path.abspath(".")
sys.path.insert(0,path + "/src/Visual_location/visual_location/scripts")

import rospy
import time
import csv
import laser_sensor
from std_msgs.msg import Float32

data_distance = [None]*10
Dis = None

def updateData(LaserRangeFinder):
    global Dis
    
    Dis = LaserRangeFinder.get()
        

if __name__ == "__main__":
    
    rospy.init_node("laser_node")
    laser_pub = rospy.Publisher("cmd_laser",Float32,queue_size=10)
    laser_msg = Float32()
    lrf = laser_sensor.LaserRangeFinder(callback_method=updateData,port="/dev/ttyUSB0", baudrate=9600)
    lrf.openDevice()
    lrf.run()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        d= Dis
        if d != None:
            data_distance.pop(0)
            data_distance.append(d)
            laser_msg = data_distance[9]
            laser_pub.publish(laser_msg)
            rospy.loginfo("%f",d)
        else:
            data_distance.pop(0)
            data_distance.append(data_distance[8])
            laser_msg = -500
            laser_pub.publish(laser_msg)
            rospy.loginfo("激光测距仪没有返回有价值的信息")
        
        rate.sleep()
       

    lrf.closeDevice()