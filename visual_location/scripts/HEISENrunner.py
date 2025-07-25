
#! /usr/bin/env python3
# coding:UTF-8

import os
import sys
path = os.path.abspath(".")
sys.path.insert(0,path + "/src/Visual_location/visual_location/scripts")

import rospy

from std_msgs.msg import Float32
import height_sensor

data_height = [None]*10
Dis = None

def updateData(HeightSensor):
    global Dis
    rospy.loginfo("CallBack Method Running")
    Dis = HeightSensor.height

if __name__ == "__main__":    
    rospy.init_node("height_node")
    height_pub = rospy.Publisher("cmd_height",Float32,queue_size=10)
    height_msg = Float32()
    
    sensor = height_sensor.HeightSensor(updateData)
    sensor.connect()
    sensor.run()
    
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
       
        var = Dis
        if var != None:
            data_height.pop(0)
            data_height.append(var)
            height_msg = var
            height_pub.publish(var)
            rospy.loginfo("%f",var)
            
        else:
            
            data_height.pop(0)
            data_height.append(data_height[8])
            height_pub.publish(-500)
            rospy.loginfo("激光测距仪没有返回有价值的信息")
        
    
        rate.sleep()
       

    sensor.close()
    
    




