#! /usr/bin/env python3
# coding:UTF-8

import os
import sys
path = os.path.abspath(".")
sys.path.insert(0,path + "/src/Visual_location/visual_location/scripts")
import rospy
import time


from geometry_msgs.msg import Quaternion
import device_model
import serial

data_time = [None] * 10
data_AngX = [None] * 10
data_AngY = [None] * 10
data_AngZ = [None] * 10
data_AsX = [None] * 10
ang_x = None
ang_y = None
ang_z = None
as_x = None

def updateData(DeviceModel):
    # 获取当前时间戳
    global data_AngX,data_AngY,data_AngZ,data_AsX,data_time
    global ang_x,ang_y,ang_z,as_x
    current_time = time.time()
   
    # 获取设备的数据
    ang_x = DeviceModel.get(DeviceModel.addrLis[0], "AngX")
    ang_y = DeviceModel.get(DeviceModel.addrLis[0], "AngY")
    ang_z = DeviceModel.get(DeviceModel.addrLis[0], "AngZ")
    as_x = DeviceModel.get(DeviceModel.addrLis[0], "AsX")
    

if __name__ == "__main__":
    # 读取的modbus地址列表 List of Modbus addresses read
    
    rospy.init_node("imu_node")
    imu_pub = rospy.Publisher("cmd_imu",Quaternion,queue_size=10)
    imu_msg = Quaternion()
    addrLis = [0x50]
    
    # 拿到设备模型 Get the device model
    device = device_model.DeviceModel("测试设备1", "/dev/ttyUSB0", 9600, addrLis, updateData, 30)
    
    
    # 开启设备 Turn on the device
    device.openDevice()
    device.startLoopRead()
    # 开启轮询 Enable loop reading
  
    #rospy.loginfo("%f,%f,%f,%f",imu_msg.x,imu_msg.y,imu_msg.z,imu_msg.w)
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        ax,ay,az,asx = ang_x,ang_y,ang_z,as_x
        if  (ax != None) and  (ay != None) and (az != None) and (asx != None):
            data_AngX.pop(0)
            data_AngX.append(ax)
            
            data_AngY.pop(0)
            data_AngY.append(ay)
                
            data_AngZ.pop(0)
            data_AngZ.append(az)
            
            data_AsX.pop(0)
            data_AsX.append(asx)

            imu_msg.x = ax
            imu_msg.y = ay
            imu_msg.z = az
            imu_msg.w = asx
        
            imu_pub.publish(imu_msg)
            rospy.loginfo("%f,%f,%f,%f",imu_msg.x,imu_msg.y,imu_msg.z,imu_msg.w)
        
        else:
            data_AngX.pop(0)
            data_AngX.append(data_AngX[8])
            
            data_AngY.pop(0)
            data_AngY.append(data_AngY[8])
                
            data_AngZ.pop(0)
            data_AngZ.append(data_AngZ[8])
            
            data_AsX.pop(0)
            data_AsX.append(data_AsX[8])
            
            imu_msg.x = -500
            imu_msg.y = -500
            imu_msg.z = -500
            imu_msg.w = -500
            imu_pub.publish(imu_msg)
            rospy.loginfo("初始化没有完成")
            device.serialPort = serial.Serial(device.serialConfig.portName, device.serialConfig.baud, timeout=0.5)
        
            
        rate.sleep()
        
    
    
    # 等待一定时间，进行数据记录
    #time.sleep(30)
    
    # 停止轮询 Stop reading
    device.stopLoopRead()
    
    # 等待一段时间再关闭设备 Close the device
    time.sleep(1)
    device.closeDevice()
    
    # 绘制记录的数据 Plot the data
    #plot_data()