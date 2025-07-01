#! /usr/bin/env python3
# coding:UTF-8

import os
import sys
path = os.path.abspath(".")
sys.path.insert(0,path + "/src/Visual_location/visual_location/scripts")

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import detector_new
from geometry_msgs.msg import Quaternion

data_pitch = [0,0,0,0,0,0,0,0,0,0]
data_yaw = [0,0,0,0,0,0,0,0,0,0]
data_roll = [0,0,0,0,0,0,0,0,0,0]
data_distance =[0,0,0,0,0,0,0,0,0,0]

def image_callback(msg):
    global p,y,r,dis
    global data_pitch,data_yaw,data_roll,data_distance
    try:
        # 将ROS图像消息转换为OpenCV格式
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        #undist = cam.CorrectImage(frame)
       
        p,y,r,dis = detector_new.find_QRcode(frame, cam)
        if (p!=None) and (y!=None) and (r!=None) and (dis != None):
            data_pitch.pop(0)
            data_pitch.append(p)
            
            data_yaw.pop(0)
            data_yaw.append(y)
                
            data_roll.pop(0)
            data_roll.append(r)
            
            data_distance.pop(0)
            data_distance.append(dis)
           
            

            
        else:
            data_pitch.pop(0)
            data_pitch.append(data_pitch[8])
            
            data_yaw.pop(0)
            data_yaw.append(data_yaw[8])
                
            data_roll.pop(0)
            data_roll.append(data_roll[8])
            
            data_distance.pop(0)
            data_distance.append(data_distance[8])
        
        posdata_msg.x = data_pitch[9]
        posdata_msg.y = data_yaw[9]
        posdata_msg.z = data_roll[9]
        posdata_msg.w = data_distance[9]
        posdata_pub.publish(posdata_msg)
        
        cv2.imshow('video', frame)
        cv2.waitKey(1)
        rospy.loginfo("%f,%f,%f,%f",data_pitch[9],data_yaw[9],data_roll[9],data_distance[9])
    except Exception as e:
        rospy.logerr(f"Image conversion error: {e}")
        return
    
    # 显示图像
    #cv2.imshow("Camera View", cv_image)
    #cv2.waitKey(1)  # 必要的窗口更新操作

if __name__ == '__main__':
    # 初始化ROS节点
    cam = detector_new.CamSet()
    rospy.init_node('image_node')
    
    posdata_pub = rospy.Publisher("pos_data",Quaternion,queue_size=10)
    posdata_msg = Quaternion()
    
    # 创建OpenCV转换器
    bridge = CvBridge()
    
    # 创建图像订阅者
    rospy.Subscriber("camera", Image, image_callback, queue_size=10)
    
    
   
    try:
        # 保持节点运行
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 关闭所有OpenCV窗口
        cv2.destroyAllWindows()