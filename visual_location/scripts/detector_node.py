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

data_pitch = [None]*10
data_yaw = [None]*10
data_roll = [None]*10
data_distance =[None]*10

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
            
            posdata_msg.x = p
            posdata_msg.y = y
            posdata_msg.z = r
            posdata_msg.w = dis
            posdata_pub.publish(posdata_msg)
            rospy.loginfo("%f,%f,%f,%f",p,y,r,dis)

            
        else:
            data_pitch.pop(0)
            data_pitch.append(data_pitch[8])            
            data_yaw.pop(0)
            data_yaw.append(data_yaw[8]) 
                
            data_roll.pop(0)
            data_roll.append(data_roll[8]) 
            
            data_distance.pop(0)
            data_distance.append(data_distance[8]) 
        
            posdata_msg.x = -500
            posdata_msg.y = -500
            posdata_msg.z = -500
            posdata_msg.w = -500
            posdata_pub.publish(posdata_msg)
            rospy.loginfo("没有检测到二维码")
        
        cv2.imshow('video', frame)
        cv2.waitKey(1)
       
    except Exception as e:
        rospy.logerr(f"Image conversion error: {e}")
        return

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