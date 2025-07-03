#! /usr/bin/env python3
# coding:UTF-8


import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    rospy.init_node('image_publisher')
    
    # 创建OpenCV摄像头捕获对象
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        rospy.logerr("Failed to open camera with index 0!")
        rospy.signal_shutdown("Camera error")
        return
    
    # 创建ROS图像发布器和转换器
    image_pub = rospy.Publisher('camera', Image, queue_size=1)
    bridge = CvBridge()
    
    rate = rospy.Rate(30)  # 30Hz发布频率
    
    rospy.loginfo("Publishing camera images...")
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if not ret:
            rospy.logerr("Failed to capture frame!")
            break
        
        try:
            # 将OpenCV图像转换为ROS消息并发布
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image.header.stamp = rospy.Time.now()
            image_pub.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"Image conversion error: {str(e)}")
        
        # 添加短暂延迟并处理ROS事件
        rate.sleep()
    
    # 释放资源
    cap.release()
    rospy.loginfo("Camera released, node shutdown.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass