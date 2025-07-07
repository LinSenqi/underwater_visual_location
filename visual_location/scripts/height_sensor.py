#! /usr/bin/env python3
# coding:UTF-8
import serial
import time
import threading

from serial import SerialException
import rospy

class HeightSensor:
    
    loop = 0
    connection = False
    
    def __init__(self, callback_method,port="/dev/ttyUSB0", baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=2):
        """
        初始化高度传感器
        :param port: 串口号
        :param baudrate: 波特率
        :param bytesize: 数据位
        :param parity: 校验位
        :param stopbits: 停止位
        :param timeout: 超时时间
        """
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.ser = None
        self.callback_method = callback_method
        self.height = None
    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=self.timeout
            )
            if self.ser.is_open:
               
                #print(f"成功打开串口 {self.ser.name}")
                self.connection = True

        except Exception as e:
            print(f"串口连接异常: {str(e)}")
            self.connection = False
    
    def read_distance(self):
        """
        读取距离数据
        :return: 距离值,如果读取失败返回None
        """
        while self.loop:
            
            if self.connection == True:
                
                try:
                
                    # 发送数据
                    
                    # print(f"已发送: {send_data}")

                    # 接收数据
                
                    recv_bytes = self.ser.readline()
                    
                    if recv_bytes:
                        response = recv_bytes.decode('latin-1').strip()
                        parts = response.split(',')
                        if len(parts) >= 5 and parts[0] == '$SDDBT' and parts[4] == 'M':
                            # 距离值在第4个字段 (索引为3)
                            # The distance value is in the 4th field (index 3)
                            output_str = parts[3]
                            try:
                                rospy.loginfo(1)
                                distance = float(output_str)
                                self.height = distance
                                self.callback_method(self)
                            except ValueError:
                                rospy.loginfo(2)
                                print(f" 无法将提取的值 '{output_str}' 转换为数值。")
                                self.height =  None
                                self.callback_method(self)
                        else:
                            rospy.loginfo(3)
                            print(" 接收到的数据格式不正确或不是$SDDBT语句 跳过解析。")
                            self.height =  None
                            self.callback_method(self)
                    else:
                        rospy.loginfo(4)
                        self.height =  None
                        self.callback_method(self)
                except Exception as e:
                    rospy.loginfo(5)
                    print(f"读取数据异常: {str(e)}")
                    self.height =  None
                    self.callback_method(self)
            else:
                rospy.loginfo('串口异常')
                
                time.sleep(5)
    
    
    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")
            
    def run(self):
        """持续读取串口数据并解析距离"""
        #print("启动连续测量中...")
        
        if self.loop == 0:
            self.loop = 1
           
            #print("开始读取距离（Ctrl+C 停止）：")
            send_data = "$1,21,1,LAKEAS*02FA\r\n"
            
            self.ser.write(send_data.encode('ascii'))
            time.sleep(0.5)
            self.t2 = threading.Thread(target=self.read_distance)
           
            self.t2.start()

# 使用示例




