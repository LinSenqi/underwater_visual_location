#! /usr/bin/env python3
# coding:UTF-8
import serial
import time
import threading

from serial import SerialException
import rospy
LIGHT_OPEN,MESURE_START,LIGHT_CLOSE,MESURE_STOP = 1,2,3,4

class LaserRangeFinder:
    FRAME_HEAD = bytes([0x55, 0x7A])  # 读取数据的响应帧头
    mesure = -1
    isopen = False
    loop = 0
    

    def __init__(self,callback_method, port: str, baudrate: int = 9600, dev_id: int = 0x01, timeout: float = 1.0):
        self.dev_id = dev_id
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = serial.Serial(self.port, self.baudrate,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=self.timeout)
        
        self.connection = False
        self.callback_method = callback_method
        

    def openDevice(self):
        """发送水下连续测量命令"""
        #self.closeDevice()
        self.connect()
        if (self.connection == True) :
            
            cmd = self._build_cmd(LIGHT_OPEN)  
            self.ser.write(cmd)
            cmd = self._build_cmd(MESURE_START)  
            self.ser.write(cmd)
            self.isopen == True
        else:
            print("连接未成功，设备无法打开")

    def _build_cmd(self, order) -> bytes:
        # 构造6字节命令：[0xAA,0x75, cmd_code, 0x01, dev_id, BCC]
        light_open = bytearray([0xAA, 0x75, 0x99, 0x01, self.dev_id])
        mesure_start = bytearray([0xAA, 0x75, 0x11, 0x01, self.dev_id])
        light_close = bytearray([0xAA, 0x75, 0xAA, 0x01, self.dev_id])
        mesure_stop = bytearray([0xAA, 0x75, 0x55, 0x01, self.dev_id])
        bcc = 0
        
        if order == 1 :
            frame = light_open
        elif order == 2 :
            frame = mesure_start
        elif order == 3:
            frame = light_close
        else:
            frame = mesure_stop
            
        for b in frame:
            bcc ^= b
        frame.append(bcc)
        return bytes(frame)

    def _parse_response(self, data: bytes):
        # 判断帧格式：[0x55, 0x7A, 0xAA, 0x03, ID, LO, HI, BCC]
        if len(data) != 8 or data[0:2] != self.FRAME_HEAD:
           
            return None
        # 校验 BCC
        bcc = 0
        for b in data[:7]:
            bcc ^= b
        if bcc != data[7] or data[2] != 0xAA:
            
            return None
            
        # 解析距离
        distance_mm = data[6] << 8 | data[5]
        
        return distance_mm / 1000.0  # 单位：米
    
    def get(self):
        
        if self.mesure == -1:
            
            return None
            
        else:
            return self.mesure
        
    def closeDevice(self):
        if self.connection == True:
            self.ser.close()
            print("端口关闭了")
        self.isOpen = False
        cmd = self._build_cmd(MESURE_STOP)  
        self.ser.write(cmd)
        #cmd = self._build_cmd(LIGHT_CLOSE) 
        #self.ser.write(cmd)
        print("设备关闭了")
        
    
    
    
    def connect(self) -> bool:
        """
        建立与激光测距仪的连接
        :return: 连接是否成功
        """
        try:
            if not self.ser.is_open:
                self.ser.open()
            print("连接成功")
            self.connection = True
            # 测试连接：查询设备ID
        except Exception as e:
            print(f"连接时发生错误：{str(e)}")
            self.connection = False
    
    def DataReceiveTH(self):
         while self.loop == 1:
            if  (self.connection == True) :
               
                try:
                    frame = self.ser.read(8)
                   
                    
                    if len(frame) == 8:
                        
                        distance = self._parse_response(frame)
                        
                                
                        self.mesure = distance
                        self.callback_method(self)
                            
                            #print(f"[{time.strftime('%H:%M:%S')}] 距离：{distance:.3f} m")
                    # time.sleep(0.5)
                except KeyboardInterrupt:
                    print("\n已手动停止程序")
                    break
                except Exception as e:
                    print("错误:", e)
                    
           
                
                
       


    def run(self):
        """持续读取串口数据并解析距离"""
        #print("启动连续测量中...")
        
        if self.loop == 0:
            self.loop = 1
            #print("开始读取距离（Ctrl+C 停止）：")
            self.t2 = threading.Thread(target=self.DataReceiveTH)
            self.t2.start()
            
    

'''
if __name__ == "__main__":
    # 修改为实际串口号
    lrf = LaserRangeFinder(port="COM19", baudrate=9600)
    lrf.connect()
    lrf.run()
'''