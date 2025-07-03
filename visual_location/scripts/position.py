#! /usr/bin/env python3
# coding:UTF-8

import numpy as np
import math



class UnderwaterLocalizer:
    def __init__(self, pool_dimensions=(8.0, 4.0, 5.0),initial_yaw = 0.0):
        """
        初始化水下定位系统
        :param pool_dimensions: 水池尺寸 (长, 宽, 高) 单位米
        """
        self.L, self.W, self.H = pool_dimensions  # 水池尺寸
        self.l = self.L / 2.0  # 半长
        self.w = self.W / 2.0  # 半宽
        self.initial_yaw = initial_yaw
        # 前一时刻的状态 (用于连续定位)
        self.previous_position = None
        self.previous_attitude = None
        self.previous_alpha = None
        self.previous_region = None
        
    def angle2radian(angle):
        
        return angle*np.pi/180
    
    def RegularYaw(self,angle):
        
        # 将航向角归一化到[-π, π]范围
        
        # 计算最接近的整数倍数
        two_pi = 2 * math.pi
        angle_prime = angle - self.initial_yaw
        n = round(angle_prime/ two_pi)
        normalized = angle_prime - n * two_pi
        
        # 处理浮点精度问题（确保结果严格在 [-π, π] 内）
        if normalized < -math.pi:
            normalized += two_pi
        elif normalized > math.pi:
            normalized -= two_pi
            
        return normalized 
            
    
    def get_beta(self, psi):
        """
        根据航向角ψ确定旋转角度β
        :param psi: 航向角 (弧度)
        :return: 旋转角度β
        """
        # 将航向角归一化到[-π, π]范围
        psir =self.RegularYaw(psi)
                
        # 确定航向角所在的区间
        if -np.pi <= psir < -np.pi/2:
            return -np.pi
        elif -np.pi/2 <= psir < 0:
            return -np.pi/2
        elif 0 <= psir < np.pi/2:
            return 0
        elif np.pi/2 <= psir <= np.pi:
            return np.pi/2
        else:
            raise ValueError(f"Invalid psi value: {psir}")
    
    def TransformedSize(self,psi):
        #我需要更新新坐标系下水池的半长和半宽
        beta = self.get_beta(psi)
        if beta == -np.pi/2 or beta == np.pi/2: 
            length = self.w
            width = self.l
            return length,width
        else:
            length = self.l
            width = self.w
            return length,width
            
    def get_alpha(self,psi):
        psir =self.RegularYaw(psi)
        return psir - self.get_beta(psi)
        

    def transform_coordinates(self, position, psi):
        """
        坐标系变换
        :param position: 原始坐标系中的位置 [x, y, z]
        :param beta: 旋转角度
        :return: 新坐标系中的位置 [x', y', z']
        """
        x, y, z = position
        beta = self.get_beta(psi)
        cos_b = math.cos(beta)
        sin_b = math.sin(beta)
        
        # 绕Z轴旋转
        x_prime = x * cos_b + y * sin_b
        y_prime = -x * sin_b + y * cos_b
        return np.array([x_prime, y_prime, z])

    def inverse_transform(self, position_prime, beta):
        """
        逆坐标系变换
        :param position_prime: 新坐标系中的位置 [x', y', z']
        :param beta: 旋转角度
        :return: 原始坐标系中的位置 [x, y, z]
        """
        xp, yp, zp = position_prime
        
        cos_b = math.cos(beta)
        sin_b = math.sin(beta)
        
        # 绕Z轴逆旋转
        x = xp * cos_b - yp * sin_b
        y = xp * sin_b + yp * cos_b
        return np.array([x, y, zp])

    def get_direction_vectors(self, attitude):
        """
        计算高度计的方向向量
        :param attitude: 姿态角 (roll, pitch, yaw) 弧度
        :return: (前高度计方向向量, 侧高度计方向向量)
        """
        phi, theta, psi = attitude  
        
        # 前高度计方向向量
        s_f = np.array([
            math.cos(psi) * math.cos(theta),
            math.sin(psi) * math.cos(theta),
            -math.sin(theta)
        ])
        
        # 侧激光测距仪方向向量
        s_s = np.array([
            -math.sin(psi) * math.cos(phi) + math.cos(psi) * math.sin(theta) * math.sin(phi),
            math.cos(psi) * math.cos(phi) + math.sin(psi) * math.sin(theta) * math.sin(phi),
            math.cos(theta) * math.sin(phi)
        ])
        
        return s_f, s_s

    def determine_region(self, position_prime, psi,alpha):
        """
        :param position_prime: 新坐标系中的位置
        :param alpha: 角度α = ψ - β
        :return: 区域编号 (1-4)
        """
       
        xp, yp, _ = position_prime
        
        
        l_rot,w_rot = self.TransformedSize(psi)
        x_RB, y_RB = l_rot,w_rot
        x_BL, y_BL = -l_rot, w_rot
        
        slope1 = (yp - y_RB) / (xp - x_RB) if (xp - x_RB) != 0 else float('inf')
        slope2 = (yp - y_BL) / (xp - x_BL) if (xp - x_BL) != 0 else float('inf')#这个是无穷大的表示方法
        
        tan_alpha = math.tan(alpha)
        inv_tan_alpha = 1.0 / tan_alpha if tan_alpha != 0 else float('inf')
        

        if slope1 >= tan_alpha and slope2 >= -inv_tan_alpha:
            return 1
        elif slope1 >= tan_alpha and slope2 < -inv_tan_alpha:
            return 2
        elif slope1 < tan_alpha and slope2 < -inv_tan_alpha:
            return 3
        elif slope1 < tan_alpha and slope2 >= -inv_tan_alpha:
            return 4
        else:
            # 边界情况待处理
            return 1

    def calculate_position(self, d_f, d_s, attitude, region):
        """
        :param d_f: 前高度计测量距离
        :param d_s: 侧高度计测量距离
        :param attitude: 姿态角 (roll, pitch, yaw)
        :param region: 当前区域
        :param alpha: 角度α = ψ - β
        :param beta: 旋转角度β
        :return: 新坐标系中的位置 [x', y', z']
        """
        phi, theta, psi = attitude
        beta = self.get_beta(psi)
        alpha = self.get_alpha(psi)
        l_rot,w_rot = self.TransformedSize(psi)
        s_f, s_s = self.get_direction_vectors(attitude)
        
        # 根据区域选择计算方式
        if region == 1:
            # 区域1
            n_R = np.array([1, 0, 0]) 
            n_B = np.array([0, 1, 0])  
            
            # 计算有效距离
            d_f_prime = d_f * abs(np.dot(s_f, n_R))
            d_s_prime = d_s * abs(np.dot(s_s, n_B))
            
            # 计算位置
            x_prime = self.l - d_f_prime * math.cos(alpha)
            y_prime = d_s_prime * math.cos(alpha) - w_rot
            return np.array([x_prime, y_prime, 0])
        
        elif region == 2:
            # 区域2
            n_R = np.array([1, 0, 0])  
            n_L = np.array([0, -1, 0])
            
            d_f_prime = d_f * abs(np.dot(s_f, n_R))
            d_s_prime = d_s * abs(np.dot(s_s, n_L))
            
           
            if self.previous_position is None:
                # 初始位置处理
                x_prime = l_rot - d_f_prime * math.cos(alpha)
                y_prime = d_s_prime * math.cos(alpha) - w_rot
            else:
                x_prev, y_prev, _ = self.previous_position
                x_prime = l_rot - d_f_prime * math.cos(alpha)
                y_prime = y_prev + (x_prime - x_prev) * math.tan(self.previous_alpha)
            
            return np.array([x_prime, y_prime, 0])
        
        elif region == 3:
            # 区域3
            n_B = np.array([0, 1, 0])  # B面法向量
            n_L = np.array([0, -1, 0])  # L面法向量
            
            d_f_prime = d_f * abs(np.dot(s_f, n_B))
            d_s_prime = d_s * abs(np.dot(s_s, n_L))
            
            # 计算位置 (论文公式13)
            x_prime = -l_rot + d_f_prime * math.sin(alpha)
            y_prime = w_rot - d_f_prime * math.sin(alpha)
            return np.array([x_prime, y_prime, 0])
        
        elif region == 4:
            # 区域4
            n_B = np.array([0, 1, 0])  # B面法向量
            
            d_f_prime = d_f * abs(np.dot(s_f, n_B))
            d_s_prime = d_s * abs(np.dot(s_s, n_B))
            
            # 使用前一时刻的位置信息 (论文公式16)
            if self.previous_position is None:
                # 初始位置处理
                y_prime = w_rot - d_f_prime * math.sin(alpha)
                x_prime = -l_rot+ d_s_prime * math.cos(alpha)
            else:
                x_prev, y_prev, _ = self.previous_position
                y_prime = w_rot - d_f_prime * math.sin(alpha)
                x_prime = x_prev + (y_prime - y_prev) / math.tan(self.previous_alpha)
            
            return np.array([x_prime, y_prime, 0])
        
        else:
            raise ValueError(f"Invalid region: {region}")
            

    def update_position(self, d_f, d_s, attitude):
        """
        更新UWV的位置估计
        :param d_f: 前高度计测量距离 (米)
        :param d_s: 侧高度计测量距离 (米)
        :param attitude: 姿态角 (roll, pitch, yaw) 弧度
        :return: 原始坐标系中的位置 [x, y, z]
        """
        # 获取航向角
        _, _, psi = attitude
        
        # 步骤1: 确定旋转角度β
        beta = self.get_beta(psi)
        
        # 步骤2: 计算角度α
        alpha = self.get_alpha(psi)
        
        # 步骤3: 确定当前区域
        if self.previous_position is None:
            # 初始位置处理 - 假设从区域1开始
            region = 1
        else:
            # 使用前一时刻的位置确定区域
            region = self.determine_region(self.previous_position, psi,alpha)
        
        # 步骤4: 在新坐标系中计算位置
        position_prime = self.calculate_position(d_f, d_s, attitude, region)
        
        # 步骤5: 转换回原始坐标系
        position = self.inverse_transform(position_prime, beta)
        
        # 存储当前状态用于下一时刻计算
        self.previous_position = position_prime
        self.previous_attitude = attitude
        self.previous_alpha = alpha
        self.previous_region = region
        
        return position


