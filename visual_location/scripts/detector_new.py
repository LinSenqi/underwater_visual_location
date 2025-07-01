#! /usr/bin/env python3
# coding:UTF-8

import cv2
import numpy as np
import math

class CamSet:
    def __init__(self):
        # 单目相机参数
        self.cameraMatrix = np.array([[1487.431388, 0.0, 1241.60040811],
                                      [0.0, 1478.4943856, 687.64527377],
                                      [0.0, 0.0, 1.0]])
        self.distCoeffs = np.array([-0.24776359, 0.19104283, 0.0029431, 0.00005472, -0.10815356])

    def CorrectImage(self, img):
        h, w = img.shape[:2]
        newCamMtx, _ = cv2.getOptimalNewCameraMatrix(self.cameraMatrix, self.distCoeffs, (w, h), 1, (w, h), 0)
        mapx, mapy = cv2.initUndistortRectifyMap(self.cameraMatrix, self.distCoeffs, None, newCamMtx, (w, h), cv2.CV_16SC2)
        return cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

def angle(p, q, r):
        v1 = q - p
        v2 = r - p
        cosang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        return np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))


def find_QRcode(src, cam):
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    gray = cv2.blur(gray, (3,3))
    gray = cv2.equalizeHist(gray)
    _, thresh = cv2.threshold(gray, 112, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 找到嵌套的方形Marker
    markers = []
    for i in range(len(contours)):
        if hierarchy[0][i][2] != -1:
            c1 = hierarchy[0][i][2]
            if hierarchy[0][c1][2] != -1:
                peri = cv2.arcLength(contours[i], True)
                approx = cv2.approxPolyDP(contours[i], 0.05 * peri, True)
                if len(approx) == 4:
                    M = cv2.moments(contours[i])
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        markers.append((cx, cy))
    if len(markers) < 3:
        #cv2.imshow('video', src)
        return None,None,None,None

    # 提取前三个marker点：P, A, B
    pts = np.array(markers[:3], dtype=np.float32)
    # 计算直角点P
    

    right_idx = None
    for i in range(3):
        j, k = [x for x in range(3) if x != i]
        if abs(angle(pts[i], pts[j], pts[k]) - 90) < 20:
            right_idx = i
            break
    if right_idx is None:
        errors = [abs(angle(pts[i], pts[(i+1)%3], pts[(i+2)%3]) - 90) for i in range(3)]
        right_idx = int(np.argmin(errors))
    P = pts[right_idx]
    idx = [i for i in range(3) if i != right_idx]
    A, B = pts[idx[0]], pts[idx[1]]

    # 计算第四点 D = A + B - P
    D = A + B - P
    # 构造图像平面四个角点排序
    quad = np.array([P, A, D, B], dtype=np.float32)
    s = quad.sum(axis=1)
    diff = np.diff(quad, axis=1).reshape(-1)
    ordered = np.array([
        quad[np.argmin(s)],    # top-left
        quad[np.argmin(diff)], # top-right
        quad[np.argmax(s)],    # bottom-right
        quad[np.argmax(diff)]  # bottom-left
    ], dtype=np.int32)

    # 绘制三点连线，线条加粗
    draw_pts = [tuple(P.astype(int)), tuple(A.astype(int)), tuple(B.astype(int))]
    for i in range(3):
        cv2.line(src, draw_pts[i], draw_pts[(i+1)%3], (0, 255, 0), 4)

    # 3D-2D 对应点：真实世界坐标(米)
    real_size = 0.0355  # QR实际尺寸9cm
    obj_pts = np.array([
        [-real_size, -real_size, 0],
        [real_size, -real_size, 0],
        [real_size, real_size, 0],
        [-real_size, real_size, 0]
    ], dtype=np.float32)
    img_pts = np.array(ordered, dtype=np.float32)

    # 求解PnP获得rvec, tvec
    success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, cam.cameraMatrix, cam.distCoeffs)
    if not success:
        #cv2.imshow('video', src)
        return None,None,None,None

    # 计算欧拉角: 绕X(pitch), Y(yaw), Z(roll)
    R, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    pitch = math.degrees(math.atan2(R[2,1], R[2,2]))
    yaw   = math.degrees(math.atan2(-R[2,0], sy))
    roll  = math.degrees(math.atan2(R[1,0], R[0,0]))

    # 距离为 tvec 的 Z 分量
    distance = tvec[2][0]
    return pitch,yaw,roll,distance
    # 显示距离与倾斜角度（opencv不支持Unicode度符号，改用小写字母o表示）
    '''
    text = f"Dist: {distance:.2f}m Pitch:{pitch:.1f}degree"
    pos = (max(draw_pts[0][0], 0), max(draw_pts[0][1] - 10, 20))
    cv2.putText(src, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

    cv2.imshow('video', src)
    '''