# -*- coding: utf-8 -*-


# from os import WCOREDUMP
# from typing import OrderedDict
import cv2
import numpy as np



# ---------------------------color---------------------------------------
# 设定yellow的阈值
lower_yellow = np.array([20, 120, 150])
upper_yellow = np.array([55, 180, 255])
# 设定red的阈值
lower_red = np.array([0, 200, 80])
upper_red = np.array([20, 255, 255])
# 设定white的阈值
# lower_white = np.array([40, 0, 140])
# upper_white = np.array([90, 30, 255])
# 设定orange的阈值
lower_orange = np.array([0, 150, 170])
upper_orange = np.array([25, 230, 255])
# 设定green的阈值 robust
lower_green = np.array([55, 120, 100])
upper_green = np.array([80, 200, 255])
# 设定blue的阈值 robust
lower_blue = np.array([80, 100, 80])
upper_blue = np.array([150, 255, 255])



def color_recog(frame):
    # 转换到HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.circle(frame, (40, 40), 10, color=(0, 0, 255))

    # 根据阈值构建掩模
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    # mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 膨胀腐蚀
    mask = cv2.dilate(mask, None, iterations=10)
    mask = cv2.erode(mask, None, iterations=10)

    # 对mask检测轮廓
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x,y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            #print('center:', x,y)

        return x,y
    else:
        return 0, (0,0)



if __name__ == '__main__':        

    cap = cv2.VideoCapture(0)

    while 1:
         # 获取每一帧
        ret, frame = cap.read()

        x,y = color_recog(frame)
        print('color:','blue')
        print('center:',x,y)




        # 显示图像
        cv2.imshow('frame', frame)
        # cv2.imshow('mask', mask)
        #  cv2.imshow('res',res)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    # 关闭窗口
    cv2.destroyAllWindows()
    
