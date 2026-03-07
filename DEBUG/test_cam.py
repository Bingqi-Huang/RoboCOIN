import cv2
import time

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# 关键：必须在设置分辨率之前，强制指定 FOURCC 为 MJPG
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# 关键：放弃 10 FPS，使用固件最稳定的 30 FPS
cap.set(cv2.CAP_PROP_FPS, 30) 

time.sleep(1) # 给予固件响应和初始化曝光的时间

ret, frame = cap.read()
if ret:
    print("成功读取图像，尺寸:", frame.shape)
else:
    print("读取失败")

cap.release()