"메인 입니다."
"상효 수정"

import cv2
from ultralytics import YOLO
from pymycobot.mycobot import MyCobot
import time
import numpy as np
import threading
# 빨간색 범위
# 수정된 빨간색 범위
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])

lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

# 초록색 범위
# 수정된 초록색 범위
lower_green = np.array([40, 100, 100])
upper_green = np.array([80, 255, 255])

#노란색 범위
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([30, 255, 255])


# MyCobot 객체 생성
mc = MyCobot('COM3', 115200)
mc.send_angles([0, 0, 0, 0, 0, 0], 30)
time.sleep(3)
mc.send_angles([20, 55, -45, 65, -90, 15], 40)
time.sleep(3)

# YOLO 모델 로드
model = YOLO('C:/Users/user/Desktop/final-dataset4/runs/detect/train4/weights/best.pt')

# 객체 탐지 및 로봇 팔 제어를 위한 함수 정의
def return_to_origin():
    time.sleep(1)
    mc.send_angles([0,0,0,0,0,0],40)
    time.sleep(3)
    mc.set_gripper_value(100,30)
    time.sleep(2)
    mc.send_angles([20, 55, -45, 65, -90, 15], 40)
    time.sleep(2)
    camera_thread.start_detection()
    


def arm_pick():
    print("움직입니다")
    mc.set_gripper_calibration()
    mc.set_gripper_mode(0)
    mc.init_eletric_gripper()
    time.sleep(1)
    mc.send_angles([20, 76, -45, 50, -90, 15], 40)  # 잡는 위치로 이동
    time.sleep(2) 
    mc.set_gripper_value(13, 30)
    time.sleep(3)
    return_to_origin()
   

# 카메라 스레드 클래스 정의
class CameraThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.cap = cv2.VideoCapture(1)
        self.running = False
        self.detect_objects = True  # 객체 탐지 여부를 나타내는 플래그
        
    def run(self):
        self.running = True
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # YOLO 모델을 사용하여 객체 탐지 수행
            results = model(frame, stream=True)
            # generator에서 반환된 결과를 리스트로 변환하여 처리
            results = list(results)
            for result in results:
                annotated_frame = result.plot()  # 결과를 시각화
                
                # 바운딩 박스에 있는 탐지된 객체의 클래스 ID를 확인하고, ID가 0이면 팔을 움직임
                for cls_id_tensor, conf_tensor in zip(result.boxes.cls, result.boxes.conf):
                    cls_id = cls_id_tensor.item()  # 텐서에서 값 추출
                    confidence = conf_tensor.item()  # 텐서에서 값 추출
                    print("Class ID:", cls_id, "Confidence :", confidence)
                    if confidence > 0.75 :
                        if cls_id == 0:
                            print("캡슐입니다")
                            camera_thread.stop_detection()  # 객체 탐지 중지
                            arm_pick()
                        elif cls_id == 1:
                            print("필 입니다")
                            camera_thread.stop_detection()  # 객체 탐지 중지
                            arm_pick()
                        elif cls_id == 2:
                            print("타블렛 입니다")
                            camera_thread.stop_detection()  # 객체 탐지 중지
                            arm_pick()
                            
                
                cv2.imshow('Webcam', annotated_frame)

            if cv2.waitKey(1) == ord('q'):
                self.stop()

    def stop(self):
        self.running = False
        self.cap.release()
        
    def stop_detection(self):
        self.detect_objects = False
        
    def start_detection(self):
        self.detect_objects = True

# 카메라 스레드 시작
camera_thread = CameraThread()
camera_thread.start()