from ultralytics import YOLO
import cv2

# YOLO 모델 로드
model = YOLO('yolov8n-pose.pt')

# 웹캠 피드 캡처 설정
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# 비디오 스트림 루프
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break
    
    # YOLO 모델로 프레임 처리
    results = model(frame, conf=0.6)
    
    # 결과 시각화
    annotated_frame = results[0].plot()  # 결과를 프레임에 그립니다.

    # 키포인트 인덱스 확인
    for result in results:
        if hasattr(result, 'keypoints'):
            keypoints = result.keypoints
            if keypoints is not None:
                for kp_index, keypoint in enumerate(keypoints):
                    print(f"Keypoint {kp_index}: {keypoint}")
    
    # 화면에 표시
    cv2.imshow("YOLOv8 Live Detection", annotated_frame)
    
    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()
