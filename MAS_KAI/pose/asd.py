import cv2
import mediapipe as mp
import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

cap = cv2.VideoCapture(0)

# Setup mediapipe instance
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    previous_y = None
    while cap.isOpened():
        ret, frame = cap.read()
        
        # Recolor image to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
      
        # Make detection
        results = pose.process(image)
    
        # Recolor back to BGR
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        try:
            landmarks = results.pose_landmarks.landmark
        
            # 엉덩이 찾기
            L_hip = landmarks[mp_pose.PoseLandmark.LEFT_HIP.value]
            R_hip = landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value]
            
            # 화면에 왼쪽 엉덩이 좌표 표시
            L_hip_text = "L: ({:.2f}, {:.2f})".format(L_hip.x, L_hip.y)
            L_hip_position = (int(L_hip.x * image.shape[1]), int(L_hip.y * image.shape[0]))
            cv2.putText(image, L_hip_text, L_hip_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            
            # 화면에 오른쪽 엉덩이 좌표 표시
            R_hip_text = "R: ({:.2f}, {:.2f})".format(R_hip.x, R_hip.y)
            R_hip_position = (int(R_hip.x * image.shape[1]), int(R_hip.y * image.shape[0]))
            cv2.putText(image, R_hip_text, R_hip_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            
            current_y = (L_hip.y + R_hip.y) / 2
            if previous_y is not None:
                if current_y > previous_y + 0.05:  # 임계값으로 0.05를 설정
                    print("낙상이 감지되었습니다!")
                    
            previous_y = current_y
            
        except Exception as e:
            print(e)

        # Render detections
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                   mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2), 
                                   mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2))
        
        cv2.imshow('Mediapipe Feed', image)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()