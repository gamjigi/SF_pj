import cv2
import mediapipe as mp
import numpy as np
import socket
import json
import requests

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Kakao API 관련 함수들
def save_tokens(tokens):
    with open('kakao_tokens.json', 'w') as f:
        json.dump(tokens, f)

def load_tokens():
    try:
        with open('kakao_tokens.json', 'r') as f:
            tokens = json.load(f)
        return tokens
    except FileNotFoundError:
        print("Token file not found. Please obtain new tokens.")
        return None

def get_kakao_access_token(client_id, redirect_uri, authorization_code):
    url = 'https://kauth.kakao.com/oauth/token'
    data = {
        'grant_type': 'authorization_code',
        'client_id': client_id,
        'redirect_uri': redirect_uri,
        'code': authorization_code,
    }
    response = requests.post(url, data=data)

    if response.status_code != 200:
        print("Failed to get access token:", response.json())
        return None

    tokens = response.json()
    if 'access_token' not in tokens:
        print("Access token not found in response:", tokens)
        return None

    return tokens

def send_kakao_message(access_token, message):
    url = 'https://kapi.kakao.com/v2/api/talk/memo/default/send'
    headers = {
        'Authorization': f'Bearer {access_token}'
    }
    data = {
        'template_object': json.dumps({
            'object_type': 'text',
            'text': message,
            'link': {
                'web_url': 'https://developers.kakao.com',
                'mobile_web_url': 'https://developers.kakao.com'
            }
        })
    }
    response = requests.post(url, headers=headers, data=data)
    print("Kakao API Response Status Code:", response.status_code)
    print("Kakao API Response Text:", response.text)
    if response.status_code != 200:
        print("Failed to send Kakao message:", response.json())
    return response.json()

# 클라이언트 ID, 리다이렉트 URI, 인증 코드 설정
client_id = '5fc2fec9da03b82ba3400d272332b928'
redirect_uri = 'https://localhost:5000'
authorization_code = '0IQy2PS_K82sfN6ZXdljF79bSibIY7nd-UmgOTClJJ4xqwovn0XpUgAAAAQKKiUQAAABkAEnSFQq3eF1vjqPRg'


#여기 페이지 들어가서 토큰(authorization_code -> 여기에 넣으면 됨) 받아오기
#https://kauth.kakao.com/oauth/authorize?client_id=5fc2fec9da03b82ba3400d272332b928&redirect_uri=https://localhost:5000&response_type=code


# 처음 토큰을 받고 저장
tokens = get_kakao_access_token(client_id, redirect_uri, authorization_code)
if tokens:
    save_tokens(tokens)

# 저장된 토큰을 로드
tokens = load_tokens()
if tokens:
    access_token = tokens['access_token']
else:
    print("Failed to retrieve access token.")
    access_token = None

cap = cv2.VideoCapture(0)   #0번 적외선 1번 웹캠

# MediaPipe Pose 설정
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    previous_y = None

    # 소켓 설정
    HOST = '172.30.1.30'  # 서버의 IP 주소
    PORT = 12345  # 사용할 포트 번호

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        # 이미지를 RGB로 변환
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False

        # 포즈 감지
        results = pose.process(image)

        # 이미지를 다시 BGR로 변환
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        try:
            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark

                # 엉덩이 좌표 추출
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
                    if current_y > previous_y + 0.1:  # 임계값으로 0.05 설정
                        print("낙상이 감지되었습니다!")

                        # 소켓을 통해 메시지 전송
                        try:
                            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                                s.connect((HOST, PORT))
                                message = "낙상사고가 발생했습니다!!"
                                s.sendall(message.encode('utf-8'))
                        except Exception as e:
                            print("소켓 전송 오류:", e)

                        # 카카오톡으로 메시지 전송
                        if access_token:
                            try:
                                kakao_message = "낙상사고가 발생했습니다!"
                                print("Sending Kakao message...")
                                response = send_kakao_message(access_token, kakao_message)
                                print("Kakao API Response:", response)
                            except Exception as e:
                                print("카카오톡 메시지 전송 오류:", e)
                        else:
                            print("Access token is not available for Kakao message.")

                previous_y = current_y

        except Exception as e:
            print("포즈 감지 오류:", e)

        # 감지된 포즈 랜드마크 렌더링
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                    mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                                    mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2))


        cv2.imshow('Mediapipe Feed', image)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()