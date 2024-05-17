import cv2
import csv
import pyzbar.pyzbar as pyzbar


import csv

def drug_info():
    room_data = {}  # 방 번호에 따른 정보를 저장할 딕셔너리 생성

    with open('hospital.csv', newline='') as csvfile:
        reader = csv.reader(csvfile)
        for i, row in enumerate(reader):
            room_num = f"Room {401 + i}"  # 방 번호 설정
            room_data[room_num] = row

    return room_data  # 수정: 딕셔너리 반환

# drug_info 함수 호출하여 결과를 변수에 저장
room_data = drug_info() 

#여기에 qr코드안에 분류한 약들 저장하고 딕셔너리안에 입력된 숫자들과 비교?



#카메라로부터 프레임 읽기
cap = cv2.VideoCapture(0)


while True:
    ret, frame = cap.read()
    
    # QR 코드 찾기
    decoded_objects = pyzbar.decode(frame)
    
    # 발견된 모든 QR 코드에 대해 반복
    for obj in decoded_objects:
        # 코드의 데이터 및 유형 출력
        print("타입:", obj.type)
        print("데이터:", obj.data.decode('utf-8'))  # 데이터 디코딩하여 출력
        decoded_data = obj.data.decode('utf-8') #qr로 읽은 데이터 변수 저장  ex.호실
        
        # 수정: room_data의 각 키에서 "Room"을 제외하고 숫자만 추출하여 비교
        for idx, key in enumerate(room_data.keys(), start=1):
            room_num = key.split()[1]
            if decoded_data == room_num:
                if idx ==1:
                    print(f"{key} 입니다")
                elif idx == 2:
                    print(f"{key} 입니다.")
                elif idx == 3:
                    print(f"{key} 입니다.")
                elif idx == 4:
                    print(f"{key} 입니다.")
                break
        else:
            print("일치하는 호실이 없습니다.")
        
        
    
    # 프레임 표시
    cv2.imshow("QR Code Scanner", frame)
    
    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 해제
cap.release()
cv2.destroyAllWindows()
