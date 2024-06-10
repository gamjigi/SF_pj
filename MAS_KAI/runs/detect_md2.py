import cv2
import pyzbar.pyzbar as pyzbar
import time
from detection import detect_objects  # 객체 검출 모듈을 임포트합니다.
from ui_info import drug_info
from PyQt5.QtWidgets import QLineEdit, QTextEdit
import queue


# 큐 생성
qr_data_queue = queue.Queue()

def run_qr_detection(result_text_edit: QTextEdit):
    # 웹캠에 액세스합니다 (기본적으로 인덱스 0에 위치한 웹캠을 가정합니다).
    cap = cv2.VideoCapture(0)

    # qr 데이터를 저장할 딕셔너리를 생성합니다.
    qr = {}

    last_qr_data = None  # 마지막으로 읽은 QR 코드 데이터
    qr_read_time = 0  # 마지막으로 QR 코드를 읽은 시간
    qr_start = False  # QR 코드가 인식되었는지 여부를 저장
    qr_data_initial = None

    while True:
        ret, img = cap.read()
        
        if not ret:
            print("웹캠에서 영상을 가져올 수 없습니다.")
            break
        
        # QR 코드를 읽습니다.
        decoded_objects = pyzbar.decode(img)
        qr_data = None
        if decoded_objects:
            qr_data = decoded_objects[0].data.decode('utf-8')  #QR코드 내장된 데이터 qr_data에 저장

            # 마지막으로 읽은 QR 코드와 다르거나 일정 시간이 지난 경우에만 처리합니다.
            if qr_data != last_qr_data or (time.time() - qr_read_time) > 5:  # 5초 딜레이
                last_qr_data = qr_data
                qr_read_time = time.time()
                print(f"QR:", qr_data)
                
                # qr_data 업데이트
                # qr_entry.setText(qr_data)

                # qr 딕셔너리에 qr_data를 키로 사용하여 빈 리스트를 초기화합니다.
                if qr_data not in qr:
                    qr[qr_data] = []
                
                # QR 코드 인식 후 2초 대기 후 객체 검출 시작
                qr_start = True
                qr_data_initial = qr_data

        # 객체 검출을 수행합니다.
        if qr_start:
            annotated_frame, class_ids = detect_objects(img)

            # QR 코드로 읽은 데이터의 key에 해당하는 딕셔너리에 class_ids를 저장합니다.
            if qr_data is not None:
                qr[qr_data].extend(class_ids)  # class_ids를 리스트에 추가합니다.
                print("QR Dictionary:", qr)

                room_data = drug_info()

                def compare_sorted_lists(list1, list2):
                    return sorted(list1) == sorted(list2)
                
#########약 일치 불일치 ######################################
                if qr_data_initial == "401":
                    room_value = room_data.get("Room 401", [])
                    room_value = [sorted(map(int, filter(None, item.split(',')))) for item in room_value]
                    sorted_qr_data = sorted(qr[qr_data_initial])
                    match_found = any(compare_sorted_lists(sorted_qr_data, item) for item in room_value)

                    if match_found:
                        message = f"401호에 맞는 약 입니다: {sorted_qr_data}"
                    else:
                        message = "401호에 해당하는 약이 아닙니다."

                    if result_text_edit:
                        result_text_edit.append(message)
                    else:
                        print(message)            
                                                
                     
                        
                    qr[qr_data_initial] = []  # QR 딕셔너리 값 초기화
                    qr_start = False  # 동작 초기화
                    
                elif qr_data_initial == "402":
                    room_value = room_data.get("Room 402", [])
                    room_value = [sorted(map(int, filter(None, item.split(',')))) for item in room_value]
                    sorted_qr_data = sorted(qr[qr_data_initial])
                    match_found = any(compare_sorted_lists(sorted_qr_data, item) for item in room_value)

                    if match_found:
                        message = f"402호에 맞는 약 입니다: {sorted_qr_data}"
                    else:
                        message = "402호에 해당하는 약이 아닙니다."

                    if result_text_edit:
                        result_text_edit.append(message)
                    else:
                        print(message)            
                                                
                     
                        
                    qr[qr_data_initial] = []  # QR 딕셔너리 값 초기화
                    qr_start = False  # 동작 초기화
                            
                            

                    
                elif qr_data_initial == "403":
                    room_value = room_data.get("Room 403", [])
                    room_value = [sorted(map(int, filter(None, item.split(',')))) for item in room_value]
                    sorted_qr_data = sorted(qr[qr_data_initial])
                    match_found = any(compare_sorted_lists(sorted_qr_data, item) for item in room_value)

                    if match_found:
                        message = f"403호에 맞는 약 입니다: {sorted_qr_data}"
                    else:
                        message = "403호에 해당하는 약이 아닙니다."

                    if result_text_edit:
                        result_text_edit.append(message)
                    else:
                        print(message)            
                                                
                     
                        
                    qr[qr_data_initial] = []  # QR 딕셔너리 값 초기화
                    qr_start = False  # 동작 초기화
                                        
                elif qr_data_initial == "404":
                    room_value = room_data.get("Room 404", [])
                    room_value = [sorted(map(int, filter(None, item.split(',')))) for item in room_value]
                    sorted_qr_data = sorted(qr[qr_data_initial])
                    match_found = any(compare_sorted_lists(sorted_qr_data, item) for item in room_value)

                    if match_found:
                        message = f"404호에 맞는 약 입니다: {sorted_qr_data}"
                    else:
                        message = "404호에 해당하는 약이 아닙니다."

                    if result_text_edit:
                        result_text_edit.append(message)
                    else:
                        print(message)            
                                                
                     
                        
                    qr[qr_data_initial] = []  # QR 딕셔너리 값 초기화
                    qr_start = False  # 동작 초기화

            # 객체 검출을 진행하는 캠 화면을 표시합니다.
            cv2.imshow('Webcam', annotated_frame)
            
            # qr_data queue에 넣기
            if qr_data:
                qr_data_queue.put(qr_data)

        if cv2.waitKey(1) == ord('q'):
            break

    # QR 코드 읽기와 객체 검출이 완료된 후, qr 딕셔너리를 출력합니다.
    print("프로그램을 종료합니다.")

    cap.release()
    cv2.destroyAllWindows()
