import tkinter as tk
import csv
import socket

def send_command_to_agv(command):
    HOST = '172.30.1.74'
    PORT = 12345

    # 소켓 생성
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))

    # 명령 전송
    client_socket.sendall(command.encode())

    # 연결 종료
    client_socket.close()

def drug_info():
    room_data = {}  # 방 번호에 따른 정보를 저장할 딕셔너리 생성

    with open('hospital.csv', newline='') as csvfile:
        reader = csv.reader(csvfile)
        for i, row in enumerate(reader):
            room_num = f"Room {401 + i}"  # 방 번호 설정
            room_data[room_num] = row

    return room_data  # 수정: 딕셔너리 반환

#숫자에 대응하는 약물 형태 딕셔너리
medication_types = {
    "0": "Capsule",
    "1": "Pill",
    "2": "Tablet"
}

def clear_csv():
    with open("hospital.csv", 'w', newline='') as f:
        pass  # 파일을 열어서 아무 작업도 하지 않고 내용을 지웁니다.

def submit(entry_fields):
    room_data = {}  # 각 방의 정보를 저장할 딕셔너리 생성

    for i, row in enumerate(entry_fields):
        data = [entry.get() for entry in row]
        room_num = f"Room {401 + i}"  # 방 번호 설정
        # 약물 형태를 문자열로 변경
        data = [medication_types.get(value, value) for value in data]
        room_data[room_num] = data  # 딕셔너리에 저장

    write_to_csv(room_data)
    
    for row in entry_fields:
        for entry in row:
            entry.delete(0, tk.END)

def write_to_csv(room_data):
    with open("hospital.csv", 'w', newline='') as f:
        fieldnames = [f"Room {401 + i}" for i in range(len(room_data))]  # 방 번호 설정
        writer = csv.DictWriter(f, fieldnames=[field.split()[1] for field in fieldnames])
        # writer.writeheader()  # 헤더를 쓰지 않음
        for room_num, data in room_data.items():
            writer.writerow({field.split()[1]: value for field, value in zip(fieldnames, data)})  # 각 방의 데이터를 쓰기


if __name__ == "__main__": #여기서 직접 실행할때만 실행되게 블록 해놓음
    
    # Tkinter 윈도우 생성
    root = tk.Tk()
    root.title("병원 정보")

    rooms = ["401", "402", "403", "404"]

    # 약물 형태 레이블 생성
    medication_label = tk.Label(root, text="약물 형태")
    medication_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")

    for i, (key, value) in enumerate(medication_types.items(), start=1):
        label = tk.Label(root, text=f"{key} = {value}")
        label.grid(row=i, column=0, padx=10, pady=5, sticky="w")

    # 호실별 입력 필드 생성
    entry_fields = []
    for i in range(len(rooms)):
        entry_row = []
        for j in range(4):
            if j == 0:
                label = tk.Label(root, text=f"{rooms[i]}호:")
                label.grid(row=i, column=j*2+1, padx=10, pady=5, sticky="w")
            
            entry_field = tk.Entry(root, width=10)  # 입력란의 너비를 반으로 줄임
            entry_field.grid(row=i, column=j*2+2, padx=5, pady=5)
            entry_row.append(entry_field)
        entry_fields.append(entry_row)

    # 제출 버튼 생성
    submit_button = tk.Button(root, text="입력 완료", command=lambda: submit(entry_fields))
    submit_button.grid(row=len(rooms), columnspan=8, padx=10, pady=5)

    agv_button_401 = tk.Button(root, text="401", command=lambda: send_command_to_agv("401"))
    agv_button_401.grid(row=len(rooms), columnspan=1, padx=10, pady=5)

    agv_button_402 = tk.Button(root, text="402", command=lambda: send_command_to_agv("402"))
    agv_button_402.grid(row=len(rooms) + 1, columnspan=1, padx=10, pady=5)

    agv_button_403 = tk.Button(root, text="403", command=lambda: send_command_to_agv("403"))
    agv_button_403.grid(row=len(rooms), columnspan=4, padx=10, pady=5)

    dark_navi = tk.Button(root, text="dark", command=lambda: send_command_to_agv("dark"))
    dark_navi.grid(row=len(rooms) + 1, columnspan=4, padx=10, pady=5)
    

    # 삭제 버튼 생성
    clear_button = tk.Button(root, text="정보 초기화", command=clear_csv)
    clear_button.grid(row=len(rooms) + 1, columnspan=8, padx=10, pady=5)

    # Tkinter 윈도우 실행
    root.mainloop()



    ####UI에 약 클래스 입력, 딕셔너리로 저장
    