from ui_info import drug_info

def print_csv():
    # drug_info 함수에서 데이터를 가져옵니다.
    room_data = drug_info()

    # 데이터의 제목을 출력합니다.
    print("Drug Info:")

    # 각 방의 데이터에 대해 반복합니다.
    for room, data in room_data.items():
        # 방 번호와 해당 방의 데이터를 출력합니다.
        print(f"호실: {room}")
        print("데이터:")
        
        # 데이터를 더 보기 좋게 출력합니다.
        for i, item in enumerate(data):
            print(f" {i + 1}번 환자 : {item}")
        
        # 방 사이에 빈 줄을 추가하여 가독성을 높입니다.
        print()
if __name__=="__main__":
    print_csv()
