import socket
import threading

def start_receiving(message_display):
    # 클라이언트로부터 메시지를 수신하는 함수
    def receive_message(sock):
        while True:
            try:
                # 데이터를 수신
                data = sock.recv(1024)
                if data:
                    # 수신된 데이터를 UTF-8로 디코딩하여 메시지 리스트에 추가
                    received_message = data.decode('utf-8')
                    message_display.append(received_message)
            except ConnectionError:
                break

    # 새로운 클라이언트 연결을 수락하는 함수
    def accept_connections(server_socket):
        while True:
            # 클라이언트 연결 수락
            client_socket, _ = server_socket.accept()
            # 수신 스레드 생성 및 시작
            receive_thread = threading.Thread(target=receive_message, args=(client_socket,))
            receive_thread.start()

    HOST = '0.0.0.0'  # 모든 인터페이스에서 연결을 수락
    PORT = 12345      # 사용할 포트 번호

    # 서버 소켓 생성
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))  # 소켓을 호스트와 포트에 바인드
    server_socket.listen(5)           # 최대 5개의 연결 대기

    # 연결 수락 스레드 생성 및 시작
    accept_thread = threading.Thread(target=accept_connections, args=(server_socket,))
    accept_thread.start()

# 예시 메시지 디스플레이 리스트
message_display = []

# 수신 함수 시작
start_receiving(message_display)
