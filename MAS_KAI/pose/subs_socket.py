import socket

# 호스트와 포트를 지정합니다.
HOST = '172.30.1.36'  # 모든 IP 주소를 허용합니다.
PORT = 12345       # 사용할 포트 번호를 지정합니다.

# 소켓을 생성합니다.
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    # 소켓을 바인딩합니다.
    s.bind((HOST, PORT))
    
    # 클라이언트의 연결을 기다립니다.
    s.listen()
    print(f"서버가 {PORT} 포트에서 클라이언트의 연결을 기다리고 있습니다.")
    
    # 연결 요청을 수락합니다.
    conn, addr = s.accept()
    with conn:
        print('클라이언트가 연결되었습니다.', addr)
        
        # 클라이언트로부터 메시지를 수신합니다.
        data = conn.recv(1024)
        received_message = data.decode('utf-8')
        print('수신한 메시지:', received_message)
