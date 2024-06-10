#아두이노랑 시리얼통신
import serial
import threading

def init_serial_communication(result_text_edit):
    def read_from_serial():
        while True:
            if serial_port.in_waiting > 0:
                line = serial_port.readline().decode('utf-8').strip()
                result_text_edit.append(f"Received: {line}")

    try:
        serial_port = serial.Serial('COM3', 115200, timeout=1)
        serial_thread = threading.Thread(target=read_from_serial)
        serial_thread.daemon = True
        serial_thread.start()
    except serial.SerialException as e:
        result_text_edit.append(f"Serial communication error: {e}")
