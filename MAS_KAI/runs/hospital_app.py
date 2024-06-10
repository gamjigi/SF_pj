import csv
import socket
import threading
import requests
from PyQt5.QtWidgets import QMainWindow, QTabWidget, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QGroupBox, QTextEdit, QTableWidgetItem, QTableWidget, QSplitter, QSizePolicy, QHeaderView, QMessageBox
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from db_md import fetch_data_from_db
from form import PrescriptionForm
from detect_md2 import run_qr_detection
from socket_communication import start_receiving
from serial_communication import init_serial_communication

class HospitalApp(QMainWindow):
    def __init__(self, use_serial=False):
        super().__init__()
        self.use_serial = use_serial
        self.initUI()
        if self.use_serial:
            self.init_serial_communication()

    def initUI(self):
        self.setWindowTitle('병원 정보 시스템')
        self.setGeometry(100, 100, 1200, 800)
        self.setStyleSheet("QMainWindow { background-color: #f5f5f5; }")

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.day_tab = QWidget()
        self.night_tab = QWidget()
        self.data_tab = QWidget()

        self.tabs.addTab(self.day_tab, '주간')
        self.tabs.addTab(self.night_tab, '야간')
        self.tabs.addTab(self.data_tab, '데이터 보기')
        
        self.initDayTab()
        self.initNightTab()
        self.initDataTab()

    def initDayTab(self):
        main_splitter = QSplitter(Qt.Horizontal)

        left_widget = QWidget()
        left_layout = QVBoxLayout()

        self.drug_group = QGroupBox("약물 정보")
        self.drug_group.setFont(QFont('Arial', 16))
        drug_layout = QVBoxLayout()

        self.entry_fields = []
        rooms = ["401", "402", "403", "404"]

        for room in rooms:
            room_layout = QHBoxLayout()
            room_label = QLabel(f"{room}호:")
            room_label.setFont(QFont('Arial', 14, QFont.Bold))
            room_layout.addWidget(room_label)

            room_entries = []
            for _ in range(4):
                entry = QLineEdit()
                entry.setFont(QFont('Arial', 12))
                entry.setStyleSheet("padding: 5px;")
                room_entries.append(entry)
                room_layout.addWidget(entry)
            self.entry_fields.append(room_entries)
            
            drug_layout.addLayout(room_layout)

        self.drug_group.setLayout(drug_layout)
        left_layout.addWidget(self.drug_group)

        button_layout = QHBoxLayout()
        button_layout.setSpacing(20)

        self.save_button = QPushButton("저장")
        self.save_button.setFont(QFont('Arial', 14))
        self.save_button.setStyleSheet("padding: 10px 20px; background-color: #4CAF50; color: white; border: none; border-radius: 5px;")
        self.save_button.clicked.connect(self.submit)
        button_layout.addWidget(self.save_button)

        self.clear_button = QPushButton("지우기")
        self.clear_button.setFont(QFont('Arial', 14))
        self.clear_button.setStyleSheet("padding: 10px 20px; background-color: #f44336; color: white; border: none; border-radius: 5px;")
        self.clear_button.clicked.connect(self.clear_csv)
        button_layout.addWidget(self.clear_button)
        
        self.detect_button = QPushButton("QR 탐지")
        self.detect_button.setFont(QFont('Arial', 14))
        self.detect_button.setStyleSheet("padding: 10px 20px; background-color: #008CBA; color: white; border: none; border-radius: 5px;")
        self.detect_button.clicked.connect(self.start_detection)
        button_layout.addWidget(self.detect_button)

        left_layout.addLayout(button_layout)

        self.confirm_group = QGroupBox("확인")
        self.confirm_group.setFont(QFont('Arial', 16))
        confirm_layout = QVBoxLayout()

        self.result_text_edit = QTextEdit()
        self.result_text_edit.setReadOnly(True)
        self.result_text_edit.setFont(QFont('Arial', 12))
        confirm_layout.addWidget(self.result_text_edit)

        self.confirm_group.setLayout(confirm_layout)
        left_layout.addWidget(self.confirm_group)

        self.delivery_group = QGroupBox("배송")
        self.delivery_group.setFont(QFont('Arial', 16))
        delivery_layout = QVBoxLayout()

        self.agv_buttons = []
        for room in rooms:
            button = QPushButton(room)
            button.setFont(QFont('Arial', 14))
            button.setStyleSheet("padding: 10px 20px; background-color: #555; color: white; border: none; border-radius: 5px;")
            button.clicked.connect(lambda checked, r=room: self.send_command_to_agv(r))
            self.agv_buttons.append(button)
            delivery_layout.addWidget(button)

        self.all_button = QPushButton("전체")
        self.all_button.setFont(QFont('Arial', 14))
        self.all_button.setStyleSheet("padding: 10px 20px; background-color: #555; color: white; border: none; border-radius: 5px;")
        self.all_button.clicked.connect(lambda: self.send_command_to_agv(''))
        delivery_layout.addWidget(self.all_button)

        self.stop_button = QPushButton("정지")
        self.stop_button.setFont(QFont('Arial', 14))
        self.stop_button.setStyleSheet("padding: 10px 20px; background-color: #555; color: white; border: none; border-radius: 5px;")
        self.stop_button.clicked.connect(self.stop_agv_communication)
        delivery_layout.addWidget(self.stop_button)

        self.delivery_group.setLayout(delivery_layout)
        left_layout.addWidget(self.delivery_group)

        left_widget.setLayout(left_layout)
        main_splitter.addWidget(left_widget)

        right_widget = QWidget()
        right_layout = QVBoxLayout()

        self.load_data_button = QPushButton("DB 불러오기")
        self.load_data_button.setFont(QFont('Arial', 14))
        self.load_data_button.setStyleSheet("padding: 10px 20px; background-color: #4CAF50; color: white; border: none; border-radius: 5px;")
        self.load_data_button.clicked.connect(self.load_data_from_db)
        right_layout.addWidget(self.load_data_button)

        self.data_table = QTableWidget()
        self.data_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.data_table.horizontalHeader().setStretchLastSection(True)
        self.data_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        right_layout.addWidget(self.data_table)

        right_widget.setLayout(right_layout)
        main_splitter.addWidget(right_widget)

        main_splitter.setSizes([300, 900])

        main_layout = QVBoxLayout()
        main_layout.addWidget(main_splitter)
        self.day_tab.setLayout(main_layout)

    def initNightTab(self):
        layout = QVBoxLayout()

        self.message_display = QTextEdit()
        self.message_display.setFont(QFont('Arial', 12))
        self.message_display.setReadOnly(True)
        layout.addWidget(self.message_display)

        self.start_receiving_button = QPushButton("메시지 수신 시작")
        self.start_receiving_button.setFont(QFont('Arial', 14))
        self.start_receiving_button.setStyleSheet("padding: 10px 20px; background-color: #008CBA; color: white; border: none; border-radius: 5px;")
        self.start_receiving_button.clicked.connect(self.start_receiving)
        layout.addWidget(self.start_receiving_button)

        self.dark_navi_button = QPushButton("야간 주행")
        self.dark_navi_button.setFont(QFont('Arial', 14))
        self.dark_navi_button.setStyleSheet("padding: 10px 20px; background-color: #4CAF50; color: white; border: none; border-radius: 5px;")
        self.dark_navi_button.clicked.connect(lambda: self.send_command_to_agv("dark"))
        layout.addWidget(self.dark_navi_button)

        self.night_tab.setLayout(layout)


    def initDataTab(self):
        layout = QVBoxLayout()

        self.refresh_data_button = QPushButton("데이터 새로고침")
        self.refresh_data_button.setFont(QFont('Arial', 14))
        self.refresh_data_button.setStyleSheet("padding: 10px 20px; background-color: #4CAF50; color: white; border: none; border-radius: 5px;")
        self.refresh_data_button.clicked.connect(self.load_web_data)
        layout.addWidget(self.refresh_data_button)

        self.web_data_table = QTableWidget()
        self.web_data_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.web_data_table.horizontalHeader().setStretchLastSection(True)
        self.web_data_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        layout.addWidget(self.web_data_table)

        self.data_tab.setLayout(layout)

    def load_web_data(self):
        response = requests.get('http://localhost:5000/data')
        if response.status_code == 200:
            data = response.json()
            self.web_data_table.setRowCount(len(data))
            self.web_data_table.setColumnCount(6)
            self.web_data_table.setHorizontalHeaderLabels(['이름', '나이', '증상', '생년월일', '호실', '성별'])
            
            for row_idx, row_data in enumerate(data):
                for col_idx, col_data in enumerate(row_data):
                    self.web_data_table.setItem(row_idx, col_idx, QTableWidgetItem(str(col_data)))

    def submit(self):
        room_data = {}

        for i, row in enumerate(self.entry_fields):
            data = [entry.text() for entry in row]
            room_num = f"Room {401 + i}"
            room_data[room_num] = data

        self.write_to_csv(room_data)

        for row in self.entry_fields:
            for entry in row:
                entry.clear()

    def clear_csv(self):
        with open("hospital.csv", 'w', newline='') as f:
            pass

    def write_to_csv(self, room_data):
        with open("hospital.csv", 'w', newline='') as f:
            fieldnames = [f"Room {401 + i}" for i in range(len(room_data))]
            writer = csv.DictWriter(f, fieldnames=[field.split()[1] for field in fieldnames])
            for room_num, data in room_data.items():
                writer.writerow({field.split()[1]: value for field, value in zip(fieldnames, data)})

    def send_command_to_agv(self, command):
        HOST = '172.30.1.74'
        PORT = 12345

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((HOST, PORT))
        client_socket.sendall(command.encode())
        client_socket.close()
        
    def stop_agv_communication(self):
        print("정지 버튼 눌리고 신호 끊기시작")
        if hasattr(self, 'agv_client_socket'):
            self.agv_client_socket.close()
            print("통신 끊음")
            del self.agv_client_socket

    def start_detection(self):
        run_qr_detection(self.result_text_edit)

    def load_data_from_db(self):
        results, column_names = fetch_data_from_db()
        if results:
            self.data_table.setRowCount(len(results))
            self.data_table.setColumnCount(len(column_names))
            self.data_table.setHorizontalHeaderLabels(column_names)
            
            for row_idx, row_data in enumerate(results):
                for col_idx, col_data in enumerate(row_data):
                    if column_names[col_idx] == '처방전':
                        button = QPushButton(str(col_data))
                        button.clicked.connect(lambda checked, r=row_data: self.on_prescription_button_clicked(r))
                        self.data_table.setCellWidget(row_idx, col_idx, button)
                    else:
                        self.data_table.setItem(row_idx, col_idx, QTableWidgetItem(str(col_data)))

    def on_prescription_button_clicked(self, row_data):
        photo_path = f'C:/Users/user/Desktop/final-dataset4/face/asd.jpg'
        self.prescription_form = PrescriptionForm(prescription_data={
            'issue_number': row_data[0],
            'patient_name': row_data[1],
            'birthdate': row_data[5],
            'diagnosis': row_data[6],
            'sex' : row_data[3],
            'address': row_data[4],
            'prescription': row_data[7],
            'photo_path': photo_path
        })
        self.prescription_form.show()

    def start_receiving(self):
        start_receiving(self.message_display)

    def init_serial_communication(self):
        init_serial_communication(self.result_text_edit)
