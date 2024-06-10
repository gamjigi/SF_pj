###처방전 양식#####
import os
from PyQt5.QtWidgets import QMainWindow, QWidget, QLabel, QLineEdit, QTextEdit, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QGroupBox, QComboBox, QCheckBox, QFrame
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QPalette, QBrush

class PrescriptionForm(QMainWindow):
    def __init__(self, prescription_data=None):
        super().__init__()  # QMainWindow 초기화
        self.prescription_data = prescription_data  # 전달받은 처방전 데이터를 저장
        self.initUI()  # UI 초기화 메서드 호출

    def initUI(self):
        self.setWindowTitle('처방전')  # 창 제목 설정
        self.setGeometry(100, 100, 600, 800)  # 창 크기와 위치 설정

        # 배경 이미지 설정
        background_image_path = 'C:/Users/user/Desktop/final-dataset4/logo_tr_opacity.jpg'
        self.set_background_image(background_image_path)  # 배경 이미지 설정 메서드 호출

        # 메인 위젯과 레이아웃 설정
        main_widget = QWidget()  # 메인 위젯 생성
        main_layout = QVBoxLayout()  # 메인 레이아웃으로 QVBoxLayout 설정

        # 1. 타이틀과 용어
        title_layout = QHBoxLayout()  # 타이틀 레이아웃으로 QHBoxLayout 설정
        title_label = QLabel('처방전')  # '처방전' 레이블 생성
        title_label.setStyleSheet("font-size: 24px; font-weight: bold;")  # 레이블 스타일 설정
        title_layout.addWidget(title_label)  # 타이틀 레이블을 타이틀 레이아웃에 추가
        title_layout.addStretch()  # 레이아웃에 빈 공간 추가
        usage_label = QLabel('(약국제출용)')  # '(약국제출용)' 레이블 생성
        title_layout.addWidget(usage_label)  # 용도 레이블을 타이틀 레이아웃에 추가
        main_layout.addLayout(title_layout)  # 타이틀 레이아웃을 메인 레이아웃에 추가

        # 2. 기본 정보 입력 부분
        basic_info_group = QGroupBox()  # 기본 정보 그룹박스 생성
        basic_info_layout = QGridLayout()  # 기본 정보 레이아웃으로 QGridLayout 설정

        # 얼굴 사진 칸 추가
        self.photo_label = QLabel()  # 사진을 표시할 레이블 생성
        self.photo_label.setFrameStyle(QFrame.Box)  # 레이블의 테두리 스타일 설정
        self.photo_label.setFixedSize(100, 100)  # 레이블의 고정 크기 설정
        self.photo_label.setAlignment(Qt.AlignCenter)  # 레이블의 정렬 설정
        self.load_photo(self.prescription_data.get('photo_path', ''))  # 사진 경로 로드
        basic_info_layout.addWidget(self.photo_label, 0, 0, 4, 1)  # 사진 레이블을 기본 정보 레이아웃에 추가 (4행 1열 병합)

        self.issue_number = QLineEdit()  # 교부번호 입력 필드 생성
        self.medical_insurance = QCheckBox('의료보험')  # 의료보험 체크박스 생성
        self.medical_assistance = QCheckBox('의료급여')  # 의료급여 체크박스 생성
        self.worker_compensation = QCheckBox('산재보험')  # 산재보험 체크박스 생성
        self.others = QCheckBox('기타')  # 기타 체크박스 생성
        self.patient_name = QLineEdit()  # 성명 입력 필드 생성
        self.patient_gender = QLineEdit()
        self.birthdate = QLineEdit()  # 생년월일 입력 필드 생성
        self.id_number = QLineEdit()  # 주민등록번호 입력 필드 생성
        self.diagnosis_date = QLineEdit()  # 진료일자 입력 필드 생성
        self.address = QLineEdit()  # 진료과목 입력 필드 생성
        self.doctor_name = QLineEdit()  # 의사명 입력 필드 생성
        self.doctor_phone = QLineEdit()  # 의사 전화번호 입력 필드 생성

        # 교부번호 및 보험 정보
        basic_info_layout.addWidget(QLabel('교부번호'), 0, 1)  # '교부번호' 레이블을 기본 정보 레이아웃에 추가
        basic_info_layout.addWidget(self.issue_number, 0, 2, 1, 2)  # 교부번호 입력 필드를 기본 정보 레이아웃에 추가 (1행 2열 병합)

        insurance_layout = QHBoxLayout()  # 보험 정보 레이아웃으로 QHBoxLayout 설정
        insurance_layout.addWidget(self.medical_insurance)  # 의료보험 체크박스를 보험 정보 레이아웃에 추가
        insurance_layout.addWidget(self.medical_assistance)  # 의료급여 체크박스를 보험 정보 레이아웃에 추가
        insurance_layout.addWidget(self.worker_compensation)  # 산재보험 체크박스를 보험 정보 레이아웃에 추가
        insurance_layout.addWidget(self.others)  # 기타 체크박스를 보험 정보 레이아웃에 추가
        insurance_widget = QWidget()  # 보험 정보 위젯 생성
        insurance_widget.setLayout(insurance_layout)  # 보험 정보 레이아웃을 보험 정보 위젯에 설정
        basic_info_layout.addWidget(insurance_widget, 0, 4, 1, 4)  # 보험 정보 위젯을 기본 정보 레이아웃에 추가 (1행 4열 병합)

        basic_info_layout.addWidget(QLabel('성명'), 1, 1)  # '성명' 레이블을 기본 정보 레이아웃에 추가
        basic_info_layout.addWidget(self.patient_name, 1, 2, 1, 2)  # 성명 입력 필드를 기본 정보 레이아웃에 추가 (1행 2열 병합)
        basic_info_layout.addWidget(QLabel('성별'), 1, 4)  # '성별' 레이블을 기본 정보 레이아웃에 추가
        basic_info_layout.addWidget(self.patient_gender, 1, 5)  # 성별 선택 콤보박스를 기본 정보 레이아웃에 추가
        basic_info_layout.addWidget(QLabel('생년월일'), 1, 6)  # '생년월일' 레이블을 기본 정보 레이아웃에 추가
        basic_info_layout.addWidget(self.birthdate, 1, 7)  # 생년월일 입력 필드를 기본 정보 레이아웃에 추가

        basic_info_layout.addWidget(QLabel('증상'), 2, 1)  # '주민등록번호' 레이블을 기본 정보 레이아웃에 추가
        basic_info_layout.addWidget(self.id_number, 2, 2, 1, 2)  # 주민등록번호 입력 필드를 기본 정보 레이아웃에 추가 (1행 2열 병합)
        basic_info_layout.addWidget(QLabel('주소'), 2, 4)  # '주소' 레이블을 기본 정보 레이아웃에 추가
        basic_info_layout.addWidget(self.address, 2, 5, 1, 3)  # 진료일자 입력 필드를 기본 정보 레이아웃에 추가 (1행 3열 병합)

        basic_info_layout.addWidget(QLabel('의사명'), 3, 1)  # '의사명' 레이블을 기본 정보 레이아웃에 추가
        basic_info_layout.addWidget(self.doctor_name, 3, 2, 1, 2)  # 의사명 입력 필드를 기본 정보 레이아웃에 추가 (1행 2열 병합)
        basic_info_layout.addWidget(QLabel('의사 전화번호'), 3, 4)  # '의사 전화번호' 레이블을 기본 정보 레이아웃에 추가
        basic_info_layout.addWidget(self.doctor_phone, 3, 5)  # 의사 전화번호 입력 필드를 기본 정보 레이아웃에 추가
        basic_info_layout.addWidget(QLabel('진료일자'), 3, 6)  # '진료일자' 레이블을 기본 정보 레이아웃에 추가
        basic_info_layout.addWidget(self.diagnosis_date, 3, 7)  # 진료일자 입력 필드를 기본 정보 레이아웃에 추가

        basic_info_group.setLayout(basic_info_layout)  # 기본 정보 레이아웃을 기본 정보 그룹박스에 설정
        main_layout.addWidget(basic_info_group)  # 기본 정보 그룹박스를 메인 레이아웃에 추가

        # 3. 처방전 내용 입력 부분
        prescription_group = QGroupBox("처방 내용")  # 처방 내용 그룹박스 생성
        prescription_layout = QVBoxLayout()  # 처방 내용 레이아웃으로 QVBoxLayout 설정

        self.prescription_text = QTextEdit()  # 처방 내용 입력 필드 생성
        prescription_layout.addWidget(self.prescription_text)  # 처방 내용 입력 필드를 처방 내용 레이아웃에 추가

        prescription_group.setLayout(prescription_layout)  # 처방 내용 레이아웃을 처방 내용 그룹박스에 설정
        main_layout.addWidget(prescription_group)  # 처방 내용 그룹박스를 메인 레이아웃에 추가

        # 4. 주의사항 및 추가 정보
        additional_info_group = QGroupBox("추가 정보")  # 추가 정보 그룹박스 생성
        additional_info_layout = QGridLayout()  # 추가 정보 레이아웃으로 QGridLayout 설정

        self.warning_text = QTextEdit()  # 주의 사항 입력 필드 생성
        additional_info_layout.addWidget(QLabel('주의 사항'), 0, 0)  # '주의 사항' 레이블을 추가 정보 레이아웃에 추가
        additional_info_layout.addWidget(self.warning_text, 1, 0, 2, 1)  # 주의 사항 입력 필드를 추가 정보 레이아웃에 추가 (2행 1열 병합)

        self.pharmacist_note = QTextEdit()  # 약사 참고사항 입력 필드 생성
        additional_info_layout.addWidget(QLabel('약사 참고사항'), 0, 1)  # '약사 참고사항' 레이블을 추가 정보 레이아웃에 추가
        additional_info_layout.addWidget(self.pharmacist_note, 1, 1, 2, 1)  # 약사 참고사항 입력 필드를 추가 정보 레이아웃에 추가 (2행 1열 병합)

        additional_info_group.setLayout(additional_info_layout)  # 추가 정보 레이아웃을 추가 정보 그룹박스에 설정
        main_layout.addWidget(additional_info_group)  # 추가 정보 그룹박스를 메인 레이아웃에 추가

        # 5. 확인 및 제출 버튼
        button_layout = QHBoxLayout()  # 버튼 레이아웃으로 QHBoxLayout 설정
        self.save_button = QPushButton('저장')  # 저장 버튼 생성
        button_layout.addWidget(self.save_button)  # 저장 버튼을 버튼 레이아웃에 추가
        self.submit_button = QPushButton('제출')  # 제출 버튼 생성
        button_layout.addWidget(self.submit_button)  # 제출 버튼을 버튼 레이아웃에 추가

        main_layout.addLayout(button_layout)  # 버튼 레이아웃을 메인 레이아웃에 추가

        main_widget.setLayout(main_layout)  # 메인 레이아웃을 메인 위젯에 설정
        self.setCentralWidget(main_widget)  # 메인 위젯을 중앙 위젯으로 설정

        if self.prescription_data:
            self.populate_fields()  # 처방전 데이터가 있으면 필드 채우기

    def set_background_image(self, image_path):
        palette = QPalette()  # 팔레트 생성
        pixmap = QPixmap(image_path)  # 주어진 경로의 이미지를 픽스맵으로 로드
        pixmap = pixmap.scaled(self.size(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)  # 픽스맵 크기 조정
        palette.setBrush(QPalette.Background, QBrush(pixmap))  # 팔레트의 배경 브러시로 픽스맵 설정
        self.setPalette(palette)  # 팔레트 설정

    def load_photo(self, image_path):
        if os.path.exists(image_path):  # 파일 경로가 존재하는지 확인
            pixmap = QPixmap(image_path)  # 주어진 경로의 이미지를 픽스맵으로 로드
            self.photo_label.setPixmap(pixmap.scaled(self.photo_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))  # 사진 레이블에 픽스맵 설정
        else:
            self.photo_label.setText("이미지를 찾을 수 없습니다.")  # 경로가 유효하지 않을 경우 텍스트 표시

    def populate_fields(self): #db에서 해당하는 변수 가져옴
        self.issue_number.setText(str(self.prescription_data.get('issue_number', '')))  # 교부번호 필드 채우기
        self.patient_name.setText(self.prescription_data.get('patient_name', ''))  # 성명 필드 채우기
        self.birthdate.setText(self.prescription_data.get('birthdate', ''))  # 생년월일 필드 채우기
        self.id_number.setText(self.prescription_data.get('diagnosis', ''))  # 주민등록번호 필드 채우기
        self.diagnosis_date.setText(self.prescription_data.get('diagnosis_date', ''))  # 진료일자 필드 채우기
        self.address.setText(self.prescription_data.get('address', ''))  # 주소 필드 채우기
        self.doctor_name.setText(self.prescription_data.get('doctor_name', ''))  # 의사명 필드 채우기
        self.doctor_phone.setText(self.prescription_data.get('doctor_phone', ''))  # 의사 전화번호 필드 채우기
        self.prescription_text.setText(self.prescription_data.get('prescription', ''))  # 처방 내용 필드 채우기
        self.patient_gender.setText(self.prescription_data.get('sex', '')) #성별 필드 채우기
