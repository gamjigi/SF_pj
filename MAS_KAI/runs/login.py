from PyQt5.QtWidgets import QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QMessageBox
from PyQt5.QtGui import QPixmap, QBrush

class LoginWindow(QWidget):
    def __init__(self, hospital_window):
        super().__init__()
        self.hospital_window = hospital_window
        self.initUI()

    def initUI(self):
        self.setWindowTitle('로그인')
        self.setGeometry(100, 100, 300, 200)

        background_image = QLabel(self)
        pixmap = QPixmap('C:/Users/user/Desktop/final-dataset4/logo.jpg')
        self.setFixedSize(pixmap.width(), pixmap.height())
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setBrush(self.backgroundRole(), QBrush(pixmap))
        self.setPalette(palette)

        self.username_label = QLabel('사용자 이름')
        self.username_edit = QLineEdit()
        self.password_label = QLabel('비밀번호')
        self.password_edit = QLineEdit()
        self.password_edit.setEchoMode(QLineEdit.Password)
        self.login_button = QPushButton('로그인')
        self.login_button.clicked.connect(self.login)

        layout = QVBoxLayout()
        layout.addWidget(self.username_label)
        layout.addWidget(self.username_edit)
        layout.addWidget(self.password_label)
        layout.addWidget(self.password_edit)
        layout.addWidget(self.login_button)
        
        layout.addWidget(background_image)
        background_image.lower()

        self.setLayout(layout)

        self.password_edit.returnPressed.connect(self.login)

    def login(self):
        username = self.username_edit.text()
        password = self.password_edit.text()
        if username == 'kim' and password == '1234':
            self.hide()
            self.hospital_window.show()
        else:
            QMessageBox.warning(self, '로그인 실패', '잘못된 사용자 이름 또는 비밀번호입니다.')
