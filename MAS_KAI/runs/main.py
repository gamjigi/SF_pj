import sys
from PyQt5.QtWidgets import QApplication
from login import LoginWindow
from hospital_app import HospitalApp

if __name__ == "__main__":
    app = QApplication(sys.argv)
    hospital_window = HospitalApp()
    login_window = LoginWindow(hospital_window)
    login_window.show()
    sys.exit(app.exec_())
