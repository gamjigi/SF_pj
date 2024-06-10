from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget
import csv


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(400, 300)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")

        self.rooms = ["401", "402", "403", "404"]
        self.entry_fields = []

        for room in self.rooms:
            label = QLabel(f"{room}호:", self.centralwidget)
            self.verticalLayout.addWidget(label)

            entry_row = []
            for _ in range(3):
                entry = QLineEdit(self.centralwidget)
                self.verticalLayout.addWidget(entry)
                entry_row.append(entry)
            self.entry_fields.append(entry_row)

        self.submit_button = QPushButton("입력 완료", self.centralwidget)
        self.submit_button.clicked.connect(self.submit)
        self.verticalLayout.addWidget(self.submit_button)

        self.clear_button = QPushButton("정보 초기화", self.centralwidget)
        self.clear_button.clicked.connect(self.clear_csv)
        self.verticalLayout.addWidget(self.clear_button)

        MainWindow.setCentralWidget(self.centralwidget)

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
        with open("hospital.csv", 'w', newline=''):
            pass

    def write_to_csv(self, room_data):
        with open("hospital.csv", 'w', newline='') as f:
            fieldnames = [f"Room {401 + i}" for i in range(len(room_data))]
            writer = csv.DictWriter(f, fieldnames=[field.split()[1] for field in fieldnames])

            for room_num, data in room_data.items():
                writer.writerow({field.split()[1]: value for field, value in zip(fieldnames, data)})


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
