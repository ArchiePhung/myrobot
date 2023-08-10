# from PyQt5.QtWidgets import (QWidget, QSlider, QLineEdit, QLabel, QPushButton, QScrollArea,QApplication,
#                              QHBoxLayout, QVBoxLayout, QMainWindow, QFormLayout)
# from PyQt5.QtCore import Qt, QSize
# from PyQt5 import QtWidgets, uic
# import sys


# class MainWindow(QMainWindow):

#     def __init__(self):
#         super().__init__()
#         self.initUI()

#     def initUI(self):
#         self.scroll = QScrollArea()             # Scroll Area which contains the widgets, set as the centralWidget
#         self.widget = QWidget()                 # Widget that contains the collection of Vertical Box
#         self.vbox = QFormLayout()               # The Vertical Box that contains the Horizontal Boxes of  labels and buttons, lineedit

#         # for i in range(1,50):
#         #     object = QLabel("TextLabel")
#         #     self.vbox.addWidget(object)

#         led_1 = QLineEdit("V823")
#         self.vbox.setWidget(led_1)

#         led_2 = QLineEdit("V822")
#         self.vbox.setWidget(led_2)

#         self.widget.setLayout(self.vbox)

#         #Scroll Area Properties
#         self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
#         self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
#         self.scroll.setWidgetResizable(True)
#         self.scroll.setWidget(self.widget)

#         self.setCentralWidget(self.scroll)
#         self.setGeometry(600, 100, 731, 291)
#         self.setWindowTitle('Scroll Area Demonstration')
#         self.show()

#         return

# def main():
#     app = QtWidgets.QApplication(sys.argv)
#     main = MainWindow()
#     sys.exit(app.exec_())

# if __name__ == '__main__':
#     main()

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLineEdit,  QFormLayout


class MainWindow(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setWindowTitle('Sign Up Form')

        layout = QFormLayout()
        self.setLayout(layout)

        layout.addRow('Name:', QLineEdit(self))
        layout.addRow('Email:', QLineEdit(self))
        layout.addRow('Password:', QLineEdit(self, echoMode=QLineEdit.EchoMode.Password))
        layout.addRow('Confirm Password:', QLineEdit(self, echoMode=QLineEdit.EchoMode.Password))
        layout.addRow('Phone:', QLineEdit(self))

        layout.addRow(QPushButton('Sign Up'))

        # show the window
        self.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec())