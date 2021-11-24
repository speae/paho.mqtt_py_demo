# from PyQt5.QtCore import Qt
# from PyQt5.QtGui import QPixmap
# from PyQt5 import uic
# from PyQt5.QtWidgets import *
# import sys
# from PyQt5.QtGui import *

# # qPixmapVar = QPixmap()

# # # 상대경로 이용
# # qPixmapVar.load(
# #     "/Users/jwoh/WorkSpace/PyQt5forBeginner/02.02 PushButton/img/man4.png")

# # UI파일 연결
# # 단, UI파일은 Python 코드 파일과 같은 디렉토리에 위치해야한다.
# form_class = uic.loadUiType(
#     "/Users/jwoh/WorkSpace/PyQt5forBeginner/02.02 PushButton/ui/qpixaxmapTest.ui")[0]


# class WindowClass(QMainWindow, form_class):
#     def __init__(self):
#         super().__init__()
#         self.setupUi(self)
#         self.qPixmapVar = QPixmap()
#         self.qPixmapVar.load(
#             "/Users/jwoh/WorkSpace/PyQt5forBeginner/02.02 PushButton/img/man4.png")


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     myWindow = WindowClass()
#     myWindow.show()
#     app.exec_()

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt


class MyApp(QWidget):

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        pixmap = QPixmap(
            '/Users/jwoh/WorkSpace/PyQt5forBeginner/02.02 PushButton/img/man4.png')

        lbl_img = QLabel()
        lbl_img.setPixmap(pixmap)
        lbl_size = QLabel('Width: '+str(pixmap.width()) +
                          ', Height: '+str(pixmap.height()))
        lbl_size.setAlignment(Qt.AlignCenter)

        vbox = QVBoxLayout()
        vbox.addWidget(lbl_img)
        vbox.addWidget(lbl_size)
        self.setLayout(vbox)

        self.setWindowTitle('QPixmap')
        self.move(300, 300)
        self.show()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MyApp()
    sys.exit(app.exec_())
