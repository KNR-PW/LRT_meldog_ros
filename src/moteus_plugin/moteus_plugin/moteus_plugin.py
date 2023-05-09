import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from PySide6.QtCore import QFile
from ui_moteus_plugin import Ui_Form


class Form(QWidget):
    def __init__(self):
        super(Form, self).__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)


if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = Form()
    window.show()

    sys.exit(app.exec())
