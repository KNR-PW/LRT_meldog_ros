# -*- coding: utf-8 -*-

################################################################################
# Form generated from reading UI file 'moteus_plugin.ui'
##
# Created by: Qt User Interface Compiler version 6.5.0
##
# WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
                            QMetaObject, QObject, QPoint, QRect,
                            QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
                           QFont, QFontDatabase, QGradient, QIcon,
                           QImage, QKeySequence, QLinearGradient, QPainter,
                           QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QDoubleSpinBox, QFrame,
                               QHBoxLayout, QLCDNumber, QLabel, QPushButton,
                               QSizePolicy, QVBoxLayout, QWidget)


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(612, 617)
        Form.setStyleSheet(u"QWidget {\n"
                           "  background-color: #ffffff;\n"
                           "  color: #333333;\n"
                           "  border: 1px solid #bbbbbb;\n"
                           "  border-radius: 5px;\n"
                           "  font-family: Arial;\n"
                           "  font-size: 12pt;\n"
                           "}\n"
                           "\n"
                           "QPushButton {\n"
                           "  background-color: #008cff;\n"
                           "  color: #ffffff;\n"
                           "  border: none;\n"
                           "  border-radius: 5px;\n"
                           "  padding: 5px 10px;\n"
                           "  font-family: Arial;\n"
                           "  font-size: 12pt;\n"
                           "}\n"
                           "\n"
                           "QPushButton:hover {\n"
                           "  background-color: #0066cc;\n"
                           "}\n"
                           "\n"
                           "QLineEdit {\n"
                           "  background-color: #ffffff;\n"
                           "  color: #333333;\n"
                           "  border: 1px solid #bbbbbb;\n"
                           "  border-radius: 5px;\n"
                           "  font-family: Arial;\n"
                           "  font-size: 12pt;\n"
                           "}")
        self.verticalLayout_4 = QVBoxLayout(Form)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label_10 = QLabel(Form)
        self.label_10.setObjectName(u"label_10")
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.label_10.sizePolicy().hasHeightForWidth())
        self.label_10.setSizePolicy(sizePolicy)
        self.label_10.setStyleSheet(u"QLabel {\n"
                                    "    font-size: 30px;\n"
                                    "    font-weight: bold;\n"
                                    "    color: #333333;\n"
                                    "    background-color: rgb(153, 193, 241);\n"
                                    "    padding: 4px;\n"
                                    "    border-bottom: 2px solid #cccccc;\n"
                                    "}\n"
                                    "")

        self.verticalLayout.addWidget(self.label_10)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_8 = QLabel(Form)
        self.label_8.setObjectName(u"label_8")
        sizePolicy1 = QSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(
            self.label_8.sizePolicy().hasHeightForWidth())
        self.label_8.setSizePolicy(sizePolicy1)
        self.label_8.setStyleSheet(u"QLabel {\n"
                                   "    color: #555;\n"
                                   "    font-size: 16px;\n"
                                   "    font-weight: bold;\n"
                                   "    border: none;\n"
                                   "}")

        self.horizontalLayout_3.addWidget(self.label_8)

        self.comboBox = QComboBox(Form)
        self.comboBox.setObjectName(u"comboBox")
        sizePolicy2 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Maximum)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(
            self.comboBox.sizePolicy().hasHeightForWidth())
        self.comboBox.setSizePolicy(sizePolicy2)
        self.comboBox.setStyleSheet(u"QComboBox {\n"
                                    "    border: 1px solid gray;\n"
                                    "    border-radius: 5px;\n"
                                    "    padding: 1px 18px 1px 3px;\n"
                                    "    min-width: 6em;\n"
                                    "    font-size: 14px;\n"
                                    "    font-weight: bold;\n"
                                    "    color: #333333;\n"
                                    "    background-color: white;\n"
                                    "    selection-background-color: #DA2E2E;\n"
                                    "}\n"
                                    "\n"
                                    "QComboBox::drop-down {\n"
                                    "    subcontrol-origin: padding;\n"
                                    "    subcontrol-position: top right;\n"
                                    "    width: 20px;\n"
                                    "    border-left-width: 1px;\n"
                                    "    border-left-color: gray;\n"
                                    "    border-left-style: solid;\n"
                                    "    border-top-right-radius: 5px;\n"
                                    "    border-bottom-right-radius: 5px;\n"
                                    "}\n"
                                    "\n"
                                    "QComboBox::hover {\n"
                                    "    border: 1px solid #DA2E2E;\n"
                                    "}\n"
                                    "\n"
                                    "QComboBox::pressed {\n"
                                    "    border: 1px solid #DA2E2E;\n"
                                    "    background-color: #DA2E2E;\n"
                                    "    color: white;\n"
                                    "}")

        self.horizontalLayout_3.addWidget(self.comboBox)

        self.verticalLayout.addLayout(self.horizontalLayout_3)

        self.verticalLayout_4.addLayout(self.verticalLayout)

        self.line = QFrame(Form)
        self.line.setObjectName(u"line")
        self.line.setLineWidth(4)
        self.line.setFrameShape(QFrame.HLine)
        self.line.setFrameShadow(QFrame.Sunken)

        self.verticalLayout_4.addWidget(self.line)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.label = QLabel(Form)
        self.label.setObjectName(u"label")
        font = QFont()
        font.setFamilies([u"Arial"])
        font.setBold(True)
        self.label.setFont(font)
        self.label.setStyleSheet(u"QLabel {\n"
                                 "    font-size: 25px;\n"
                                 "    font-weight: bold;\n"
                                 "    color: #333333;\n"
                                 "    background-color: #f2f2f2;\n"
                                 "    padding: 4px;\n"
                                 "    border-bottom: 2px solid #cccccc;\n"
                                 "}\n"
                                 "")

        self.verticalLayout_2.addWidget(self.label)

        self.label_2 = QLabel(Form)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setStyleSheet(u"QLabel {\n"
                                   "    color: #555;\n"
                                   "    font-size: 16px;\n"
                                   "    font-weight: bold;\n"
                                   "    border: none;\n"
                                   "}")

        self.verticalLayout_2.addWidget(self.label_2)

        self.doubleSpinBox_3 = QDoubleSpinBox(Form)
        self.doubleSpinBox_3.setObjectName(u"doubleSpinBox_3")
        self.doubleSpinBox_3.setStyleSheet(u"QDoubleSpinBox {\n"
                                           "    border: 1px solid #d9d9d9;\n"
                                           "    border-radius: 3px;\n"
                                           "    padding: 3px;\n"
                                           "    font-size: 14px;\n"
                                           "    font-family: Arial;\n"
                                           "    color: #4d4d4d;\n"
                                           "}\n"
                                           "")

        self.verticalLayout_2.addWidget(self.doubleSpinBox_3)

        self.label_3 = QLabel(Form)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setStyleSheet(u"QLabel {\n"
                                   "    color: #555;\n"
                                   "    font-size: 16px;\n"
                                   "    font-weight: bold;\n"
                                   "    border: none;\n"
                                   "}")

        self.verticalLayout_2.addWidget(self.label_3)

        self.doubleSpinBox_2 = QDoubleSpinBox(Form)
        self.doubleSpinBox_2.setObjectName(u"doubleSpinBox_2")
        self.doubleSpinBox_2.setStyleSheet(u"QDoubleSpinBox {\n"
                                           "    border: 1px solid #d9d9d9;\n"
                                           "    border-radius: 3px;\n"
                                           "    padding: 3px;\n"
                                           "    font-size: 14px;\n"
                                           "    font-family: Arial;\n"
                                           "    color: #4d4d4d;\n"
                                           "}")

        self.verticalLayout_2.addWidget(self.doubleSpinBox_2)

        self.label_4 = QLabel(Form)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setStyleSheet(u"QLabel {\n"
                                   "    color: #555;\n"
                                   "    font-size: 16px;\n"
                                   "    font-weight: bold;\n"
                                   "    border: none;\n"
                                   "}")

        self.verticalLayout_2.addWidget(self.label_4)

        self.doubleSpinBox = QDoubleSpinBox(Form)
        self.doubleSpinBox.setObjectName(u"doubleSpinBox")
        self.doubleSpinBox.setStyleSheet(u"QDoubleSpinBox {\n"
                                         "    border: 1px solid #d9d9d9;\n"
                                         "    border-radius: 3px;\n"
                                         "    padding: 3px;\n"
                                         "    font-size: 14px;\n"
                                         "    font-family: Arial;\n"
                                         "    color: #4d4d4d;\n"
                                         "}\n"
                                         "\n"
                                         "")

        self.verticalLayout_2.addWidget(self.doubleSpinBox)

        self.pushButton = QPushButton(Form)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setMinimumSize(QSize(128, 50))
        font1 = QFont()
        font1.setFamilies([u"Arial"])
        font1.setBold(True)
        font1.setItalic(False)
        self.pushButton.setFont(font1)
        self.pushButton.setStyleSheet(u"QPushButton{\n"
                                      "    background-color: rgb(53, 132, 228);\n"
                                      "    border-style: outset;\n"
                                      "    border-width: 2px;\n"
                                      "    border-radius: 10px;\n"
                                      "    border-color: beige;\n"
                                      "    font: bold 25px;\n"
                                      "	color: white;\n"
                                      "    min-width: 4em;\n"
                                      "    padding: 4px;\n"
                                      "}")
        self.pushButton.setIconSize(QSize(19, 100))

        self.verticalLayout_2.addWidget(self.pushButton)

        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.label_5 = QLabel(Form)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setStyleSheet(u"QLabel {\n"
                                   "    font-size: 25px;\n"
                                   "    font-weight: bold;\n"
                                   "    color: #333333;\n"
                                   "    background-color: #f2f2f2;\n"
                                   "    padding: 4px;\n"
                                   "    border-bottom: 2px solid #cccccc;\n"
                                   "}\n"
                                   "")

        self.verticalLayout_3.addWidget(self.label_5)

        self.label_6 = QLabel(Form)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setStyleSheet(u"QLabel {\n"
                                   "    color: #555;\n"
                                   "    font-size: 16px;\n"
                                   "    font-weight: bold;\n"
                                   "    border: none;\n"
                                   "}")

        self.verticalLayout_3.addWidget(self.label_6)

        self.lcdNumber_2 = QLCDNumber(Form)
        self.lcdNumber_2.setObjectName(u"lcdNumber_2")
        self.lcdNumber_2.setStyleSheet(u"QLCDNumber {\n"
                                       "    background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #434343, stop:1 #2b2b2b);\n"
                                       "    color: #b3b3b3;\n"
                                       "    border-radius: 10px;\n"
                                       "    padding: 5px;\n"
                                       "    border: none;\n"
                                       "}\n"
                                       "QLCDNumber::number {\n"
                                       "    background-color: none;\n"
                                       "}")

        self.verticalLayout_3.addWidget(self.lcdNumber_2)

        self.label_7 = QLabel(Form)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setStyleSheet(u"QLabel {\n"
                                   "    color: #555;\n"
                                   "    font-size: 16px;\n"
                                   "    font-weight: bold;\n"
                                   "    border: none;\n"
                                   "}")

        self.verticalLayout_3.addWidget(self.label_7)

        self.lcdNumber = QLCDNumber(Form)
        self.lcdNumber.setObjectName(u"lcdNumber")
        self.lcdNumber.setStyleSheet(u"QLCDNumber {\n"
                                     "    background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #434343, stop:1 #2b2b2b);\n"
                                     "    color: #b3b3b3;\n"
                                     "    border-radius: 10px;\n"
                                     "    padding: 5px;\n"
                                     "    border: none;\n"
                                     "}\n"
                                     "QLCDNumber::number {\n"
                                     "    background-color: none;\n"
                                     "}")

        self.verticalLayout_3.addWidget(self.lcdNumber)

        self.label_9 = QLabel(Form)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setStyleSheet(u"QLabel {\n"
                                   "    color: #555;\n"
                                   "    font-size: 16px;\n"
                                   "    font-weight: bold;\n"
                                   "    border: none;\n"
                                   "}\n"
                                   "")

        self.verticalLayout_3.addWidget(self.label_9)

        self.lcdNumber_3 = QLCDNumber(Form)
        self.lcdNumber_3.setObjectName(u"lcdNumber_3")
        self.lcdNumber_3.setStyleSheet(u"QLCDNumber {\n"
                                       "    background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #434343, stop:1 #2b2b2b);\n"
                                       "    color: #b3b3b3;\n"
                                       "    border-radius: 10px;\n"
                                       "    padding: 5px;\n"
                                       "    border: none;\n"
                                       "}\n"
                                       "QLCDNumber::number {\n"
                                       "    background-color: none;\n"
                                       "}\n"
                                       "")

        self.verticalLayout_3.addWidget(self.lcdNumber_3)

        self.pushButton_2 = QPushButton(Form)
        self.pushButton_2.setObjectName(u"pushButton_2")
        self.pushButton_2.setMinimumSize(QSize(132, 50))
        self.pushButton_2.setFont(font1)
        self.pushButton_2.setStyleSheet(u"QPushButton{\n"
                                        "    background-color: rgb(224, 27, 36);\n"
                                        "    border-style: outset;\n"
                                        "    border-width: 2px;\n"
                                        "    border-radius: 10px;\n"
                                        "    border-color: beige;\n"
                                        "    font: bold 25px;\n"
                                        "	color: white;\n"
                                        "    min-width: 4em;\n"
                                        "    padding: 6px;\n"
                                        "}")
        self.pushButton_2.setIconSize(QSize(16, 100))
        self.pushButton_2.setAutoRepeatInterval(100)

        self.verticalLayout_3.addWidget(self.pushButton_2)

        self.horizontalLayout.addLayout(self.verticalLayout_3)

        self.verticalLayout_4.addLayout(self.horizontalLayout)

        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.label_10.setText(
            QCoreApplication.translate("Form", u"Moteus UI", None))
        self.label_8.setText(QCoreApplication.translate(
            "Form", u"Choose Moteus:", None))
        self.label.setText(QCoreApplication.translate(
            "Form", u"Moteus Control", None))
        self.label_2.setText(
            QCoreApplication.translate("Form", u"position", None))
        self.label_3.setText(
            QCoreApplication.translate("Form", u"velocity", None))
        self.label_4.setText(
            QCoreApplication.translate("Form", u"torque", None))
        self.pushButton.setText(
            QCoreApplication.translate("Form", u"SET", None))
        self.label_5.setText(QCoreApplication.translate(
            "Form", u"Moteus State", None))
        self.label_6.setText(QCoreApplication.translate(
            "Form", u"current position", None))
        self.label_7.setText(QCoreApplication.translate(
            "Form", u"current velocity", None))
        self.label_9.setText(QCoreApplication.translate(
            "Form", u"current torque", None))
        self.pushButton_2.setText(
            QCoreApplication.translate("Form", u"STOP", None))
    # retranslateUi
