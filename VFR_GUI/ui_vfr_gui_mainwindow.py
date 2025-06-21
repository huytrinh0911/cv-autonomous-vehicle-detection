# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'vfr_gui_mainwindow.ui'
##
## Created by: Qt User Interface Compiler version 6.8.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QGridLayout, QGroupBox,
    QHeaderView, QLabel, QMainWindow, QMenu,
    QMenuBar, QPushButton, QSizePolicy, QStatusBar,
    QTabWidget, QTableWidget, QTableWidgetItem, QTextBrowser,
    QTextEdit, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1920, 1030)
        MainWindow.setMaximumSize(QSize(1920, 1030))
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabWidget.setGeometry(QRect(10, 10, 1901, 971))
        self.tab_user = QWidget()
        self.tab_user.setObjectName(u"tab_user")
        self.tab_user.setEnabled(True)
        self.group_connectivity = QGroupBox(self.tab_user)
        self.group_connectivity.setObjectName(u"group_connectivity")
        self.group_connectivity.setGeometry(QRect(10, 0, 441, 201))
        font = QFont()
        font.setPointSize(13)
        self.group_connectivity.setFont(font)
        self.combobox_port_list = QComboBox(self.group_connectivity)
        self.combobox_port_list.setObjectName(u"combobox_port_list")
        self.combobox_port_list.setGeometry(QRect(130, 30, 281, 31))
        self.btn_port_refresh = QPushButton(self.group_connectivity)
        self.btn_port_refresh.setObjectName(u"btn_port_refresh")
        self.btn_port_refresh.setGeometry(QRect(20, 70, 101, 24))
        self.outmsg_controller_connection_status = QTextBrowser(self.group_connectivity)
        self.outmsg_controller_connection_status.setObjectName(u"outmsg_controller_connection_status")
        self.outmsg_controller_connection_status.setGeometry(QRect(130, 110, 171, 31))
        self.outmsg_jetson_tx2_connection_status = QTextBrowser(self.group_connectivity)
        self.outmsg_jetson_tx2_connection_status.setObjectName(u"outmsg_jetson_tx2_connection_status")
        self.outmsg_jetson_tx2_connection_status.setGeometry(QRect(130, 150, 171, 31))
        self.label_controller = QLabel(self.group_connectivity)
        self.label_controller.setObjectName(u"label_controller")
        self.label_controller.setGeometry(QRect(20, 110, 101, 31))
        self.label_controller.setFont(font)
        self.label_jetson_tx2 = QLabel(self.group_connectivity)
        self.label_jetson_tx2.setObjectName(u"label_jetson_tx2")
        self.label_jetson_tx2.setGeometry(QRect(20, 150, 101, 31))
        self.label_jetson_tx2.setFont(font)
        self.btn_port_connect = QPushButton(self.group_connectivity)
        self.btn_port_connect.setObjectName(u"btn_port_connect")
        self.btn_port_connect.setGeometry(QRect(300, 70, 111, 24))
        self.label_port_list = QLabel(self.group_connectivity)
        self.label_port_list.setObjectName(u"label_port_list")
        self.label_port_list.setGeometry(QRect(20, 30, 101, 31))
        self.label_port_list.setFont(font)
        self.btn_port_disconnect = QPushButton(self.group_connectivity)
        self.btn_port_disconnect.setObjectName(u"btn_port_disconnect")
        self.btn_port_disconnect.setGeometry(QRect(140, 70, 141, 24))
        self.group_robot_status = QGroupBox(self.tab_user)
        self.group_robot_status.setObjectName(u"group_robot_status")
        self.group_robot_status.setGeometry(QRect(10, 210, 441, 461))
        self.group_robot_status.setFont(font)
        self.inmsg_robot_max_speed = QTextEdit(self.group_robot_status)
        self.inmsg_robot_max_speed.setObjectName(u"inmsg_robot_max_speed")
        self.inmsg_robot_max_speed.setGeometry(QRect(220, 370, 111, 31))
        self.outmsg_robot_current_speed = QTextBrowser(self.group_robot_status)
        self.outmsg_robot_current_speed.setObjectName(u"outmsg_robot_current_speed")
        self.outmsg_robot_current_speed.setGeometry(QRect(220, 330, 111, 31))
        self.label_current_speed = QLabel(self.group_robot_status)
        self.label_current_speed.setObjectName(u"label_current_speed")
        self.label_current_speed.setGeometry(QRect(20, 330, 121, 31))
        self.label_current_speed.setFont(font)
        self.label_max_speed = QLabel(self.group_robot_status)
        self.label_max_speed.setObjectName(u"label_max_speed")
        self.label_max_speed.setGeometry(QRect(20, 370, 121, 31))
        self.label_max_speed.setFont(font)
        self.label_unit = QLabel(self.group_robot_status)
        self.label_unit.setObjectName(u"label_unit")
        self.label_unit.setGeometry(QRect(350, 290, 61, 31))
        self.label_unit.setFont(font)
        self.btn_robot_status_update = QPushButton(self.group_robot_status)
        self.btn_robot_status_update.setObjectName(u"btn_robot_status_update")
        self.btn_robot_status_update.setGeometry(QRect(220, 410, 111, 31))
        self.outmsg_robot_process_state = QTextBrowser(self.group_robot_status)
        self.outmsg_robot_process_state.setObjectName(u"outmsg_robot_process_state")
        self.outmsg_robot_process_state.setGeometry(QRect(220, 40, 201, 31))
        self.label_robot_process_state = QLabel(self.group_robot_status)
        self.label_robot_process_state.setObjectName(u"label_robot_process_state")
        self.label_robot_process_state.setGeometry(QRect(20, 40, 121, 31))
        self.label_robot_process_state.setFont(font)
        self.outmsg_robot_current_setpoint_speed = QTextBrowser(self.group_robot_status)
        self.outmsg_robot_current_setpoint_speed.setObjectName(u"outmsg_robot_current_setpoint_speed")
        self.outmsg_robot_current_setpoint_speed.setGeometry(QRect(220, 290, 111, 31))
        self.label_current_setpoint_speed = QLabel(self.group_robot_status)
        self.label_current_setpoint_speed.setObjectName(u"label_current_setpoint_speed")
        self.label_current_setpoint_speed.setGeometry(QRect(20, 290, 191, 31))
        self.label_current_setpoint_speed.setFont(font)
        self.label_unit_2 = QLabel(self.group_robot_status)
        self.label_unit_2.setObjectName(u"label_unit_2")
        self.label_unit_2.setGeometry(QRect(350, 330, 61, 31))
        self.label_unit_2.setFont(font)
        self.outmsg_robot_curren_rotation_angle = QTextBrowser(self.group_robot_status)
        self.outmsg_robot_curren_rotation_angle.setObjectName(u"outmsg_robot_curren_rotation_angle")
        self.outmsg_robot_curren_rotation_angle.setGeometry(QRect(220, 160, 111, 31))
        self.label_robot_curren_rotation_angle = QLabel(self.group_robot_status)
        self.label_robot_curren_rotation_angle.setObjectName(u"label_robot_curren_rotation_angle")
        self.label_robot_curren_rotation_angle.setGeometry(QRect(20, 160, 181, 31))
        self.label_robot_curren_rotation_angle.setFont(font)
        self.label_unit_3 = QLabel(self.group_robot_status)
        self.label_unit_3.setObjectName(u"label_unit_3")
        self.label_unit_3.setGeometry(QRect(350, 370, 61, 31))
        self.label_unit_3.setFont(font)
        self.label_unit_4 = QLabel(self.group_robot_status)
        self.label_unit_4.setObjectName(u"label_unit_4")
        self.label_unit_4.setGeometry(QRect(350, 160, 61, 31))
        self.label_unit_4.setFont(font)
        self.label_robot_max_rotation_angle = QLabel(self.group_robot_status)
        self.label_robot_max_rotation_angle.setObjectName(u"label_robot_max_rotation_angle")
        self.label_robot_max_rotation_angle.setGeometry(QRect(20, 200, 181, 31))
        self.label_robot_max_rotation_angle.setFont(font)
        self.label_unit_5 = QLabel(self.group_robot_status)
        self.label_unit_5.setObjectName(u"label_unit_5")
        self.label_unit_5.setGeometry(QRect(350, 200, 61, 31))
        self.label_unit_5.setFont(font)
        self.inmsg_max_rotation_angle = QTextEdit(self.group_robot_status)
        self.inmsg_max_rotation_angle.setObjectName(u"inmsg_max_rotation_angle")
        self.inmsg_max_rotation_angle.setGeometry(QRect(220, 200, 111, 31))
        self.label_robot_max_rotation_angle_2 = QLabel(self.group_robot_status)
        self.label_robot_max_rotation_angle_2.setObjectName(u"label_robot_max_rotation_angle_2")
        self.label_robot_max_rotation_angle_2.setGeometry(QRect(10, 240, 421, 31))
        self.label_robot_max_rotation_angle_2.setFont(font)
        self.label_robot_max_rotation_angle_2.setAlignment(Qt.AlignCenter)
        self.outmsg_robot_direct_state = QTextBrowser(self.group_robot_status)
        self.outmsg_robot_direct_state.setObjectName(u"outmsg_robot_direct_state")
        self.outmsg_robot_direct_state.setGeometry(QRect(220, 80, 201, 31))
        self.label_robot_direct_state = QLabel(self.group_robot_status)
        self.label_robot_direct_state.setObjectName(u"label_robot_direct_state")
        self.label_robot_direct_state.setGeometry(QRect(20, 80, 121, 31))
        self.label_robot_direct_state.setFont(font)
        self.label_robot_camera_state = QLabel(self.group_robot_status)
        self.label_robot_camera_state.setObjectName(u"label_robot_camera_state")
        self.label_robot_camera_state.setGeometry(QRect(20, 120, 121, 31))
        self.label_robot_camera_state.setFont(font)
        self.outmsg_robot_camera_state = QTextBrowser(self.group_robot_status)
        self.outmsg_robot_camera_state.setObjectName(u"outmsg_robot_camera_state")
        self.outmsg_robot_camera_state.setGeometry(QRect(220, 120, 201, 31))
        self.group_msg = QGroupBox(self.tab_user)
        self.group_msg.setObjectName(u"group_msg")
        self.group_msg.setGeometry(QRect(470, 0, 1421, 921))
        self.group_msg.setFont(font)
        self.outmsg_robot_msg = QTextBrowser(self.group_msg)
        self.outmsg_robot_msg.setObjectName(u"outmsg_robot_msg")
        self.outmsg_robot_msg.setGeometry(QRect(10, 60, 1401, 321))
        font1 = QFont()
        font1.setPointSize(11)
        self.outmsg_robot_msg.setFont(font1)
        self.label_msg_notif = QLabel(self.group_msg)
        self.label_msg_notif.setObjectName(u"label_msg_notif")
        self.label_msg_notif.setGeometry(QRect(10, 390, 191, 31))
        self.label_msg_notif.setFont(font)
        self.label_action_list = QLabel(self.group_msg)
        self.label_action_list.setObjectName(u"label_action_list")
        self.label_action_list.setGeometry(QRect(10, 600, 101, 31))
        self.label_action_list.setFont(font)
        self.btn_clear_robot_msg = QPushButton(self.group_msg)
        self.btn_clear_robot_msg.setObjectName(u"btn_clear_robot_msg")
        self.btn_clear_robot_msg.setGeometry(QRect(130, 590, 71, 31))
        self.gridLayoutWidget = QWidget(self.group_msg)
        self.gridLayoutWidget.setObjectName(u"gridLayoutWidget")
        self.gridLayoutWidget.setGeometry(QRect(340, 585, 1071, 311))
        self.gridLayout_signs = QGridLayout(self.gridLayoutWidget)
        self.gridLayout_signs.setObjectName(u"gridLayout_signs")
        self.gridLayout_signs.setContentsMargins(0, 0, 0, 0)
        self.widget_signs = QWidget(self.gridLayoutWidget)
        self.widget_signs.setObjectName(u"widget_signs")
        self.gridLayout_2 = QGridLayout(self.widget_signs)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.label_signs_06 = QLabel(self.widget_signs)
        self.label_signs_06.setObjectName(u"label_signs_06")

        self.gridLayout_2.addWidget(self.label_signs_06, 2, 1, 1, 1)

        self.label_signs_02 = QLabel(self.widget_signs)
        self.label_signs_02.setObjectName(u"label_signs_02")

        self.gridLayout_2.addWidget(self.label_signs_02, 1, 1, 1, 1)

        self.label_signs_01 = QLabel(self.widget_signs)
        self.label_signs_01.setObjectName(u"label_signs_01")

        self.gridLayout_2.addWidget(self.label_signs_01, 1, 0, 1, 1)

        self.label_signs_03 = QLabel(self.widget_signs)
        self.label_signs_03.setObjectName(u"label_signs_03")

        self.gridLayout_2.addWidget(self.label_signs_03, 1, 2, 1, 1)

        self.label_signs_05 = QLabel(self.widget_signs)
        self.label_signs_05.setObjectName(u"label_signs_05")

        self.gridLayout_2.addWidget(self.label_signs_05, 2, 0, 1, 1)

        self.label_signs_07 = QLabel(self.widget_signs)
        self.label_signs_07.setObjectName(u"label_signs_07")

        self.gridLayout_2.addWidget(self.label_signs_07, 2, 2, 1, 1)

        self.label_signs_04 = QLabel(self.widget_signs)
        self.label_signs_04.setObjectName(u"label_signs_04")

        self.gridLayout_2.addWidget(self.label_signs_04, 1, 3, 1, 1)

        self.label_signs_08 = QLabel(self.widget_signs)
        self.label_signs_08.setObjectName(u"label_signs_08")

        self.gridLayout_2.addWidget(self.label_signs_08, 2, 3, 1, 1)


        self.gridLayout_signs.addWidget(self.widget_signs, 0, 0, 1, 1)

        self.outmsg_action_list = QTextBrowser(self.group_msg)
        self.outmsg_action_list.setObjectName(u"outmsg_action_list")
        self.outmsg_action_list.setGeometry(QRect(10, 630, 291, 281))
        self.outmsg_action_list.setFont(font1)
        self.outmsg_robot_msg_notif = QTextBrowser(self.group_msg)
        self.outmsg_robot_msg_notif.setObjectName(u"outmsg_robot_msg_notif")
        self.outmsg_robot_msg_notif.setGeometry(QRect(10, 420, 1401, 141))
        self.outmsg_robot_msg_notif.setFont(font1)
        self.label_msg = QLabel(self.group_msg)
        self.label_msg.setObjectName(u"label_msg")
        self.label_msg.setGeometry(QRect(10, 30, 191, 31))
        self.label_msg.setFont(font)
        self.group_robot_control = QGroupBox(self.tab_user)
        self.group_robot_control.setObjectName(u"group_robot_control")
        self.group_robot_control.setEnabled(True)
        self.group_robot_control.setGeometry(QRect(0, 680, 441, 231))
        self.group_robot_control.setFont(font)
        self.buttuon_turn_left = QPushButton(self.group_robot_control)
        self.buttuon_turn_left.setObjectName(u"buttuon_turn_left")
        self.buttuon_turn_left.setGeometry(QRect(100, 110, 71, 31))
        self.buttuon_forward = QPushButton(self.group_robot_control)
        self.buttuon_forward.setObjectName(u"buttuon_forward")
        self.buttuon_forward.setGeometry(QRect(180, 70, 71, 31))
        self.button_turn_right = QPushButton(self.group_robot_control)
        self.button_turn_right.setObjectName(u"button_turn_right")
        self.button_turn_right.setGeometry(QRect(260, 110, 81, 31))
        self.button_start = QPushButton(self.group_robot_control)
        self.button_start.setObjectName(u"button_start")
        self.button_start.setGeometry(QRect(170, 150, 91, 31))
        self.button_stop = QPushButton(self.group_robot_control)
        self.button_stop.setObjectName(u"button_stop")
        self.button_stop.setGeometry(QRect(180, 110, 71, 31))
        self.tabWidget.addTab(self.tab_user, "")
        self.tab_graph = QWidget()
        self.tab_graph.setObjectName(u"tab_graph")
        self.groupBox = QGroupBox(self.tab_graph)
        self.groupBox.setObjectName(u"groupBox")
        self.groupBox.setGeometry(QRect(10, 20, 1881, 881))
        self.graph_error = QWidget(self.groupBox)
        self.graph_error.setObjectName(u"graph_error")
        self.graph_error.setGeometry(QRect(20, 60, 821, 371))
        self.graph_angle = QWidget(self.groupBox)
        self.graph_angle.setObjectName(u"graph_angle")
        self.graph_angle.setGeometry(QRect(20, 490, 821, 371))
        self.graph_phi = QWidget(self.groupBox)
        self.graph_phi.setObjectName(u"graph_phi")
        self.graph_phi.setGeometry(QRect(860, 60, 821, 371))
        self.graph_speed = QWidget(self.groupBox)
        self.graph_speed.setObjectName(u"graph_speed")
        self.graph_speed.setGeometry(QRect(860, 490, 821, 371))
        self.graph_legend_static = QWidget(self.groupBox)
        self.graph_legend_static.setObjectName(u"graph_legend_static")
        self.graph_legend_static.setGeometry(QRect(1700, 60, 161, 751))
        self.button_stop_nd = QPushButton(self.groupBox)
        self.button_stop_nd.setObjectName(u"button_stop_nd")
        self.button_stop_nd.setGeometry(QRect(1790, 830, 71, 31))
        self.button_start_nd = QPushButton(self.groupBox)
        self.button_start_nd.setObjectName(u"button_start_nd")
        self.button_start_nd.setGeometry(QRect(1710, 830, 71, 31))
        self.tabWidget.addTab(self.tab_graph, "")
        self.tab_programmer = QWidget()
        self.tab_programmer.setObjectName(u"tab_programmer")
        self.group_def_input = QGroupBox(self.tab_programmer)
        self.group_def_input.setObjectName(u"group_def_input")
        self.group_def_input.setGeometry(QRect(20, 10, 1491, 461))
        self.group_def_input.setFont(font)
        self.table_def_in_error = QTableWidget(self.group_def_input)
        if (self.table_def_in_error.columnCount() < 4):
            self.table_def_in_error.setColumnCount(4)
        __qtablewidgetitem = QTableWidgetItem()
        self.table_def_in_error.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.table_def_in_error.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.table_def_in_error.setHorizontalHeaderItem(2, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.table_def_in_error.setHorizontalHeaderItem(3, __qtablewidgetitem3)
        if (self.table_def_in_error.rowCount() < 5):
            self.table_def_in_error.setRowCount(5)
        __qtablewidgetitem4 = QTableWidgetItem()
        self.table_def_in_error.setVerticalHeaderItem(0, __qtablewidgetitem4)
        __qtablewidgetitem5 = QTableWidgetItem()
        self.table_def_in_error.setVerticalHeaderItem(1, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        self.table_def_in_error.setVerticalHeaderItem(2, __qtablewidgetitem6)
        __qtablewidgetitem7 = QTableWidgetItem()
        self.table_def_in_error.setVerticalHeaderItem(3, __qtablewidgetitem7)
        __qtablewidgetitem8 = QTableWidgetItem()
        self.table_def_in_error.setVerticalHeaderItem(4, __qtablewidgetitem8)
        __qtablewidgetitem9 = QTableWidgetItem()
        __qtablewidgetitem9.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(0, 0, __qtablewidgetitem9)
        __qtablewidgetitem10 = QTableWidgetItem()
        __qtablewidgetitem10.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(0, 1, __qtablewidgetitem10)
        __qtablewidgetitem11 = QTableWidgetItem()
        __qtablewidgetitem11.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(0, 2, __qtablewidgetitem11)
        __qtablewidgetitem12 = QTableWidgetItem()
        __qtablewidgetitem12.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(0, 3, __qtablewidgetitem12)
        __qtablewidgetitem13 = QTableWidgetItem()
        __qtablewidgetitem13.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(1, 0, __qtablewidgetitem13)
        __qtablewidgetitem14 = QTableWidgetItem()
        __qtablewidgetitem14.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(1, 1, __qtablewidgetitem14)
        __qtablewidgetitem15 = QTableWidgetItem()
        __qtablewidgetitem15.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(1, 2, __qtablewidgetitem15)
        __qtablewidgetitem16 = QTableWidgetItem()
        __qtablewidgetitem16.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(1, 3, __qtablewidgetitem16)
        __qtablewidgetitem17 = QTableWidgetItem()
        __qtablewidgetitem17.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(2, 0, __qtablewidgetitem17)
        __qtablewidgetitem18 = QTableWidgetItem()
        __qtablewidgetitem18.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(2, 1, __qtablewidgetitem18)
        __qtablewidgetitem19 = QTableWidgetItem()
        __qtablewidgetitem19.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(2, 2, __qtablewidgetitem19)
        __qtablewidgetitem20 = QTableWidgetItem()
        __qtablewidgetitem20.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(2, 3, __qtablewidgetitem20)
        __qtablewidgetitem21 = QTableWidgetItem()
        __qtablewidgetitem21.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(3, 0, __qtablewidgetitem21)
        __qtablewidgetitem22 = QTableWidgetItem()
        __qtablewidgetitem22.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(3, 1, __qtablewidgetitem22)
        __qtablewidgetitem23 = QTableWidgetItem()
        __qtablewidgetitem23.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(3, 2, __qtablewidgetitem23)
        __qtablewidgetitem24 = QTableWidgetItem()
        __qtablewidgetitem24.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(3, 3, __qtablewidgetitem24)
        __qtablewidgetitem25 = QTableWidgetItem()
        __qtablewidgetitem25.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(4, 0, __qtablewidgetitem25)
        __qtablewidgetitem26 = QTableWidgetItem()
        __qtablewidgetitem26.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(4, 1, __qtablewidgetitem26)
        __qtablewidgetitem27 = QTableWidgetItem()
        __qtablewidgetitem27.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(4, 2, __qtablewidgetitem27)
        __qtablewidgetitem28 = QTableWidgetItem()
        __qtablewidgetitem28.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_error.setItem(4, 3, __qtablewidgetitem28)
        self.table_def_in_error.setObjectName(u"table_def_in_error")
        self.table_def_in_error.setGeometry(QRect(20, 60, 441, 201))
        self.table_def_in_error.setAlternatingRowColors(True)
        self.table_def_in_error.horizontalHeader().setDefaultSectionSize(100)
        self.table_def_in_speed = QTableWidget(self.group_def_input)
        if (self.table_def_in_speed.columnCount() < 4):
            self.table_def_in_speed.setColumnCount(4)
        __qtablewidgetitem29 = QTableWidgetItem()
        self.table_def_in_speed.setHorizontalHeaderItem(0, __qtablewidgetitem29)
        __qtablewidgetitem30 = QTableWidgetItem()
        self.table_def_in_speed.setHorizontalHeaderItem(1, __qtablewidgetitem30)
        __qtablewidgetitem31 = QTableWidgetItem()
        self.table_def_in_speed.setHorizontalHeaderItem(2, __qtablewidgetitem31)
        __qtablewidgetitem32 = QTableWidgetItem()
        self.table_def_in_speed.setHorizontalHeaderItem(3, __qtablewidgetitem32)
        if (self.table_def_in_speed.rowCount() < 3):
            self.table_def_in_speed.setRowCount(3)
        __qtablewidgetitem33 = QTableWidgetItem()
        self.table_def_in_speed.setVerticalHeaderItem(0, __qtablewidgetitem33)
        __qtablewidgetitem34 = QTableWidgetItem()
        self.table_def_in_speed.setVerticalHeaderItem(1, __qtablewidgetitem34)
        __qtablewidgetitem35 = QTableWidgetItem()
        self.table_def_in_speed.setVerticalHeaderItem(2, __qtablewidgetitem35)
        __qtablewidgetitem36 = QTableWidgetItem()
        __qtablewidgetitem36.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(0, 0, __qtablewidgetitem36)
        __qtablewidgetitem37 = QTableWidgetItem()
        __qtablewidgetitem37.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(0, 1, __qtablewidgetitem37)
        __qtablewidgetitem38 = QTableWidgetItem()
        __qtablewidgetitem38.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(0, 2, __qtablewidgetitem38)
        __qtablewidgetitem39 = QTableWidgetItem()
        __qtablewidgetitem39.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(0, 3, __qtablewidgetitem39)
        __qtablewidgetitem40 = QTableWidgetItem()
        __qtablewidgetitem40.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(1, 0, __qtablewidgetitem40)
        __qtablewidgetitem41 = QTableWidgetItem()
        __qtablewidgetitem41.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(1, 1, __qtablewidgetitem41)
        __qtablewidgetitem42 = QTableWidgetItem()
        __qtablewidgetitem42.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(1, 2, __qtablewidgetitem42)
        __qtablewidgetitem43 = QTableWidgetItem()
        __qtablewidgetitem43.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(1, 3, __qtablewidgetitem43)
        __qtablewidgetitem44 = QTableWidgetItem()
        __qtablewidgetitem44.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(2, 0, __qtablewidgetitem44)
        __qtablewidgetitem45 = QTableWidgetItem()
        __qtablewidgetitem45.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(2, 1, __qtablewidgetitem45)
        __qtablewidgetitem46 = QTableWidgetItem()
        __qtablewidgetitem46.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(2, 2, __qtablewidgetitem46)
        __qtablewidgetitem47 = QTableWidgetItem()
        __qtablewidgetitem47.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_speed.setItem(2, 3, __qtablewidgetitem47)
        self.table_def_in_speed.setObjectName(u"table_def_in_speed")
        self.table_def_in_speed.setGeometry(QRect(480, 290, 441, 141))
        self.table_def_in_speed.setAlternatingRowColors(True)
        self.table_def_in_speed.horizontalHeader().setDefaultSectionSize(100)
        self.label_in_error = QLabel(self.group_def_input)
        self.label_in_error.setObjectName(u"label_in_error")
        self.label_in_error.setGeometry(QRect(20, 35, 131, 21))
        self.label_in_derivative_error = QLabel(self.group_def_input)
        self.label_in_derivative_error.setObjectName(u"label_in_derivative_error")
        self.label_in_derivative_error.setGeometry(QRect(480, 35, 151, 21))
        self.label_in_rotation_angle = QLabel(self.group_def_input)
        self.label_in_rotation_angle.setObjectName(u"label_in_rotation_angle")
        self.label_in_rotation_angle.setGeometry(QRect(940, 30, 221, 31))
        self.label_in_speed = QLabel(self.group_def_input)
        self.label_in_speed.setObjectName(u"label_in_speed")
        self.label_in_speed.setGeometry(QRect(480, 260, 211, 31))
        self.table_def_in_derivative_error = QTableWidget(self.group_def_input)
        if (self.table_def_in_derivative_error.columnCount() < 4):
            self.table_def_in_derivative_error.setColumnCount(4)
        __qtablewidgetitem48 = QTableWidgetItem()
        self.table_def_in_derivative_error.setHorizontalHeaderItem(0, __qtablewidgetitem48)
        __qtablewidgetitem49 = QTableWidgetItem()
        self.table_def_in_derivative_error.setHorizontalHeaderItem(1, __qtablewidgetitem49)
        __qtablewidgetitem50 = QTableWidgetItem()
        self.table_def_in_derivative_error.setHorizontalHeaderItem(2, __qtablewidgetitem50)
        __qtablewidgetitem51 = QTableWidgetItem()
        self.table_def_in_derivative_error.setHorizontalHeaderItem(3, __qtablewidgetitem51)
        if (self.table_def_in_derivative_error.rowCount() < 5):
            self.table_def_in_derivative_error.setRowCount(5)
        __qtablewidgetitem52 = QTableWidgetItem()
        self.table_def_in_derivative_error.setVerticalHeaderItem(0, __qtablewidgetitem52)
        __qtablewidgetitem53 = QTableWidgetItem()
        self.table_def_in_derivative_error.setVerticalHeaderItem(1, __qtablewidgetitem53)
        __qtablewidgetitem54 = QTableWidgetItem()
        self.table_def_in_derivative_error.setVerticalHeaderItem(2, __qtablewidgetitem54)
        __qtablewidgetitem55 = QTableWidgetItem()
        self.table_def_in_derivative_error.setVerticalHeaderItem(3, __qtablewidgetitem55)
        __qtablewidgetitem56 = QTableWidgetItem()
        self.table_def_in_derivative_error.setVerticalHeaderItem(4, __qtablewidgetitem56)
        __qtablewidgetitem57 = QTableWidgetItem()
        __qtablewidgetitem57.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(0, 0, __qtablewidgetitem57)
        __qtablewidgetitem58 = QTableWidgetItem()
        __qtablewidgetitem58.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(0, 1, __qtablewidgetitem58)
        __qtablewidgetitem59 = QTableWidgetItem()
        __qtablewidgetitem59.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(0, 2, __qtablewidgetitem59)
        __qtablewidgetitem60 = QTableWidgetItem()
        __qtablewidgetitem60.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(0, 3, __qtablewidgetitem60)
        __qtablewidgetitem61 = QTableWidgetItem()
        __qtablewidgetitem61.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(1, 0, __qtablewidgetitem61)
        __qtablewidgetitem62 = QTableWidgetItem()
        __qtablewidgetitem62.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(1, 1, __qtablewidgetitem62)
        __qtablewidgetitem63 = QTableWidgetItem()
        __qtablewidgetitem63.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(1, 2, __qtablewidgetitem63)
        __qtablewidgetitem64 = QTableWidgetItem()
        __qtablewidgetitem64.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(1, 3, __qtablewidgetitem64)
        __qtablewidgetitem65 = QTableWidgetItem()
        __qtablewidgetitem65.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(2, 0, __qtablewidgetitem65)
        __qtablewidgetitem66 = QTableWidgetItem()
        __qtablewidgetitem66.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(2, 1, __qtablewidgetitem66)
        __qtablewidgetitem67 = QTableWidgetItem()
        __qtablewidgetitem67.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(2, 2, __qtablewidgetitem67)
        __qtablewidgetitem68 = QTableWidgetItem()
        __qtablewidgetitem68.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(2, 3, __qtablewidgetitem68)
        __qtablewidgetitem69 = QTableWidgetItem()
        __qtablewidgetitem69.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(3, 0, __qtablewidgetitem69)
        __qtablewidgetitem70 = QTableWidgetItem()
        __qtablewidgetitem70.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(3, 1, __qtablewidgetitem70)
        __qtablewidgetitem71 = QTableWidgetItem()
        __qtablewidgetitem71.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(3, 2, __qtablewidgetitem71)
        __qtablewidgetitem72 = QTableWidgetItem()
        __qtablewidgetitem72.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(3, 3, __qtablewidgetitem72)
        __qtablewidgetitem73 = QTableWidgetItem()
        __qtablewidgetitem73.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(4, 0, __qtablewidgetitem73)
        __qtablewidgetitem74 = QTableWidgetItem()
        __qtablewidgetitem74.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(4, 1, __qtablewidgetitem74)
        __qtablewidgetitem75 = QTableWidgetItem()
        __qtablewidgetitem75.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(4, 2, __qtablewidgetitem75)
        __qtablewidgetitem76 = QTableWidgetItem()
        __qtablewidgetitem76.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_derivative_error.setItem(4, 3, __qtablewidgetitem76)
        self.table_def_in_derivative_error.setObjectName(u"table_def_in_derivative_error")
        self.table_def_in_derivative_error.setGeometry(QRect(480, 60, 441, 201))
        self.table_def_in_derivative_error.setAlternatingRowColors(True)
        self.table_def_in_derivative_error.horizontalHeader().setDefaultSectionSize(100)
        self.table_def_in_rotation_angle = QTableWidget(self.group_def_input)
        if (self.table_def_in_rotation_angle.columnCount() < 4):
            self.table_def_in_rotation_angle.setColumnCount(4)
        __qtablewidgetitem77 = QTableWidgetItem()
        self.table_def_in_rotation_angle.setHorizontalHeaderItem(0, __qtablewidgetitem77)
        __qtablewidgetitem78 = QTableWidgetItem()
        self.table_def_in_rotation_angle.setHorizontalHeaderItem(1, __qtablewidgetitem78)
        __qtablewidgetitem79 = QTableWidgetItem()
        self.table_def_in_rotation_angle.setHorizontalHeaderItem(2, __qtablewidgetitem79)
        __qtablewidgetitem80 = QTableWidgetItem()
        self.table_def_in_rotation_angle.setHorizontalHeaderItem(3, __qtablewidgetitem80)
        if (self.table_def_in_rotation_angle.rowCount() < 5):
            self.table_def_in_rotation_angle.setRowCount(5)
        __qtablewidgetitem81 = QTableWidgetItem()
        self.table_def_in_rotation_angle.setVerticalHeaderItem(0, __qtablewidgetitem81)
        __qtablewidgetitem82 = QTableWidgetItem()
        self.table_def_in_rotation_angle.setVerticalHeaderItem(1, __qtablewidgetitem82)
        __qtablewidgetitem83 = QTableWidgetItem()
        self.table_def_in_rotation_angle.setVerticalHeaderItem(2, __qtablewidgetitem83)
        __qtablewidgetitem84 = QTableWidgetItem()
        self.table_def_in_rotation_angle.setVerticalHeaderItem(3, __qtablewidgetitem84)
        __qtablewidgetitem85 = QTableWidgetItem()
        self.table_def_in_rotation_angle.setVerticalHeaderItem(4, __qtablewidgetitem85)
        __qtablewidgetitem86 = QTableWidgetItem()
        __qtablewidgetitem86.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(0, 0, __qtablewidgetitem86)
        __qtablewidgetitem87 = QTableWidgetItem()
        __qtablewidgetitem87.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(0, 1, __qtablewidgetitem87)
        __qtablewidgetitem88 = QTableWidgetItem()
        __qtablewidgetitem88.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(0, 2, __qtablewidgetitem88)
        __qtablewidgetitem89 = QTableWidgetItem()
        __qtablewidgetitem89.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(0, 3, __qtablewidgetitem89)
        __qtablewidgetitem90 = QTableWidgetItem()
        __qtablewidgetitem90.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(1, 0, __qtablewidgetitem90)
        __qtablewidgetitem91 = QTableWidgetItem()
        __qtablewidgetitem91.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(1, 1, __qtablewidgetitem91)
        __qtablewidgetitem92 = QTableWidgetItem()
        __qtablewidgetitem92.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(1, 2, __qtablewidgetitem92)
        __qtablewidgetitem93 = QTableWidgetItem()
        __qtablewidgetitem93.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(1, 3, __qtablewidgetitem93)
        __qtablewidgetitem94 = QTableWidgetItem()
        __qtablewidgetitem94.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(2, 0, __qtablewidgetitem94)
        __qtablewidgetitem95 = QTableWidgetItem()
        __qtablewidgetitem95.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(2, 1, __qtablewidgetitem95)
        __qtablewidgetitem96 = QTableWidgetItem()
        __qtablewidgetitem96.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(2, 2, __qtablewidgetitem96)
        __qtablewidgetitem97 = QTableWidgetItem()
        __qtablewidgetitem97.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(2, 3, __qtablewidgetitem97)
        __qtablewidgetitem98 = QTableWidgetItem()
        __qtablewidgetitem98.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(3, 0, __qtablewidgetitem98)
        __qtablewidgetitem99 = QTableWidgetItem()
        __qtablewidgetitem99.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(3, 1, __qtablewidgetitem99)
        __qtablewidgetitem100 = QTableWidgetItem()
        __qtablewidgetitem100.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(3, 2, __qtablewidgetitem100)
        __qtablewidgetitem101 = QTableWidgetItem()
        __qtablewidgetitem101.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(3, 3, __qtablewidgetitem101)
        __qtablewidgetitem102 = QTableWidgetItem()
        __qtablewidgetitem102.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(4, 0, __qtablewidgetitem102)
        __qtablewidgetitem103 = QTableWidgetItem()
        __qtablewidgetitem103.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(4, 1, __qtablewidgetitem103)
        __qtablewidgetitem104 = QTableWidgetItem()
        __qtablewidgetitem104.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(4, 2, __qtablewidgetitem104)
        __qtablewidgetitem105 = QTableWidgetItem()
        __qtablewidgetitem105.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_rotation_angle.setItem(4, 3, __qtablewidgetitem105)
        self.table_def_in_rotation_angle.setObjectName(u"table_def_in_rotation_angle")
        self.table_def_in_rotation_angle.setGeometry(QRect(940, 60, 441, 201))
        self.table_def_in_rotation_angle.setAlternatingRowColors(True)
        self.table_def_in_rotation_angle.horizontalHeader().setDefaultSectionSize(100)
        self.label_in_phi = QLabel(self.group_def_input)
        self.label_in_phi.setObjectName(u"label_in_phi")
        self.label_in_phi.setGeometry(QRect(20, 265, 131, 21))
        self.table_def_in_phi = QTableWidget(self.group_def_input)
        if (self.table_def_in_phi.columnCount() < 4):
            self.table_def_in_phi.setColumnCount(4)
        __qtablewidgetitem106 = QTableWidgetItem()
        self.table_def_in_phi.setHorizontalHeaderItem(0, __qtablewidgetitem106)
        __qtablewidgetitem107 = QTableWidgetItem()
        self.table_def_in_phi.setHorizontalHeaderItem(1, __qtablewidgetitem107)
        __qtablewidgetitem108 = QTableWidgetItem()
        self.table_def_in_phi.setHorizontalHeaderItem(2, __qtablewidgetitem108)
        __qtablewidgetitem109 = QTableWidgetItem()
        self.table_def_in_phi.setHorizontalHeaderItem(3, __qtablewidgetitem109)
        if (self.table_def_in_phi.rowCount() < 3):
            self.table_def_in_phi.setRowCount(3)
        __qtablewidgetitem110 = QTableWidgetItem()
        self.table_def_in_phi.setVerticalHeaderItem(0, __qtablewidgetitem110)
        __qtablewidgetitem111 = QTableWidgetItem()
        self.table_def_in_phi.setVerticalHeaderItem(1, __qtablewidgetitem111)
        __qtablewidgetitem112 = QTableWidgetItem()
        self.table_def_in_phi.setVerticalHeaderItem(2, __qtablewidgetitem112)
        __qtablewidgetitem113 = QTableWidgetItem()
        __qtablewidgetitem113.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(0, 0, __qtablewidgetitem113)
        __qtablewidgetitem114 = QTableWidgetItem()
        __qtablewidgetitem114.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(0, 1, __qtablewidgetitem114)
        __qtablewidgetitem115 = QTableWidgetItem()
        __qtablewidgetitem115.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(0, 2, __qtablewidgetitem115)
        __qtablewidgetitem116 = QTableWidgetItem()
        __qtablewidgetitem116.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(0, 3, __qtablewidgetitem116)
        __qtablewidgetitem117 = QTableWidgetItem()
        __qtablewidgetitem117.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(1, 0, __qtablewidgetitem117)
        __qtablewidgetitem118 = QTableWidgetItem()
        __qtablewidgetitem118.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(1, 1, __qtablewidgetitem118)
        __qtablewidgetitem119 = QTableWidgetItem()
        __qtablewidgetitem119.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(1, 2, __qtablewidgetitem119)
        __qtablewidgetitem120 = QTableWidgetItem()
        __qtablewidgetitem120.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(1, 3, __qtablewidgetitem120)
        __qtablewidgetitem121 = QTableWidgetItem()
        __qtablewidgetitem121.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(2, 0, __qtablewidgetitem121)
        __qtablewidgetitem122 = QTableWidgetItem()
        __qtablewidgetitem122.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(2, 1, __qtablewidgetitem122)
        __qtablewidgetitem123 = QTableWidgetItem()
        __qtablewidgetitem123.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(2, 2, __qtablewidgetitem123)
        __qtablewidgetitem124 = QTableWidgetItem()
        __qtablewidgetitem124.setTextAlignment(Qt.AlignCenter);
        self.table_def_in_phi.setItem(2, 3, __qtablewidgetitem124)
        self.table_def_in_phi.setObjectName(u"table_def_in_phi")
        self.table_def_in_phi.setGeometry(QRect(20, 290, 441, 141))
        self.table_def_in_phi.setAlternatingRowColors(True)
        self.table_def_in_phi.horizontalHeader().setDefaultSectionSize(100)
        self.label_nor_coef = QLabel(self.group_def_input)
        self.label_nor_coef.setObjectName(u"label_nor_coef")
        self.label_nor_coef.setGeometry(QRect(940, 260, 221, 31))
        self.table_def_nor_coef = QTableWidget(self.group_def_input)
        if (self.table_def_nor_coef.columnCount() < 2):
            self.table_def_nor_coef.setColumnCount(2)
        __qtablewidgetitem125 = QTableWidgetItem()
        self.table_def_nor_coef.setHorizontalHeaderItem(0, __qtablewidgetitem125)
        __qtablewidgetitem126 = QTableWidgetItem()
        self.table_def_nor_coef.setHorizontalHeaderItem(1, __qtablewidgetitem126)
        if (self.table_def_nor_coef.rowCount() < 3):
            self.table_def_nor_coef.setRowCount(3)
        __qtablewidgetitem127 = QTableWidgetItem()
        self.table_def_nor_coef.setVerticalHeaderItem(0, __qtablewidgetitem127)
        __qtablewidgetitem128 = QTableWidgetItem()
        self.table_def_nor_coef.setVerticalHeaderItem(1, __qtablewidgetitem128)
        __qtablewidgetitem129 = QTableWidgetItem()
        self.table_def_nor_coef.setVerticalHeaderItem(2, __qtablewidgetitem129)
        __qtablewidgetitem130 = QTableWidgetItem()
        __qtablewidgetitem130.setTextAlignment(Qt.AlignCenter);
        self.table_def_nor_coef.setItem(0, 0, __qtablewidgetitem130)
        __qtablewidgetitem131 = QTableWidgetItem()
        __qtablewidgetitem131.setTextAlignment(Qt.AlignCenter);
        self.table_def_nor_coef.setItem(0, 1, __qtablewidgetitem131)
        __qtablewidgetitem132 = QTableWidgetItem()
        __qtablewidgetitem132.setTextAlignment(Qt.AlignCenter);
        self.table_def_nor_coef.setItem(1, 0, __qtablewidgetitem132)
        __qtablewidgetitem133 = QTableWidgetItem()
        __qtablewidgetitem133.setTextAlignment(Qt.AlignCenter);
        self.table_def_nor_coef.setItem(1, 1, __qtablewidgetitem133)
        __qtablewidgetitem134 = QTableWidgetItem()
        __qtablewidgetitem134.setTextAlignment(Qt.AlignCenter);
        self.table_def_nor_coef.setItem(2, 0, __qtablewidgetitem134)
        __qtablewidgetitem135 = QTableWidgetItem()
        __qtablewidgetitem135.setTextAlignment(Qt.AlignCenter);
        self.table_def_nor_coef.setItem(2, 1, __qtablewidgetitem135)
        self.table_def_nor_coef.setObjectName(u"table_def_nor_coef")
        self.table_def_nor_coef.setGeometry(QRect(940, 290, 531, 131))
        self.table_def_nor_coef.setAlternatingRowColors(True)
        self.table_def_nor_coef.horizontalHeader().setDefaultSectionSize(175)
        self.group_def_output = QGroupBox(self.tab_programmer)
        self.group_def_output.setObjectName(u"group_def_output")
        self.group_def_output.setGeometry(QRect(10, 490, 1501, 441))
        self.group_def_output.setFont(font)
        self.table_def_out_speed = QTableWidget(self.group_def_output)
        if (self.table_def_out_speed.columnCount() < 3):
            self.table_def_out_speed.setColumnCount(3)
        __qtablewidgetitem136 = QTableWidgetItem()
        self.table_def_out_speed.setHorizontalHeaderItem(0, __qtablewidgetitem136)
        __qtablewidgetitem137 = QTableWidgetItem()
        self.table_def_out_speed.setHorizontalHeaderItem(1, __qtablewidgetitem137)
        __qtablewidgetitem138 = QTableWidgetItem()
        self.table_def_out_speed.setHorizontalHeaderItem(2, __qtablewidgetitem138)
        if (self.table_def_out_speed.rowCount() < 1):
            self.table_def_out_speed.setRowCount(1)
        __qtablewidgetitem139 = QTableWidgetItem()
        self.table_def_out_speed.setVerticalHeaderItem(0, __qtablewidgetitem139)
        __qtablewidgetitem140 = QTableWidgetItem()
        __qtablewidgetitem140.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_speed.setItem(0, 0, __qtablewidgetitem140)
        __qtablewidgetitem141 = QTableWidgetItem()
        __qtablewidgetitem141.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_speed.setItem(0, 1, __qtablewidgetitem141)
        __qtablewidgetitem142 = QTableWidgetItem()
        __qtablewidgetitem142.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_speed.setItem(0, 2, __qtablewidgetitem142)
        self.table_def_out_speed.setObjectName(u"table_def_out_speed")
        self.table_def_out_speed.setGeometry(QRect(10, 80, 491, 71))
        self.table_def_out_speed.setAlternatingRowColors(True)
        self.table_def_out_speed.horizontalHeader().setDefaultSectionSize(125)
        self.table_def_out_rotation_angle = QTableWidget(self.group_def_output)
        if (self.table_def_out_rotation_angle.columnCount() < 7):
            self.table_def_out_rotation_angle.setColumnCount(7)
        __qtablewidgetitem143 = QTableWidgetItem()
        self.table_def_out_rotation_angle.setHorizontalHeaderItem(0, __qtablewidgetitem143)
        __qtablewidgetitem144 = QTableWidgetItem()
        self.table_def_out_rotation_angle.setHorizontalHeaderItem(1, __qtablewidgetitem144)
        __qtablewidgetitem145 = QTableWidgetItem()
        self.table_def_out_rotation_angle.setHorizontalHeaderItem(2, __qtablewidgetitem145)
        __qtablewidgetitem146 = QTableWidgetItem()
        self.table_def_out_rotation_angle.setHorizontalHeaderItem(3, __qtablewidgetitem146)
        __qtablewidgetitem147 = QTableWidgetItem()
        self.table_def_out_rotation_angle.setHorizontalHeaderItem(4, __qtablewidgetitem147)
        __qtablewidgetitem148 = QTableWidgetItem()
        self.table_def_out_rotation_angle.setHorizontalHeaderItem(5, __qtablewidgetitem148)
        __qtablewidgetitem149 = QTableWidgetItem()
        self.table_def_out_rotation_angle.setHorizontalHeaderItem(6, __qtablewidgetitem149)
        if (self.table_def_out_rotation_angle.rowCount() < 1):
            self.table_def_out_rotation_angle.setRowCount(1)
        __qtablewidgetitem150 = QTableWidgetItem()
        self.table_def_out_rotation_angle.setVerticalHeaderItem(0, __qtablewidgetitem150)
        __qtablewidgetitem151 = QTableWidgetItem()
        __qtablewidgetitem151.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle.setItem(0, 0, __qtablewidgetitem151)
        __qtablewidgetitem152 = QTableWidgetItem()
        __qtablewidgetitem152.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle.setItem(0, 1, __qtablewidgetitem152)
        __qtablewidgetitem153 = QTableWidgetItem()
        __qtablewidgetitem153.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle.setItem(0, 2, __qtablewidgetitem153)
        __qtablewidgetitem154 = QTableWidgetItem()
        __qtablewidgetitem154.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle.setItem(0, 3, __qtablewidgetitem154)
        __qtablewidgetitem155 = QTableWidgetItem()
        __qtablewidgetitem155.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle.setItem(0, 4, __qtablewidgetitem155)
        __qtablewidgetitem156 = QTableWidgetItem()
        __qtablewidgetitem156.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle.setItem(0, 5, __qtablewidgetitem156)
        __qtablewidgetitem157 = QTableWidgetItem()
        __qtablewidgetitem157.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle.setItem(0, 6, __qtablewidgetitem157)
        self.table_def_out_rotation_angle.setObjectName(u"table_def_out_rotation_angle")
        self.table_def_out_rotation_angle.setGeometry(QRect(530, 80, 841, 71))
        self.table_def_out_rotation_angle.setAlternatingRowColors(True)
        self.table_def_out_rotation_angle.horizontalHeader().setDefaultSectionSize(100)
        self.label_out_speed = QLabel(self.group_def_output)
        self.label_out_speed.setObjectName(u"label_out_speed")
        self.label_out_speed.setGeometry(QRect(10, 50, 221, 31))
        self.label_out_rotation_angle = QLabel(self.group_def_output)
        self.label_out_rotation_angle.setObjectName(u"label_out_rotation_angle")
        self.label_out_rotation_angle.setGeometry(QRect(530, 40, 301, 31))
        self.label_out_rotation_angle_3_inputs = QLabel(self.group_def_output)
        self.label_out_rotation_angle_3_inputs.setObjectName(u"label_out_rotation_angle_3_inputs")
        self.label_out_rotation_angle_3_inputs.setGeometry(QRect(10, 170, 301, 31))
        self.table_def_out_rotation_angle_3_inputs = QTableWidget(self.group_def_output)
        if (self.table_def_out_rotation_angle_3_inputs.columnCount() < 9):
            self.table_def_out_rotation_angle_3_inputs.setColumnCount(9)
        __qtablewidgetitem158 = QTableWidgetItem()
        self.table_def_out_rotation_angle_3_inputs.setHorizontalHeaderItem(0, __qtablewidgetitem158)
        __qtablewidgetitem159 = QTableWidgetItem()
        self.table_def_out_rotation_angle_3_inputs.setHorizontalHeaderItem(1, __qtablewidgetitem159)
        __qtablewidgetitem160 = QTableWidgetItem()
        self.table_def_out_rotation_angle_3_inputs.setHorizontalHeaderItem(2, __qtablewidgetitem160)
        __qtablewidgetitem161 = QTableWidgetItem()
        self.table_def_out_rotation_angle_3_inputs.setHorizontalHeaderItem(3, __qtablewidgetitem161)
        __qtablewidgetitem162 = QTableWidgetItem()
        self.table_def_out_rotation_angle_3_inputs.setHorizontalHeaderItem(4, __qtablewidgetitem162)
        __qtablewidgetitem163 = QTableWidgetItem()
        self.table_def_out_rotation_angle_3_inputs.setHorizontalHeaderItem(5, __qtablewidgetitem163)
        __qtablewidgetitem164 = QTableWidgetItem()
        self.table_def_out_rotation_angle_3_inputs.setHorizontalHeaderItem(6, __qtablewidgetitem164)
        __qtablewidgetitem165 = QTableWidgetItem()
        self.table_def_out_rotation_angle_3_inputs.setHorizontalHeaderItem(7, __qtablewidgetitem165)
        __qtablewidgetitem166 = QTableWidgetItem()
        self.table_def_out_rotation_angle_3_inputs.setHorizontalHeaderItem(8, __qtablewidgetitem166)
        if (self.table_def_out_rotation_angle_3_inputs.rowCount() < 1):
            self.table_def_out_rotation_angle_3_inputs.setRowCount(1)
        __qtablewidgetitem167 = QTableWidgetItem()
        self.table_def_out_rotation_angle_3_inputs.setVerticalHeaderItem(0, __qtablewidgetitem167)
        __qtablewidgetitem168 = QTableWidgetItem()
        __qtablewidgetitem168.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle_3_inputs.setItem(0, 0, __qtablewidgetitem168)
        __qtablewidgetitem169 = QTableWidgetItem()
        __qtablewidgetitem169.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle_3_inputs.setItem(0, 1, __qtablewidgetitem169)
        __qtablewidgetitem170 = QTableWidgetItem()
        __qtablewidgetitem170.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle_3_inputs.setItem(0, 2, __qtablewidgetitem170)
        __qtablewidgetitem171 = QTableWidgetItem()
        __qtablewidgetitem171.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle_3_inputs.setItem(0, 3, __qtablewidgetitem171)
        __qtablewidgetitem172 = QTableWidgetItem()
        __qtablewidgetitem172.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle_3_inputs.setItem(0, 4, __qtablewidgetitem172)
        __qtablewidgetitem173 = QTableWidgetItem()
        __qtablewidgetitem173.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle_3_inputs.setItem(0, 5, __qtablewidgetitem173)
        __qtablewidgetitem174 = QTableWidgetItem()
        __qtablewidgetitem174.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle_3_inputs.setItem(0, 6, __qtablewidgetitem174)
        __qtablewidgetitem175 = QTableWidgetItem()
        __qtablewidgetitem175.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle_3_inputs.setItem(0, 7, __qtablewidgetitem175)
        __qtablewidgetitem176 = QTableWidgetItem()
        __qtablewidgetitem176.setTextAlignment(Qt.AlignCenter);
        self.table_def_out_rotation_angle_3_inputs.setItem(0, 8, __qtablewidgetitem176)
        self.table_def_out_rotation_angle_3_inputs.setObjectName(u"table_def_out_rotation_angle_3_inputs")
        self.table_def_out_rotation_angle_3_inputs.setGeometry(QRect(10, 210, 1241, 71))
        self.table_def_out_rotation_angle_3_inputs.setAlternatingRowColors(True)
        self.table_def_out_rotation_angle_3_inputs.horizontalHeader().setDefaultSectionSize(100)
        self.group_program_msg = QGroupBox(self.tab_programmer)
        self.group_program_msg.setObjectName(u"group_program_msg")
        self.group_program_msg.setGeometry(QRect(1530, 10, 351, 921))
        self.group_program_msg.setFont(font)
        self.outmsg_program_msg = QTextBrowser(self.group_program_msg)
        self.outmsg_program_msg.setObjectName(u"outmsg_program_msg")
        self.outmsg_program_msg.setGeometry(QRect(20, 60, 311, 641))
        self.label_program_msg = QLabel(self.group_program_msg)
        self.label_program_msg.setObjectName(u"label_program_msg")
        self.label_program_msg.setGeometry(QRect(20, 30, 231, 31))
        self.btn_get_fuzzy_coef = QPushButton(self.group_program_msg)
        self.btn_get_fuzzy_coef.setObjectName(u"btn_get_fuzzy_coef")
        self.btn_get_fuzzy_coef.setGeometry(QRect(20, 720, 141, 71))
        self.btn_set_fuzzy_coef = QPushButton(self.group_program_msg)
        self.btn_set_fuzzy_coef.setObjectName(u"btn_set_fuzzy_coef")
        self.btn_set_fuzzy_coef.setGeometry(QRect(190, 720, 141, 71))
        self.btn_sync_time = QPushButton(self.group_program_msg)
        self.btn_sync_time.setObjectName(u"btn_sync_time")
        self.btn_sync_time.setGeometry(QRect(20, 800, 141, 71))
        self.btn_clear_program_msg = QPushButton(self.group_program_msg)
        self.btn_clear_program_msg.setObjectName(u"btn_clear_program_msg")
        self.btn_clear_program_msg.setGeometry(QRect(190, 800, 141, 71))
        self.tabWidget.addTab(self.tab_programmer, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1920, 21))
        self.menuVFR_GUI = QMenu(self.menubar)
        self.menuVFR_GUI.setObjectName(u"menuVFR_GUI")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.menubar.addAction(self.menuVFR_GUI.menuAction())

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.group_connectivity.setTitle(QCoreApplication.translate("MainWindow", u"Connectivity", None))
        self.btn_port_refresh.setText(QCoreApplication.translate("MainWindow", u"Refresh", None))
        self.outmsg_controller_connection_status.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Segoe UI'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" color:#ff5500;\">Disconnected</span></p></body></html>", None))
        self.outmsg_jetson_tx2_connection_status.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Segoe UI'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" color:#ff5500;\">Disconnected</span></p></body></html>", None))
        self.label_controller.setText(QCoreApplication.translate("MainWindow", u"Controller", None))
        self.label_jetson_tx2.setText(QCoreApplication.translate("MainWindow", u"Jestson TX2", None))
        self.btn_port_connect.setText(QCoreApplication.translate("MainWindow", u"Connect", None))
        self.label_port_list.setText(QCoreApplication.translate("MainWindow", u"Port list", None))
        self.btn_port_disconnect.setText(QCoreApplication.translate("MainWindow", u"Disconnect", None))
        self.group_robot_status.setTitle(QCoreApplication.translate("MainWindow", u"Robot status", None))
        self.inmsg_robot_max_speed.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Segoe UI'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">20</p></body></html>", None))
        self.outmsg_robot_current_speed.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Segoe UI'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">0</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", None))
        self.label_current_speed.setText(QCoreApplication.translate("MainWindow", u"Current speed", None))
        self.label_max_speed.setText(QCoreApplication.translate("MainWindow", u"Max speed", None))
        self.label_unit.setText(QCoreApplication.translate("MainWindow", u"cm/s", None))
        self.btn_robot_status_update.setText(QCoreApplication.translate("MainWindow", u"Update", None))
        self.outmsg_robot_process_state.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Segoe UI'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">IDLE</p></body></html>", None))
        self.label_robot_process_state.setText(QCoreApplication.translate("MainWindow", u"Process state", None))
        self.outmsg_robot_current_setpoint_speed.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Segoe UI'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">0</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", None))
        self.label_current_setpoint_speed.setText(QCoreApplication.translate("MainWindow", u"Current setpoint speed", None))
        self.label_unit_2.setText(QCoreApplication.translate("MainWindow", u"cm/s", None))
        self.outmsg_robot_curren_rotation_angle.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Segoe UI'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">0</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", None))
        self.label_robot_curren_rotation_angle.setText(QCoreApplication.translate("MainWindow", u"Current rotation angle", None))
        self.label_unit_3.setText(QCoreApplication.translate("MainWindow", u"cm/s", None))
        self.label_unit_4.setText(QCoreApplication.translate("MainWindow", u"degree", None))
        self.label_robot_max_rotation_angle.setText(QCoreApplication.translate("MainWindow", u"Max rotation angle", None))
        self.label_unit_5.setText(QCoreApplication.translate("MainWindow", u"degree", None))
        self.inmsg_max_rotation_angle.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Segoe UI'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">45</p></body></html>", None))
        self.label_robot_max_rotation_angle_2.setText(QCoreApplication.translate("MainWindow", u"Left: Angle  > 0 ; Right: Angle < 0 ; Max = 45 degree", None))
        self.outmsg_robot_direct_state.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Segoe UI'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">STOP</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", None))
        self.label_robot_direct_state.setText(QCoreApplication.translate("MainWindow", u"Direct state", None))
        self.label_robot_camera_state.setText(QCoreApplication.translate("MainWindow", u"Camera state", None))
        self.outmsg_robot_camera_state.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Segoe UI'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">ZERO</p></body></html>", None))
        self.group_msg.setTitle(QCoreApplication.translate("MainWindow", u"Robot Response", None))
        self.label_msg_notif.setText(QCoreApplication.translate("MainWindow", u"Notification message", None))
        self.label_action_list.setText(QCoreApplication.translate("MainWindow", u"Action list", None))
        self.btn_clear_robot_msg.setText(QCoreApplication.translate("MainWindow", u"Clear", None))
        self.label_signs_06.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label_signs_02.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label_signs_01.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label_signs_03.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label_signs_05.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label_signs_07.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label_signs_04.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label_signs_08.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label_msg.setText(QCoreApplication.translate("MainWindow", u"Information message", None))
        self.group_robot_control.setTitle(QCoreApplication.translate("MainWindow", u"Robot control", None))
        self.buttuon_turn_left.setText(QCoreApplication.translate("MainWindow", u"<---", None))
        self.buttuon_forward.setText(QCoreApplication.translate("MainWindow", u"^", None))
        self.button_turn_right.setText(QCoreApplication.translate("MainWindow", u"--->", None))
        self.button_start.setText(QCoreApplication.translate("MainWindow", u"Start", None))
        self.button_stop.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_user), QCoreApplication.translate("MainWindow", u"User Tab", None))
        self.groupBox.setTitle(QCoreApplication.translate("MainWindow", u"GroupBox", None))
        self.button_stop_nd.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.button_start_nd.setText(QCoreApplication.translate("MainWindow", u"Start", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_graph), QCoreApplication.translate("MainWindow", u"Graph tab", None))
        self.group_def_input.setTitle(QCoreApplication.translate("MainWindow", u"Definition of linguistic input variables and normalization coefficient", None))
        ___qtablewidgetitem = self.table_def_in_error.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("MainWindow", u"LEFT", None));
        ___qtablewidgetitem1 = self.table_def_in_error.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("MainWindow", u"CEN LEFT", None));
        ___qtablewidgetitem2 = self.table_def_in_error.horizontalHeaderItem(2)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("MainWindow", u"CEN RIGHT", None));
        ___qtablewidgetitem3 = self.table_def_in_error.horizontalHeaderItem(3)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("MainWindow", u"RIGHT", None));
        ___qtablewidgetitem4 = self.table_def_in_error.verticalHeaderItem(0)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("MainWindow", u"NB", None));
        ___qtablewidgetitem5 = self.table_def_in_error.verticalHeaderItem(1)
        ___qtablewidgetitem5.setText(QCoreApplication.translate("MainWindow", u"NS", None));
        ___qtablewidgetitem6 = self.table_def_in_error.verticalHeaderItem(2)
        ___qtablewidgetitem6.setText(QCoreApplication.translate("MainWindow", u"ZE", None));
        ___qtablewidgetitem7 = self.table_def_in_error.verticalHeaderItem(3)
        ___qtablewidgetitem7.setText(QCoreApplication.translate("MainWindow", u"PS", None));
        ___qtablewidgetitem8 = self.table_def_in_error.verticalHeaderItem(4)
        ___qtablewidgetitem8.setText(QCoreApplication.translate("MainWindow", u"PB", None));

        __sortingEnabled = self.table_def_in_error.isSortingEnabled()
        self.table_def_in_error.setSortingEnabled(False)
        ___qtablewidgetitem9 = self.table_def_in_error.item(0, 0)
        ___qtablewidgetitem9.setText(QCoreApplication.translate("MainWindow", u"-1", None));
        ___qtablewidgetitem10 = self.table_def_in_error.item(0, 1)
        ___qtablewidgetitem10.setText(QCoreApplication.translate("MainWindow", u"-1", None));
        ___qtablewidgetitem11 = self.table_def_in_error.item(0, 2)
        ___qtablewidgetitem11.setText(QCoreApplication.translate("MainWindow", u"-0.8", None));
        ___qtablewidgetitem12 = self.table_def_in_error.item(0, 3)
        ___qtablewidgetitem12.setText(QCoreApplication.translate("MainWindow", u"-0.3", None));
        ___qtablewidgetitem13 = self.table_def_in_error.item(1, 0)
        ___qtablewidgetitem13.setText(QCoreApplication.translate("MainWindow", u"-0.8", None));
        ___qtablewidgetitem14 = self.table_def_in_error.item(1, 1)
        ___qtablewidgetitem14.setText(QCoreApplication.translate("MainWindow", u"-0.3", None));
        ___qtablewidgetitem15 = self.table_def_in_error.item(1, 2)
        ___qtablewidgetitem15.setText(QCoreApplication.translate("MainWindow", u"-0.3", None));
        ___qtablewidgetitem16 = self.table_def_in_error.item(1, 3)
        ___qtablewidgetitem16.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem17 = self.table_def_in_error.item(2, 0)
        ___qtablewidgetitem17.setText(QCoreApplication.translate("MainWindow", u"-0.3", None));
        ___qtablewidgetitem18 = self.table_def_in_error.item(2, 1)
        ___qtablewidgetitem18.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem19 = self.table_def_in_error.item(2, 2)
        ___qtablewidgetitem19.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem20 = self.table_def_in_error.item(2, 3)
        ___qtablewidgetitem20.setText(QCoreApplication.translate("MainWindow", u"0.3", None));
        ___qtablewidgetitem21 = self.table_def_in_error.item(3, 0)
        ___qtablewidgetitem21.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem22 = self.table_def_in_error.item(3, 1)
        ___qtablewidgetitem22.setText(QCoreApplication.translate("MainWindow", u"0.3", None));
        ___qtablewidgetitem23 = self.table_def_in_error.item(3, 2)
        ___qtablewidgetitem23.setText(QCoreApplication.translate("MainWindow", u"0.3", None));
        ___qtablewidgetitem24 = self.table_def_in_error.item(3, 3)
        ___qtablewidgetitem24.setText(QCoreApplication.translate("MainWindow", u"0.8", None));
        ___qtablewidgetitem25 = self.table_def_in_error.item(4, 0)
        ___qtablewidgetitem25.setText(QCoreApplication.translate("MainWindow", u"0.3", None));
        ___qtablewidgetitem26 = self.table_def_in_error.item(4, 1)
        ___qtablewidgetitem26.setText(QCoreApplication.translate("MainWindow", u"0.8", None));
        ___qtablewidgetitem27 = self.table_def_in_error.item(4, 2)
        ___qtablewidgetitem27.setText(QCoreApplication.translate("MainWindow", u"1", None));
        ___qtablewidgetitem28 = self.table_def_in_error.item(4, 3)
        ___qtablewidgetitem28.setText(QCoreApplication.translate("MainWindow", u"1", None));
        self.table_def_in_error.setSortingEnabled(__sortingEnabled)

        ___qtablewidgetitem29 = self.table_def_in_speed.horizontalHeaderItem(0)
        ___qtablewidgetitem29.setText(QCoreApplication.translate("MainWindow", u"LEFT", None));
        ___qtablewidgetitem30 = self.table_def_in_speed.horizontalHeaderItem(1)
        ___qtablewidgetitem30.setText(QCoreApplication.translate("MainWindow", u"CEN LEFT", None));
        ___qtablewidgetitem31 = self.table_def_in_speed.horizontalHeaderItem(2)
        ___qtablewidgetitem31.setText(QCoreApplication.translate("MainWindow", u"CEN RIGHT", None));
        ___qtablewidgetitem32 = self.table_def_in_speed.horizontalHeaderItem(3)
        ___qtablewidgetitem32.setText(QCoreApplication.translate("MainWindow", u"RIGHT", None));
        ___qtablewidgetitem33 = self.table_def_in_speed.verticalHeaderItem(0)
        ___qtablewidgetitem33.setText(QCoreApplication.translate("MainWindow", u"PS", None));
        ___qtablewidgetitem34 = self.table_def_in_speed.verticalHeaderItem(1)
        ___qtablewidgetitem34.setText(QCoreApplication.translate("MainWindow", u"PM", None));
        ___qtablewidgetitem35 = self.table_def_in_speed.verticalHeaderItem(2)
        ___qtablewidgetitem35.setText(QCoreApplication.translate("MainWindow", u"PB", None));

        __sortingEnabled1 = self.table_def_in_speed.isSortingEnabled()
        self.table_def_in_speed.setSortingEnabled(False)
        ___qtablewidgetitem36 = self.table_def_in_speed.item(0, 0)
        ___qtablewidgetitem36.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem37 = self.table_def_in_speed.item(0, 1)
        ___qtablewidgetitem37.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem38 = self.table_def_in_speed.item(0, 2)
        ___qtablewidgetitem38.setText(QCoreApplication.translate("MainWindow", u"0.25", None));
        ___qtablewidgetitem39 = self.table_def_in_speed.item(0, 3)
        ___qtablewidgetitem39.setText(QCoreApplication.translate("MainWindow", u"0.5", None));
        ___qtablewidgetitem40 = self.table_def_in_speed.item(1, 0)
        ___qtablewidgetitem40.setText(QCoreApplication.translate("MainWindow", u"0.25", None));
        ___qtablewidgetitem41 = self.table_def_in_speed.item(1, 1)
        ___qtablewidgetitem41.setText(QCoreApplication.translate("MainWindow", u"0.5", None));
        ___qtablewidgetitem42 = self.table_def_in_speed.item(1, 2)
        ___qtablewidgetitem42.setText(QCoreApplication.translate("MainWindow", u"0.5", None));
        ___qtablewidgetitem43 = self.table_def_in_speed.item(1, 3)
        ___qtablewidgetitem43.setText(QCoreApplication.translate("MainWindow", u"0.75", None));
        ___qtablewidgetitem44 = self.table_def_in_speed.item(2, 0)
        ___qtablewidgetitem44.setText(QCoreApplication.translate("MainWindow", u"0.5", None));
        ___qtablewidgetitem45 = self.table_def_in_speed.item(2, 1)
        ___qtablewidgetitem45.setText(QCoreApplication.translate("MainWindow", u"0.75", None));
        ___qtablewidgetitem46 = self.table_def_in_speed.item(2, 2)
        ___qtablewidgetitem46.setText(QCoreApplication.translate("MainWindow", u"1", None));
        ___qtablewidgetitem47 = self.table_def_in_speed.item(2, 3)
        ___qtablewidgetitem47.setText(QCoreApplication.translate("MainWindow", u"1", None));
        self.table_def_in_speed.setSortingEnabled(__sortingEnabled1)

        self.label_in_error.setText(QCoreApplication.translate("MainWindow", u"Error", None))
        self.label_in_derivative_error.setText(QCoreApplication.translate("MainWindow", u"Derivative of error", None))
        self.label_in_rotation_angle.setText(QCoreApplication.translate("MainWindow", u"Normalized Rotation Angle", None))
        self.label_in_speed.setText(QCoreApplication.translate("MainWindow", u"Normailized Robot Speed", None))
        ___qtablewidgetitem48 = self.table_def_in_derivative_error.horizontalHeaderItem(0)
        ___qtablewidgetitem48.setText(QCoreApplication.translate("MainWindow", u"LEFT", None));
        ___qtablewidgetitem49 = self.table_def_in_derivative_error.horizontalHeaderItem(1)
        ___qtablewidgetitem49.setText(QCoreApplication.translate("MainWindow", u"CEN LEFT", None));
        ___qtablewidgetitem50 = self.table_def_in_derivative_error.horizontalHeaderItem(2)
        ___qtablewidgetitem50.setText(QCoreApplication.translate("MainWindow", u"CEN RIGHT", None));
        ___qtablewidgetitem51 = self.table_def_in_derivative_error.horizontalHeaderItem(3)
        ___qtablewidgetitem51.setText(QCoreApplication.translate("MainWindow", u"RIGHT", None));
        ___qtablewidgetitem52 = self.table_def_in_derivative_error.verticalHeaderItem(0)
        ___qtablewidgetitem52.setText(QCoreApplication.translate("MainWindow", u"NB", None));
        ___qtablewidgetitem53 = self.table_def_in_derivative_error.verticalHeaderItem(1)
        ___qtablewidgetitem53.setText(QCoreApplication.translate("MainWindow", u"NS", None));
        ___qtablewidgetitem54 = self.table_def_in_derivative_error.verticalHeaderItem(2)
        ___qtablewidgetitem54.setText(QCoreApplication.translate("MainWindow", u"ZE", None));
        ___qtablewidgetitem55 = self.table_def_in_derivative_error.verticalHeaderItem(3)
        ___qtablewidgetitem55.setText(QCoreApplication.translate("MainWindow", u"PS", None));
        ___qtablewidgetitem56 = self.table_def_in_derivative_error.verticalHeaderItem(4)
        ___qtablewidgetitem56.setText(QCoreApplication.translate("MainWindow", u"PB", None));

        __sortingEnabled2 = self.table_def_in_derivative_error.isSortingEnabled()
        self.table_def_in_derivative_error.setSortingEnabled(False)
        ___qtablewidgetitem57 = self.table_def_in_derivative_error.item(0, 0)
        ___qtablewidgetitem57.setText(QCoreApplication.translate("MainWindow", u"-1", None));
        ___qtablewidgetitem58 = self.table_def_in_derivative_error.item(0, 1)
        ___qtablewidgetitem58.setText(QCoreApplication.translate("MainWindow", u"-1", None));
        ___qtablewidgetitem59 = self.table_def_in_derivative_error.item(0, 2)
        ___qtablewidgetitem59.setText(QCoreApplication.translate("MainWindow", u"-0.75", None));
        ___qtablewidgetitem60 = self.table_def_in_derivative_error.item(0, 3)
        ___qtablewidgetitem60.setText(QCoreApplication.translate("MainWindow", u"-0.3", None));
        ___qtablewidgetitem61 = self.table_def_in_derivative_error.item(1, 0)
        ___qtablewidgetitem61.setText(QCoreApplication.translate("MainWindow", u"-0.75", None));
        ___qtablewidgetitem62 = self.table_def_in_derivative_error.item(1, 1)
        ___qtablewidgetitem62.setText(QCoreApplication.translate("MainWindow", u"-0.3", None));
        ___qtablewidgetitem63 = self.table_def_in_derivative_error.item(1, 2)
        ___qtablewidgetitem63.setText(QCoreApplication.translate("MainWindow", u"-0.3", None));
        ___qtablewidgetitem64 = self.table_def_in_derivative_error.item(1, 3)
        ___qtablewidgetitem64.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem65 = self.table_def_in_derivative_error.item(2, 0)
        ___qtablewidgetitem65.setText(QCoreApplication.translate("MainWindow", u"-0.3", None));
        ___qtablewidgetitem66 = self.table_def_in_derivative_error.item(2, 1)
        ___qtablewidgetitem66.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem67 = self.table_def_in_derivative_error.item(2, 2)
        ___qtablewidgetitem67.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem68 = self.table_def_in_derivative_error.item(2, 3)
        ___qtablewidgetitem68.setText(QCoreApplication.translate("MainWindow", u"0.3", None));
        ___qtablewidgetitem69 = self.table_def_in_derivative_error.item(3, 0)
        ___qtablewidgetitem69.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem70 = self.table_def_in_derivative_error.item(3, 1)
        ___qtablewidgetitem70.setText(QCoreApplication.translate("MainWindow", u"0.3", None));
        ___qtablewidgetitem71 = self.table_def_in_derivative_error.item(3, 2)
        ___qtablewidgetitem71.setText(QCoreApplication.translate("MainWindow", u"0.3", None));
        ___qtablewidgetitem72 = self.table_def_in_derivative_error.item(3, 3)
        ___qtablewidgetitem72.setText(QCoreApplication.translate("MainWindow", u"0.75", None));
        ___qtablewidgetitem73 = self.table_def_in_derivative_error.item(4, 0)
        ___qtablewidgetitem73.setText(QCoreApplication.translate("MainWindow", u"0.3", None));
        ___qtablewidgetitem74 = self.table_def_in_derivative_error.item(4, 1)
        ___qtablewidgetitem74.setText(QCoreApplication.translate("MainWindow", u"0.75", None));
        ___qtablewidgetitem75 = self.table_def_in_derivative_error.item(4, 2)
        ___qtablewidgetitem75.setText(QCoreApplication.translate("MainWindow", u"1", None));
        ___qtablewidgetitem76 = self.table_def_in_derivative_error.item(4, 3)
        ___qtablewidgetitem76.setText(QCoreApplication.translate("MainWindow", u"1", None));
        self.table_def_in_derivative_error.setSortingEnabled(__sortingEnabled2)

        ___qtablewidgetitem77 = self.table_def_in_rotation_angle.horizontalHeaderItem(0)
        ___qtablewidgetitem77.setText(QCoreApplication.translate("MainWindow", u"LEFT", None));
        ___qtablewidgetitem78 = self.table_def_in_rotation_angle.horizontalHeaderItem(1)
        ___qtablewidgetitem78.setText(QCoreApplication.translate("MainWindow", u"CEN LEFT", None));
        ___qtablewidgetitem79 = self.table_def_in_rotation_angle.horizontalHeaderItem(2)
        ___qtablewidgetitem79.setText(QCoreApplication.translate("MainWindow", u"CEN RIGHT", None));
        ___qtablewidgetitem80 = self.table_def_in_rotation_angle.horizontalHeaderItem(3)
        ___qtablewidgetitem80.setText(QCoreApplication.translate("MainWindow", u"RIGHT", None));
        ___qtablewidgetitem81 = self.table_def_in_rotation_angle.verticalHeaderItem(0)
        ___qtablewidgetitem81.setText(QCoreApplication.translate("MainWindow", u"NB", None));
        ___qtablewidgetitem82 = self.table_def_in_rotation_angle.verticalHeaderItem(1)
        ___qtablewidgetitem82.setText(QCoreApplication.translate("MainWindow", u"NS", None));
        ___qtablewidgetitem83 = self.table_def_in_rotation_angle.verticalHeaderItem(2)
        ___qtablewidgetitem83.setText(QCoreApplication.translate("MainWindow", u"ZE", None));
        ___qtablewidgetitem84 = self.table_def_in_rotation_angle.verticalHeaderItem(3)
        ___qtablewidgetitem84.setText(QCoreApplication.translate("MainWindow", u"PS", None));
        ___qtablewidgetitem85 = self.table_def_in_rotation_angle.verticalHeaderItem(4)
        ___qtablewidgetitem85.setText(QCoreApplication.translate("MainWindow", u"PB", None));

        __sortingEnabled3 = self.table_def_in_rotation_angle.isSortingEnabled()
        self.table_def_in_rotation_angle.setSortingEnabled(False)
        ___qtablewidgetitem86 = self.table_def_in_rotation_angle.item(0, 0)
        ___qtablewidgetitem86.setText(QCoreApplication.translate("MainWindow", u"-1", None));
        ___qtablewidgetitem87 = self.table_def_in_rotation_angle.item(0, 1)
        ___qtablewidgetitem87.setText(QCoreApplication.translate("MainWindow", u"-1", None));
        ___qtablewidgetitem88 = self.table_def_in_rotation_angle.item(0, 2)
        ___qtablewidgetitem88.setText(QCoreApplication.translate("MainWindow", u"-0.8", None));
        ___qtablewidgetitem89 = self.table_def_in_rotation_angle.item(0, 3)
        ___qtablewidgetitem89.setText(QCoreApplication.translate("MainWindow", u"-0.4", None));
        ___qtablewidgetitem90 = self.table_def_in_rotation_angle.item(1, 0)
        ___qtablewidgetitem90.setText(QCoreApplication.translate("MainWindow", u"-0.8", None));
        ___qtablewidgetitem91 = self.table_def_in_rotation_angle.item(1, 1)
        ___qtablewidgetitem91.setText(QCoreApplication.translate("MainWindow", u"-0.4", None));
        ___qtablewidgetitem92 = self.table_def_in_rotation_angle.item(1, 2)
        ___qtablewidgetitem92.setText(QCoreApplication.translate("MainWindow", u"-0.4", None));
        ___qtablewidgetitem93 = self.table_def_in_rotation_angle.item(1, 3)
        ___qtablewidgetitem93.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem94 = self.table_def_in_rotation_angle.item(2, 0)
        ___qtablewidgetitem94.setText(QCoreApplication.translate("MainWindow", u"-0.4", None));
        ___qtablewidgetitem95 = self.table_def_in_rotation_angle.item(2, 1)
        ___qtablewidgetitem95.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem96 = self.table_def_in_rotation_angle.item(2, 2)
        ___qtablewidgetitem96.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem97 = self.table_def_in_rotation_angle.item(2, 3)
        ___qtablewidgetitem97.setText(QCoreApplication.translate("MainWindow", u"0.4", None));
        ___qtablewidgetitem98 = self.table_def_in_rotation_angle.item(3, 0)
        ___qtablewidgetitem98.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem99 = self.table_def_in_rotation_angle.item(3, 1)
        ___qtablewidgetitem99.setText(QCoreApplication.translate("MainWindow", u"0.4", None));
        ___qtablewidgetitem100 = self.table_def_in_rotation_angle.item(3, 2)
        ___qtablewidgetitem100.setText(QCoreApplication.translate("MainWindow", u"0.4", None));
        ___qtablewidgetitem101 = self.table_def_in_rotation_angle.item(3, 3)
        ___qtablewidgetitem101.setText(QCoreApplication.translate("MainWindow", u"0.8", None));
        ___qtablewidgetitem102 = self.table_def_in_rotation_angle.item(4, 0)
        ___qtablewidgetitem102.setText(QCoreApplication.translate("MainWindow", u"0.4", None));
        ___qtablewidgetitem103 = self.table_def_in_rotation_angle.item(4, 1)
        ___qtablewidgetitem103.setText(QCoreApplication.translate("MainWindow", u"0.8", None));
        ___qtablewidgetitem104 = self.table_def_in_rotation_angle.item(4, 2)
        ___qtablewidgetitem104.setText(QCoreApplication.translate("MainWindow", u"1", None));
        ___qtablewidgetitem105 = self.table_def_in_rotation_angle.item(4, 3)
        ___qtablewidgetitem105.setText(QCoreApplication.translate("MainWindow", u"1", None));
        self.table_def_in_rotation_angle.setSortingEnabled(__sortingEnabled3)

        self.label_in_phi.setText(QCoreApplication.translate("MainWindow", u"Phi Error", None))
        ___qtablewidgetitem106 = self.table_def_in_phi.horizontalHeaderItem(0)
        ___qtablewidgetitem106.setText(QCoreApplication.translate("MainWindow", u"LEFT", None));
        ___qtablewidgetitem107 = self.table_def_in_phi.horizontalHeaderItem(1)
        ___qtablewidgetitem107.setText(QCoreApplication.translate("MainWindow", u"CEN LEFT", None));
        ___qtablewidgetitem108 = self.table_def_in_phi.horizontalHeaderItem(2)
        ___qtablewidgetitem108.setText(QCoreApplication.translate("MainWindow", u"CEN RIGHT", None));
        ___qtablewidgetitem109 = self.table_def_in_phi.horizontalHeaderItem(3)
        ___qtablewidgetitem109.setText(QCoreApplication.translate("MainWindow", u"RIGHT", None));
        ___qtablewidgetitem110 = self.table_def_in_phi.verticalHeaderItem(0)
        ___qtablewidgetitem110.setText(QCoreApplication.translate("MainWindow", u"NE", None));
        ___qtablewidgetitem111 = self.table_def_in_phi.verticalHeaderItem(1)
        ___qtablewidgetitem111.setText(QCoreApplication.translate("MainWindow", u"ZE", None));
        ___qtablewidgetitem112 = self.table_def_in_phi.verticalHeaderItem(2)
        ___qtablewidgetitem112.setText(QCoreApplication.translate("MainWindow", u"PO", None));

        __sortingEnabled4 = self.table_def_in_phi.isSortingEnabled()
        self.table_def_in_phi.setSortingEnabled(False)
        ___qtablewidgetitem113 = self.table_def_in_phi.item(0, 0)
        ___qtablewidgetitem113.setText(QCoreApplication.translate("MainWindow", u"-1", None));
        ___qtablewidgetitem114 = self.table_def_in_phi.item(0, 1)
        ___qtablewidgetitem114.setText(QCoreApplication.translate("MainWindow", u"-1", None));
        ___qtablewidgetitem115 = self.table_def_in_phi.item(0, 2)
        ___qtablewidgetitem115.setText(QCoreApplication.translate("MainWindow", u"-0.75", None));
        ___qtablewidgetitem116 = self.table_def_in_phi.item(0, 3)
        ___qtablewidgetitem116.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem117 = self.table_def_in_phi.item(1, 0)
        ___qtablewidgetitem117.setText(QCoreApplication.translate("MainWindow", u"-0.75", None));
        ___qtablewidgetitem118 = self.table_def_in_phi.item(1, 1)
        ___qtablewidgetitem118.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem119 = self.table_def_in_phi.item(1, 2)
        ___qtablewidgetitem119.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem120 = self.table_def_in_phi.item(1, 3)
        ___qtablewidgetitem120.setText(QCoreApplication.translate("MainWindow", u"0.75", None));
        ___qtablewidgetitem121 = self.table_def_in_phi.item(2, 0)
        ___qtablewidgetitem121.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem122 = self.table_def_in_phi.item(2, 1)
        ___qtablewidgetitem122.setText(QCoreApplication.translate("MainWindow", u"0.75", None));
        ___qtablewidgetitem123 = self.table_def_in_phi.item(2, 2)
        ___qtablewidgetitem123.setText(QCoreApplication.translate("MainWindow", u"1", None));
        ___qtablewidgetitem124 = self.table_def_in_phi.item(2, 3)
        ___qtablewidgetitem124.setText(QCoreApplication.translate("MainWindow", u"1", None));
        self.table_def_in_phi.setSortingEnabled(__sortingEnabled4)

        self.label_nor_coef.setText(QCoreApplication.translate("MainWindow", u"Normalization coefficient", None))
        ___qtablewidgetitem125 = self.table_def_nor_coef.horizontalHeaderItem(0)
        ___qtablewidgetitem125.setText(QCoreApplication.translate("MainWindow", u"Error", None));
        ___qtablewidgetitem126 = self.table_def_nor_coef.horizontalHeaderItem(1)
        ___qtablewidgetitem126.setText(QCoreApplication.translate("MainWindow", u"Derivative of error", None));
        ___qtablewidgetitem127 = self.table_def_nor_coef.verticalHeaderItem(0)
        ___qtablewidgetitem127.setText(QCoreApplication.translate("MainWindow", u"Following lane state", None));
        ___qtablewidgetitem128 = self.table_def_nor_coef.verticalHeaderItem(1)
        ___qtablewidgetitem128.setText(QCoreApplication.translate("MainWindow", u"Turning state", None));
        ___qtablewidgetitem129 = self.table_def_nor_coef.verticalHeaderItem(2)
        ___qtablewidgetitem129.setText(QCoreApplication.translate("MainWindow", u"Phi", None));

        __sortingEnabled5 = self.table_def_nor_coef.isSortingEnabled()
        self.table_def_nor_coef.setSortingEnabled(False)
        ___qtablewidgetitem130 = self.table_def_nor_coef.item(0, 0)
        ___qtablewidgetitem130.setText(QCoreApplication.translate("MainWindow", u"150", None));
        ___qtablewidgetitem131 = self.table_def_nor_coef.item(0, 1)
        ___qtablewidgetitem131.setText(QCoreApplication.translate("MainWindow", u"600", None));
        ___qtablewidgetitem132 = self.table_def_nor_coef.item(1, 0)
        ___qtablewidgetitem132.setText(QCoreApplication.translate("MainWindow", u"160", None));
        ___qtablewidgetitem133 = self.table_def_nor_coef.item(1, 1)
        ___qtablewidgetitem133.setText(QCoreApplication.translate("MainWindow", u"300", None));
        ___qtablewidgetitem134 = self.table_def_nor_coef.item(2, 0)
        ___qtablewidgetitem134.setText(QCoreApplication.translate("MainWindow", u"45", None));
        ___qtablewidgetitem135 = self.table_def_nor_coef.item(2, 1)
        ___qtablewidgetitem135.setText(QCoreApplication.translate("MainWindow", u"x", None));
        self.table_def_nor_coef.setSortingEnabled(__sortingEnabled5)

        self.group_def_output.setTitle(QCoreApplication.translate("MainWindow", u"Definition of linguistic output variables", None))
        ___qtablewidgetitem136 = self.table_def_out_speed.horizontalHeaderItem(0)
        ___qtablewidgetitem136.setText(QCoreApplication.translate("MainWindow", u"PS", None));
        ___qtablewidgetitem137 = self.table_def_out_speed.horizontalHeaderItem(1)
        ___qtablewidgetitem137.setText(QCoreApplication.translate("MainWindow", u"PM", None));
        ___qtablewidgetitem138 = self.table_def_out_speed.horizontalHeaderItem(2)
        ___qtablewidgetitem138.setText(QCoreApplication.translate("MainWindow", u"PB", None));
        ___qtablewidgetitem139 = self.table_def_out_speed.verticalHeaderItem(0)
        ___qtablewidgetitem139.setText(QCoreApplication.translate("MainWindow", u"Robot Speed", None));

        __sortingEnabled6 = self.table_def_out_speed.isSortingEnabled()
        self.table_def_out_speed.setSortingEnabled(False)
        ___qtablewidgetitem140 = self.table_def_out_speed.item(0, 0)
        ___qtablewidgetitem140.setText(QCoreApplication.translate("MainWindow", u"0.35", None));
        ___qtablewidgetitem141 = self.table_def_out_speed.item(0, 1)
        ___qtablewidgetitem141.setText(QCoreApplication.translate("MainWindow", u"0.65", None));
        ___qtablewidgetitem142 = self.table_def_out_speed.item(0, 2)
        ___qtablewidgetitem142.setText(QCoreApplication.translate("MainWindow", u"1", None));
        self.table_def_out_speed.setSortingEnabled(__sortingEnabled6)

        ___qtablewidgetitem143 = self.table_def_out_rotation_angle.horizontalHeaderItem(0)
        ___qtablewidgetitem143.setText(QCoreApplication.translate("MainWindow", u"NB", None));
        ___qtablewidgetitem144 = self.table_def_out_rotation_angle.horizontalHeaderItem(1)
        ___qtablewidgetitem144.setText(QCoreApplication.translate("MainWindow", u"NM", None));
        ___qtablewidgetitem145 = self.table_def_out_rotation_angle.horizontalHeaderItem(2)
        ___qtablewidgetitem145.setText(QCoreApplication.translate("MainWindow", u"NS", None));
        ___qtablewidgetitem146 = self.table_def_out_rotation_angle.horizontalHeaderItem(3)
        ___qtablewidgetitem146.setText(QCoreApplication.translate("MainWindow", u"ZE", None));
        ___qtablewidgetitem147 = self.table_def_out_rotation_angle.horizontalHeaderItem(4)
        ___qtablewidgetitem147.setText(QCoreApplication.translate("MainWindow", u"PS", None));
        ___qtablewidgetitem148 = self.table_def_out_rotation_angle.horizontalHeaderItem(5)
        ___qtablewidgetitem148.setText(QCoreApplication.translate("MainWindow", u"PM", None));
        ___qtablewidgetitem149 = self.table_def_out_rotation_angle.horizontalHeaderItem(6)
        ___qtablewidgetitem149.setText(QCoreApplication.translate("MainWindow", u"PB", None));
        ___qtablewidgetitem150 = self.table_def_out_rotation_angle.verticalHeaderItem(0)
        ___qtablewidgetitem150.setText(QCoreApplication.translate("MainWindow", u"Rotation Angle", None));

        __sortingEnabled7 = self.table_def_out_rotation_angle.isSortingEnabled()
        self.table_def_out_rotation_angle.setSortingEnabled(False)
        ___qtablewidgetitem151 = self.table_def_out_rotation_angle.item(0, 0)
        ___qtablewidgetitem151.setText(QCoreApplication.translate("MainWindow", u"-1", None));
        ___qtablewidgetitem152 = self.table_def_out_rotation_angle.item(0, 1)
        ___qtablewidgetitem152.setText(QCoreApplication.translate("MainWindow", u"-0.8", None));
        ___qtablewidgetitem153 = self.table_def_out_rotation_angle.item(0, 2)
        ___qtablewidgetitem153.setText(QCoreApplication.translate("MainWindow", u"-0.6", None));
        ___qtablewidgetitem154 = self.table_def_out_rotation_angle.item(0, 3)
        ___qtablewidgetitem154.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem155 = self.table_def_out_rotation_angle.item(0, 4)
        ___qtablewidgetitem155.setText(QCoreApplication.translate("MainWindow", u"0.6", None));
        ___qtablewidgetitem156 = self.table_def_out_rotation_angle.item(0, 5)
        ___qtablewidgetitem156.setText(QCoreApplication.translate("MainWindow", u"0.8", None));
        ___qtablewidgetitem157 = self.table_def_out_rotation_angle.item(0, 6)
        ___qtablewidgetitem157.setText(QCoreApplication.translate("MainWindow", u"1", None));
        self.table_def_out_rotation_angle.setSortingEnabled(__sortingEnabled7)

        self.label_out_speed.setText(QCoreApplication.translate("MainWindow", u"Robot Speed", None))
        self.label_out_rotation_angle.setText(QCoreApplication.translate("MainWindow", u"Rotation Angle (2 inputs controller)", None))
        self.label_out_rotation_angle_3_inputs.setText(QCoreApplication.translate("MainWindow", u"Rotation Angle (4 inputs controller)", None))
        ___qtablewidgetitem158 = self.table_def_out_rotation_angle_3_inputs.horizontalHeaderItem(0)
        ___qtablewidgetitem158.setText(QCoreApplication.translate("MainWindow", u"NB", None));
        ___qtablewidgetitem159 = self.table_def_out_rotation_angle_3_inputs.horizontalHeaderItem(1)
        ___qtablewidgetitem159.setText(QCoreApplication.translate("MainWindow", u"NM", None));
        ___qtablewidgetitem160 = self.table_def_out_rotation_angle_3_inputs.horizontalHeaderItem(2)
        ___qtablewidgetitem160.setText(QCoreApplication.translate("MainWindow", u"NS", None));
        ___qtablewidgetitem161 = self.table_def_out_rotation_angle_3_inputs.horizontalHeaderItem(3)
        ___qtablewidgetitem161.setText(QCoreApplication.translate("MainWindow", u"NVS", None));
        ___qtablewidgetitem162 = self.table_def_out_rotation_angle_3_inputs.horizontalHeaderItem(4)
        ___qtablewidgetitem162.setText(QCoreApplication.translate("MainWindow", u"ZE", None));
        ___qtablewidgetitem163 = self.table_def_out_rotation_angle_3_inputs.horizontalHeaderItem(5)
        ___qtablewidgetitem163.setText(QCoreApplication.translate("MainWindow", u"PVS", None));
        ___qtablewidgetitem164 = self.table_def_out_rotation_angle_3_inputs.horizontalHeaderItem(6)
        ___qtablewidgetitem164.setText(QCoreApplication.translate("MainWindow", u"PS", None));
        ___qtablewidgetitem165 = self.table_def_out_rotation_angle_3_inputs.horizontalHeaderItem(7)
        ___qtablewidgetitem165.setText(QCoreApplication.translate("MainWindow", u"PM", None));
        ___qtablewidgetitem166 = self.table_def_out_rotation_angle_3_inputs.horizontalHeaderItem(8)
        ___qtablewidgetitem166.setText(QCoreApplication.translate("MainWindow", u"PB", None));
        ___qtablewidgetitem167 = self.table_def_out_rotation_angle_3_inputs.verticalHeaderItem(0)
        ___qtablewidgetitem167.setText(QCoreApplication.translate("MainWindow", u"Rotation Angle", None));

        __sortingEnabled8 = self.table_def_out_rotation_angle_3_inputs.isSortingEnabled()
        self.table_def_out_rotation_angle_3_inputs.setSortingEnabled(False)
        ___qtablewidgetitem168 = self.table_def_out_rotation_angle_3_inputs.item(0, 0)
        ___qtablewidgetitem168.setText(QCoreApplication.translate("MainWindow", u"-1", None));
        ___qtablewidgetitem169 = self.table_def_out_rotation_angle_3_inputs.item(0, 1)
        ___qtablewidgetitem169.setText(QCoreApplication.translate("MainWindow", u"-0.8", None));
        ___qtablewidgetitem170 = self.table_def_out_rotation_angle_3_inputs.item(0, 2)
        ___qtablewidgetitem170.setText(QCoreApplication.translate("MainWindow", u"-0.5", None));
        ___qtablewidgetitem171 = self.table_def_out_rotation_angle_3_inputs.item(0, 3)
        ___qtablewidgetitem171.setText(QCoreApplication.translate("MainWindow", u"-0.25", None));
        ___qtablewidgetitem172 = self.table_def_out_rotation_angle_3_inputs.item(0, 4)
        ___qtablewidgetitem172.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem173 = self.table_def_out_rotation_angle_3_inputs.item(0, 5)
        ___qtablewidgetitem173.setText(QCoreApplication.translate("MainWindow", u"0.25", None));
        ___qtablewidgetitem174 = self.table_def_out_rotation_angle_3_inputs.item(0, 6)
        ___qtablewidgetitem174.setText(QCoreApplication.translate("MainWindow", u"0.5", None));
        ___qtablewidgetitem175 = self.table_def_out_rotation_angle_3_inputs.item(0, 7)
        ___qtablewidgetitem175.setText(QCoreApplication.translate("MainWindow", u"0.8", None));
        ___qtablewidgetitem176 = self.table_def_out_rotation_angle_3_inputs.item(0, 8)
        ___qtablewidgetitem176.setText(QCoreApplication.translate("MainWindow", u"1", None));
        self.table_def_out_rotation_angle_3_inputs.setSortingEnabled(__sortingEnabled8)

        self.group_program_msg.setTitle(QCoreApplication.translate("MainWindow", u"GroupBox", None))
        self.label_program_msg.setText(QCoreApplication.translate("MainWindow", u"Message", None))
        self.btn_get_fuzzy_coef.setText(QCoreApplication.translate("MainWindow", u"Get", None))
        self.btn_set_fuzzy_coef.setText(QCoreApplication.translate("MainWindow", u"Set", None))
        self.btn_sync_time.setText(QCoreApplication.translate("MainWindow", u"Sync time", None))
        self.btn_clear_program_msg.setText(QCoreApplication.translate("MainWindow", u"Clear", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_programmer), QCoreApplication.translate("MainWindow", u"Programmer Tab", None))
        self.menuVFR_GUI.setTitle(QCoreApplication.translate("MainWindow", u"VFR GUI", None))
    # retranslateUi

