# This Python file uses the following encoding: utf-8
import sys
sys.path.append('../../Intergrated_protocol/python_output')
sys.path.append('../../Intergrated_protocol/nanopb/generator/proto')
from PySide6.QtWidgets import QApplication, QMainWindow, QStackedWidget, QComboBox, QVBoxLayout, QWidget, QTextBrowser, QTableWidget, QTableWidgetItem, QMessageBox
from PySide6.QtCore import QObject, Signal, QTimer, Qt
from ui_vfr_gui_mainwindow import Ui_MainWindow
from protocol_parser_qt import VFR
import protocol_pb2 as proto
import serial
import serial.tools.list_ports
import time
from datetime import datetime
import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PySide6.QtGui import QPixmap

# pyside6-uic vfr_gui_mainwindow.ui -o vfr_gui_mainwindow.py
class VFR_GUI_main(QMainWindow):
    NETWORK_CONNTECTION_COUNTER_TRACK  = 4

    def __init__(self, parent=None):
        super().__init__(parent)
        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        # Set up the QMainWindow from form.ui
        self.user_ui = QMainWindow()
        self.ui_user = Ui_MainWindow()
        self.ui_user.setupUi(self.user_ui)

        # Set the window title for the User UI window
        self.setWindowTitle("User UI")

        # Add the vfr_gui_mainwindow (vfr_gui_mainwindow.ui) to the stacked widget
        self.stacked_widget.addWidget(self.user_ui)

        # Set the initial widget to user_ui
        self.stacked_widget.setCurrentWidget(self.user_ui)

        # Set fixed display size
        # self.setFixedSize(1920, 1030)
        self.resize(1920, 1030)

        # Initialize global variable
        self.vfr = VFR(src = proto.DEVICE_ADDR_APP, debug = False, gui_process = True)
        self.jetson_connection_cnt              = 0
        self.controller_connection_cnt          = 0
        self.debug_connection_check_enter_last  = 0
        self.debug_connection_check_enter_dur   = 0

        # Connect the object to the methods
        self.ui_user.btn_port_refresh.clicked.connect(self.port_list_refresh)
        self.ui_user.btn_port_connect.clicked.connect(self.port_connect)
        self.ui_user.btn_port_disconnect.clicked.connect(self.port_disconnect)
        self.ui_user.btn_clear_robot_msg.clicked.connect(self.clear_robot_msg)
        self.ui_user.button_start.clicked.connect(self.robot_cmd_start)
        self.ui_user.button_stop.clicked.connect(self.robot_cmd_stop)
        self.ui_user.button_start_nd.clicked.connect(self.robot_cmd_start)
        self.ui_user.button_stop_nd.clicked.connect(self.robot_cmd_stop)
        self.ui_user.buttuon_forward.clicked.connect(self.robot_cmd_move_forward)
        self.ui_user.buttuon_turn_left.clicked.connect(self.robot_cmd_turn_left)
        self.ui_user.button_turn_right.clicked.connect(self.robot_cmd_turn_right)
        self.ui_user.btn_get_fuzzy_coef.clicked.connect(self.get_robot_fuzzy_coef)
        self.ui_user.btn_set_fuzzy_coef.clicked.connect(self.set_robot_fuzzy_coef)
        self.ui_user.btn_clear_program_msg.clicked.connect(self.clear_robot_program_msg)
        self.ui_user.btn_sync_time.clicked.connect(self.sync_robot_time)
        self.ui_user.btn_robot_status_update.clicked.connect(self.update_robot_status)

        self.ui_user.table_def_in_error.cellChanged.connect(self.validate_input)
        self.ui_user.table_def_in_derivative_error.cellChanged.connect(self.validate_input)
        self.ui_user.table_def_in_rotation_angle.cellChanged.connect(self.validate_input)
        self.ui_user.table_def_in_speed.cellChanged.connect(self.validate_input)
        self.ui_user.table_def_out_speed.cellChanged.connect(self.validate_input)
        self.ui_user.table_def_out_rotation_angle.cellChanged.connect(self.validate_input)
        self.ui_user.table_def_nor_coef.cellChanged.connect(self.validate_nor_coef_input)

        # Connect function to signal
        self.vfr.signal_packet_process.connect(self.packet_parse_process)

        # Initialize function process
        self.port_list_refresh()

        # Initialize timer
        self.connection_check_timer = QTimer(self)
        self.connection_check_timer.timeout.connect(self.sys_network_connection_status_check)
        self.connection_check_timer.start(200)


        # ------------------------------------------------------------------------------------
        # Initialize graph data
        self.graph_data_time_point_len = 500

        self.graph_data_time = []

        self.graph_data_error = []
        self.graph_data_phi = []
        self.graph_data_speed = []
        self.graph_data_angle = []

        self.current_error = 0.0
        self.current_phi = 0.0
        self.current_speed = 0.0
        self.current_angle = 0.0

        self.graph_first_start = False
        self.graph_current_time = 0

        # Danh sách lưu nhiều curve theo từng camera_state
        self.curves_error = []
        self.curves_phi = []
        self.curves_speed = []
        self.curves_angle = []

        # Curve hiện tại đang vẽ (tùy theo trạng thái camera)
        self.current_curve_error = None
        self.current_curve_phi = None
        self.current_curve_speed = None
        self.current_curve_angle = None

        # Initialize graph
        self.camera_state = proto.ROBOT_CAMERA_STATE_0

        self.plot_widget_error = pg.PlotWidget()
        self.plot_widget_error.setBackground('#2f2f2f')
        self.plot_widget_error.setTitle("Center Error vs Time", color='w')
        self.plot_widget_error.setLabel('left', 'Error', units='pixel', color='white')
        self.plot_widget_error.setLabel('bottom', 'Time', units='s', color='white')
        self.plot_widget_error.showGrid(x=True, y=True, alpha=0.8)

        self.plot_widget_phi = pg.PlotWidget()
        self.plot_widget_phi.setBackground('#2f2f2f')
        self.plot_widget_phi.setTitle("Phi Error vs Time", color='w')
        self.plot_widget_phi.setLabel('left', 'Phi', units='degree', color='white')
        self.plot_widget_phi.setLabel('bottom', 'Time', units='s', color='white')
        self.plot_widget_phi.showGrid(x=True, y=True, alpha=0.8)

        self.plot_widget_speed = pg.PlotWidget()
        self.plot_widget_speed.setBackground('#2f2f2f')
        self.plot_widget_speed.setTitle("Speed vs Time", color='w')
        self.plot_widget_speed.setLabel('left', 'Speed', units='cm/s', color='white')
        self.plot_widget_speed.setLabel('bottom', 'Time', units='s', color='white')
        self.plot_widget_speed.showGrid(x=True, y=True, alpha=0.8)

        self.plot_widget_angle = pg.PlotWidget()
        self.plot_widget_angle.setBackground('#2f2f2f')
        self.plot_widget_angle.setTitle("Rotation Angle vs Time", color='w')
        self.plot_widget_angle.setLabel('left', 'Rotation Angle', units='degree', color='white')
        self.plot_widget_angle.setLabel('bottom', 'Time', units='s', color='white')
        self.plot_widget_angle.showGrid(x=True, y=True, alpha=0.8)

        pen = pg.mkPen(color='aqua', width=2, style=pg.QtCore.Qt.SolidLine)
        self.plot_curve_error   = self.plot_widget_error.plot([], [], pen=pen)
        self.plot_curve_phi     = self.plot_widget_phi.plot([], [], pen=pen)
        self.plot_curve_speed   = self.plot_widget_speed.plot([], [], pen=pen)
        self.plot_curve_angle   = self.plot_widget_angle.plot([], [], pen=pen)

        # Set up graph
        layout_error = QVBoxLayout(self.ui_user.graph_error)
        layout_error.setContentsMargins(0, 0, 0, 0)
        layout_error.addWidget(self.plot_widget_error)

        layout_phi = QVBoxLayout(self.ui_user.graph_phi)
        layout_phi.setContentsMargins(0, 0, 0, 0)
        layout_phi.addWidget(self.plot_widget_phi)

        layout_speed = QVBoxLayout(self.ui_user.graph_speed)
        layout_speed.setContentsMargins(0, 0, 0, 0)
        layout_speed.addWidget(self.plot_widget_speed)

        layout_angle = QVBoxLayout(self.ui_user.graph_angle)
        layout_angle.setContentsMargins(0, 0, 0, 0)
        layout_angle.addWidget(self.plot_widget_angle)

        self.zero_line_error = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('red', width=3, style=pg.QtCore.Qt.SolidLine))
        self.zero_line_phi = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('red', width=3, style=pg.QtCore.Qt.SolidLine))
        self.zero_line_speed = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('red', width=3, style=pg.QtCore.Qt.SolidLine))
        self.zero_line_angle = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('red', width=3, style=pg.QtCore.Qt.SolidLine))
        
        self.plot_widget_error.addItem(self.zero_line_error)
        self.plot_widget_phi.addItem(self.zero_line_phi)
        self.plot_widget_speed.addItem(self.zero_line_speed)
        self.plot_widget_angle.addItem(self.zero_line_angle)

        self.init_static_legend()

        # ------------------------------------------------------------------------------------
        
        self.sign_labels = [
            self.ui_user.label_signs_01,
            self.ui_user.label_signs_02,
            self.ui_user.label_signs_03,
            self.ui_user.label_signs_04,
            self.ui_user.label_signs_05,
            self.ui_user.label_signs_06,
            self.ui_user.label_signs_07,
            self.ui_user.label_signs_08,
        ]

        self.sign_images = [
            'bus_stop.png',
            'children_crossing.png',
            'green_light.png',
            'left_turn_only.png',
            'no_stopping.png',
            'red_light.png',
            'speed_limit_40.png',
            'stop.png',
        ]

        for label in self.sign_labels:
            label.clear()

        # ------------------------------------------------------------------------------------
        self.current_max_speed_setpoint = 0
        self.is_update_current_max_speed_setpoint = False

    def init_static_legend(self):
        camera_state_names = {
            proto.ROBOT_CAMERA_STATE_P_45  : "State Left +45°",
            proto.ROBOT_CAMERA_STATE_P_30  : "State Left +30°",
            proto.ROBOT_CAMERA_STATE_0     : "State 0 (Default)",
            proto.ROBOT_CAMERA_STATE_N_30  : "State Right -30",
            proto.ROBOT_CAMERA_STATE_N_45  : "State Right -45°",
            # Không thêm MAX để giữ đúng 5 dòng như bạn yêu cầu
        }

        colors = {
            proto.ROBOT_CAMERA_STATE_P_45  : "yellow",
            proto.ROBOT_CAMERA_STATE_P_30  : "lime",
            proto.ROBOT_CAMERA_STATE_0     : "aqua",
            proto.ROBOT_CAMERA_STATE_N_30  : "magenta",
            proto.ROBOT_CAMERA_STATE_N_45  : "pink",
        }

        # Layout chính
        layout = QVBoxLayout()
        layout.setSpacing(8)
        layout.setContentsMargins(5, 5, 5, 5)

        for state, color in colors.items():
            row_layout = QHBoxLayout()

            # Đường thẳng màu
            color_line = QFrame()
            color_line.setFixedSize(40, 6)
            color_line.setStyleSheet(f"background-color: {color}; border-radius: 3px;")

            # Nhãn
            label = QLabel(camera_state_names[state])
            label.setStyleSheet("color: white; font-size: 12px;")
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

            row_layout.addWidget(color_line)
            row_layout.addWidget(label)
            row_layout.addStretch()

            layout.addLayout(row_layout)

        self.ui_user.graph_legend_static.setLayout(layout)


    def message_convert_warning(self, msg):
        warning_msg = f'<span style="color: orange;">{msg}</span>'
        return warning_msg


    def message_convert_error(self, msg):
        error_msg = f'<span style="color: red;">{msg}</span>'
        return error_msg


    def message_convert_info(self, msg):
        info_msg = f'<span style="color: aqua;">{msg}</span>'
        return info_msg


    def message_convert_nor(self, msg):
        nor_msg = f'<span style="color: white;">{msg}</span>'
        return nor_msg
    
    def message_format_convert(self, src, msg, epoch_time_ms):
        dt = datetime.fromtimestamp(epoch_time_ms / 1000)

        time_str = dt.strftime("%H:%M:%S.%f")[:-3]
        # time_str = dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        if (src == proto.DEVICE_ADDR_CONTROLLER):
            src_text = '[CONTROLLER]'
        else:
            src_text = '[TX2]'

        formatted_msg = f"[{time_str}]{src_text}: {msg}"
        return formatted_msg


    def port_list_refresh(self):
        self.ui_user.combobox_port_list.clear()
        ports = self.vfr.detect_com_port()
        
        if not len(ports):
            self.ui_user.outmsg_robot_msg_notif.append('<span style="color: yellow;">[Warning] No port detected.</span>')
        else:
            for port in ports:
                item_text = f"{port.device} - {port.description}"
                self.ui_user.combobox_port_list.addItem(item_text)


    def port_connect(self):
        port_name = self.ui_user.combobox_port_list.currentText()
        self.vfr.com_port = port_name.split(' - ')[0]
        try:
            self.vfr.connect_com_port()
            self.ui_user.outmsg_robot_msg_notif.append(f'<span style="color: Aqua;">[Notification] Connect to port {port_name} successfully.</span>')
        except Exception as e:
            self.ui_user.outmsg_robot_msg_notif.append(f'<span style="color: red;">[Error] Connect to port {port_name} failed: {e}.</span>')


    def port_disconnect(self):
        if self.vfr.connected:
            try:
                self.vfr.disconnect_com_port()
                self.ui_user.outmsg_robot_msg_notif.append(f'<span style="color: Aqua;">[Notification] Disconnect port successfully.</span>')
                self.jetson_connection_cnt = 0
                self.controller_connection_cnt = 0
                self.controller_status_update(False)
                self.jetson_status_update(False)
            except Exception as e:
                self.ui_user.outmsg_robot_msg_notif.append(f'<span style="color: red;">[Error] Disconnect port failed: {e}.</span>')
        else:
            self.ui_user.outmsg_robot_msg_notif.append(f'<span style="color: yellow;">[Warning] No port connected.</span>')


    def clear_robot_msg(self):
        self.ui_user.outmsg_robot_msg.clear()
        self.ui_user.outmsg_robot_msg_notif.clear()
        self.clear_signs_pics()


    def clear_robot_program_msg(self):
        self.ui_user.outmsg_program_msg.clear()


    def robot_msg_update(self, data):
        self.ui_user.outmsg_robot_msg.append(data)


    def controller_status_update(self, connect):
        if connect:
            self.ui_user.outmsg_controller_connection_status.clear()
            self.ui_user.outmsg_controller_connection_status.append(f'<span style="color: Aqua;">Connected</span>')
        else:
            self.ui_user.outmsg_controller_connection_status.clear()
            self.ui_user.outmsg_controller_connection_status.append(f'<span style="color: Red;">Disconnected</span>')


    def jetson_status_update(self, connect):
        if connect:
            self.ui_user.outmsg_jetson_tx2_connection_status.clear()
            self.ui_user.outmsg_jetson_tx2_connection_status.append(f'<span style="color: Aqua;">Connected</span>')
        else:
            self.ui_user.outmsg_jetson_tx2_connection_status.clear()
            self.ui_user.outmsg_jetson_tx2_connection_status.append(f'<span style="color: Red;">Disconnected</span>')

    def robot_cmd_start(self):
        # self.vfr.packet_robot_cmd_start()
        # self.vfr.packet_network_status_get(proto.DEVICE_ADDR_CONTROLLER)
        self.clear_chart()

        packet = proto.network_packet_t()
        packet.robot_cmd_start.dummy = 0
        self.vfr.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet)

        return
    

    def robot_cmd_stop(self):
        # self.vfr.packet_robot_cmd_start()
        # self.vfr.packet_network_status_get(proto.DEVICE_ADDR_CONTROLLER)
        packet = proto.network_packet_t()
        packet.robot_cmd_stop.dummy = 0
        self.vfr.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet)
        return
    

    def robot_cmd_move_forward(self):
        packet = proto.network_packet_t()
        packet.robot_direction_cmd.cmd = proto.DIRECTION_CMD_MOVE_FORWARD
        self.vfr.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet)

        return
    

    def robot_cmd_turn_left(self):
        packet = proto.network_packet_t()
        packet.robot_direction_cmd.cmd = proto.DIRECTION_CMD_MOVE_TURN_LEFT
        self.vfr.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet)

        return
    

    def robot_cmd_turn_right(self):
        packet = proto.network_packet_t()
        packet.robot_direction_cmd.cmd = proto.DIRECTION_CMD_MOVE_TURN_RIGHT
        self.vfr.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet)

        return

    def sys_network_connection_status_check(self):
        if self.vfr.connected:
            if(self.jetson_connection_cnt != 0):
                self.jetson_connection_cnt -= 1
                if (self.jetson_connection_cnt == 0):
                    self.jetson_status_update(False)

            if(self.controller_connection_cnt != 0):
                self.controller_connection_cnt -= 1
                if (self.controller_connection_cnt == 0):
                    self.controller_status_update(False)

            self.vfr.packet_network_status_get(proto.DEVICE_ADDR_CONTROLLER)
            self.vfr.packet_network_status_get(proto.DEVICE_ADDR_TX2)
        return


    def packet_parse_process(self, packet):
        packet_tag = packet.WhichOneof("params")
        if (self.packet_parse_def[packet_tag]["parser"] != None):
            self.packet_parse_def[packet_tag]["parser"](self, packet)
        return
    

    def packet_parse_netowrk_status_rsp(self, packet):
        if (packet.hdr.addr.src == self.vfr.DEVICE_ADDR_TX2):
            if self.jetson_connection_cnt == 0:
                self.jetson_status_update(True)
            self.jetson_connection_cnt = self.NETWORK_CONNTECTION_COUNTER_TRACK
        elif (packet.hdr.addr.src == self.vfr.DEVICE_ADDR_CONTROLLER):
            if self.controller_connection_cnt == 0:
                self.controller_status_update(True)
            self.controller_connection_cnt = self.NETWORK_CONNTECTION_COUNTER_TRACK
        return
    
    
    def packet_parse_fuzzy_coef_resp(self, packet):
        row_in_3_names          = ["PS", "PM", "PB"]
        row_in_3_namess         = ["NE", "ZE", "PO"]
        row_in_5_names          = ["NB", "NS", "ZE", "PS", "PB"]
        row_out_speed_names     = ["speed"]
        row_out_nor_coef_names  = ["coef"]
        row_out_angle_names     = ["angle"]
        col_in_2_names          = ["error", "derivative_error"]
        col_in_4_names          = ["left", "center_left", "center_right", "right"]
        col_out_3_names         = ["PS", "PM", "PB"]
        col_out_7_names         = ["NB", "NM", "NS", "ZE", "PS", "PM", "PB"]
        col_out_9_names        = ["NB", "NM", "NS", "NVS", "ZE", "PVS", "PS", "PM", "PB"]

        nor_coef                = packet.fuzzy_coef_resp.fuzzy_coef.nor_coef
        e_in_rule               = packet.fuzzy_coef_resp.fuzzy_coef.e_in_rule
        e_in_dot_rule           = packet.fuzzy_coef_resp.fuzzy_coef.e_dot_in_rule
        theta_in_rule           = packet.fuzzy_coef_resp.fuzzy_coef.theta_in_rule
        velo_in_rule            = packet.fuzzy_coef_resp.fuzzy_coef.velo_in_rule
        theta_out_rule          = packet.fuzzy_coef_resp.fuzzy_coef.theta_out_rule
        theta_out_3_input_rule  = packet.fuzzy_coef_resp.fuzzy_coef.theta_out_3_input_rule
        velo_out_rule           = packet.fuzzy_coef_resp.fuzzy_coef.velo_out_rule
        phi_in_rule             = packet.fuzzy_coef_resp.fuzzy_coef.phi_in_rule

        for row, rule_name in enumerate(row_in_5_names):
            rule_obj = getattr(e_in_rule, rule_name)             
            for col, col_name in enumerate(col_in_4_names):
                value = getattr(rule_obj, col_name)  
                item = QTableWidgetItem(f"{value:.3f}")  
                item.setTextAlignment(Qt.AlignCenter)  
                self.ui_user.table_def_in_error.setItem(row, col, item)  

        for row, rule_name in enumerate(row_in_5_names):
            rule_obj = getattr(e_in_dot_rule, rule_name)             
            for col, col_name in enumerate(col_in_4_names):
                value = getattr(rule_obj, col_name)  
                item = QTableWidgetItem(f"{value:.3f}")  
                item.setTextAlignment(Qt.AlignCenter)  
                self.ui_user.table_def_in_derivative_error.setItem(row, col, item)  

        for row, rule_name in enumerate(row_in_5_names):
            rule_obj = getattr(theta_in_rule, rule_name)             
            for col, col_name in enumerate(col_in_4_names):
                value = getattr(rule_obj, col_name)  
                item = QTableWidgetItem(f"{value:.3f}")  
                item.setTextAlignment(Qt.AlignCenter)  
                self.ui_user.table_def_in_rotation_angle.setItem(row, col, item)  

        for row, rule_name in enumerate(row_in_3_names):
            rule_obj = getattr(velo_in_rule, rule_name)             
            for col, col_name in enumerate(col_in_4_names):
                value = getattr(rule_obj, col_name)  
                item = QTableWidgetItem(f"{value:.3f}")  
                item.setTextAlignment(Qt.AlignCenter)  
                self.ui_user.table_def_in_speed.setItem(row, col, item)  

        for row, rule_name in enumerate(row_in_3_namess):
            rule_obj = getattr(phi_in_rule, rule_name)             
            for col, col_name in enumerate(col_in_4_names):
                value = getattr(rule_obj, col_name)  
                item = QTableWidgetItem(f"{value:.3f}")  
                item.setTextAlignment(Qt.AlignCenter)  
                self.ui_user.table_def_in_phi.setItem(row, col, item)   

        for col, col_name in enumerate(col_out_3_names):
            value = getattr(velo_out_rule, col_name)  
            item = QTableWidgetItem(f"{value:.3f}")  
            item.setTextAlignment(Qt.AlignCenter)  
            self.ui_user.table_def_out_speed.setItem(0, col, item)  

        value = nor_coef.e_nor_following_state
        item = QTableWidgetItem(f"{value:.3f}")  
        item.setTextAlignment(Qt.AlignCenter)  
        self.ui_user.table_def_nor_coef.setItem(0, 0, item)  

        value = nor_coef.e_dot_nor_following_state
        item = QTableWidgetItem(f"{value:.3f}")  
        item.setTextAlignment(Qt.AlignCenter)  
        self.ui_user.table_def_nor_coef.setItem(0, 1, item) 

        value = nor_coef.e_nor_turning_state
        item = QTableWidgetItem(f"{value:.3f}")  
        item.setTextAlignment(Qt.AlignCenter)  
        self.ui_user.table_def_nor_coef.setItem(1, 0, item)  

        value = nor_coef.e_dot_nor_turning_state
        item = QTableWidgetItem(f"{value:.3f}")  
        item.setTextAlignment(Qt.AlignCenter)  
        self.ui_user.table_def_nor_coef.setItem(1, 1, item)  

        value = nor_coef.e_phi_nor
        item = QTableWidgetItem(f"{value:.3f}")  
        item.setTextAlignment(Qt.AlignCenter)  
        self.ui_user.table_def_nor_coef.setItem(2, 0, item)  

        for col, col_name in enumerate(col_out_7_names):
            value = getattr(theta_out_rule, col_name)  
            item = QTableWidgetItem(f"{value:.3f}")  
            item.setTextAlignment(Qt.AlignCenter)  
            self.ui_user.table_def_out_rotation_angle.setItem(0, col, item)  

        for col, col_name in enumerate(col_out_9_names):
            value = getattr(theta_out_3_input_rule, col_name)  
            item = QTableWidgetItem(f"{value:.3f}")  
            item.setTextAlignment(Qt.AlignCenter)  
            self.ui_user.table_def_out_rotation_angle_3_inputs.setItem(0, col, item)  

        return
    

    def packet_parse_speed_resp(self, packet):
        speed = packet.speed_resp.speed
        self.ui_user.outmsg_robot_current_speed.append(f'<span style="color: white;">{speed:.1f}</span>')
        return
    

    def packet_parse_speed_max_set_t(self, packet):
        max_speed_text = self.ui_user.inmsg_robot_max_speed.toPlainText().strip()
        max_speed = float(max_speed_text)
        if((max_speed >= 10) & (max_speed <= 20)):
            return
        else:
            return
        return


    def packet_parse_speed_max_resp_t(self, packet):
        max_speed = packet.speed_max_resp.speed
        self.ui_user.inmsg_robot_max_speed.append(f'<span style="color: white;">{max_speed}</span>')
        return
    

    def packet_parse_time_rsp(self, packet):
        return
    

    def packet_parse_robot_info_rsp(self, packet):
        process_state           = packet.robot_info_rsp.state_info.process_state
        direct_state            = packet.robot_info_rsp.state_info.direct_state
        camera_state            = packet.robot_info_rsp.state_info.camera_state            

        current_rotation_angle  = packet.robot_info_rsp.var_info.theta
        max_rotation_angle      = packet.robot_info_rsp.var_info.max_theta

        current_setpoint_speed      = packet.robot_info_rsp.var_info.current_setpoint_speed
        current_setpoint_speed_nor  = packet.robot_info_rsp.var_info.current_setpoint_speed_nor
        current_speed                = packet.robot_info_rsp.var_info.current_speed
        current_speed_nor           = packet.robot_info_rsp.var_info.current_speed_nor
        current_max_setpoint_speed  = packet.robot_info_rsp.var_info.max_setpoint_speed

        if self.current_max_speed_setpoint != current_max_setpoint_speed:
            self.current_max_speed_setpoint = current_max_setpoint_speed
            self.is_update_current_max_speed_setpoint = True


        current_error           = packet.robot_info_rsp.var_info.e
        current_e_phi           = packet.robot_info_rsp.var_info.e_phi
        time_last               = packet.robot_info_rsp.var_info.time_process

        self.update_graph(current_error, current_e_phi, current_speed, current_rotation_angle, time_last, camera_state)

        process_state_msg   = self.robot_process_state_msg_def[process_state]
        direct_state_msg    = self.robot_direct_state_msg_def[direct_state]
        camera_state_msg    = self.robot_camera_state_msg_def[camera_state]

        current_rotation_angle_msg  = f"{current_rotation_angle:.2f}"
        current_setpoint_speed_msg  = f"{current_setpoint_speed:.2f}"
        current_speed_msg           = f"{current_speed:.2f}"
        current_max_speed_msg       = f"{current_max_setpoint_speed:.2f}"

        fields = [
            ("center:",     packet.robot_info_rsp.var_info.center),
            ("left:",       packet.robot_info_rsp.var_info.left_down_coor),
            ("right:",      packet.robot_info_rsp.var_info.right_down_coor),
            ("phi:",        packet.robot_info_rsp.var_info.phi),
            ("theta:",      packet.robot_info_rsp.var_info.theta),
            ("e:",          packet.robot_info_rsp.var_info.e),
            ("edot:",       packet.robot_info_rsp.var_info.edot),
            ("e_phi:",      packet.robot_info_rsp.var_info.e_phi),
            ("e_nor:",      packet.robot_info_rsp.var_info.e_nor),
            ("edot_nor:",   packet.robot_info_rsp.var_info.edot_nor),
            ("e_phi_nor:",  packet.robot_info_rsp.var_info.e_phi_nor),
            ("t_ms:",       packet.robot_info_rsp.var_info.time_process),
        ]

        info_msg_tmp = " | ".join(f"{label}<span style='padding-left: 5px;'>&nbsp;{value:>6.2f}</span>" for label, value in fields)
        info_msg_tmp = f"<b>{camera_state_msg}</b> | " + info_msg_tmp

        info_msg = f"""
            <html>
            <head>
                <style>
                    body {{
                        font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
                        font-size: 12pt;
                    }}
                </style>
            </head>
            <body>
                <pre>{info_msg_tmp}</pre>
            </body>
            </html>
"""

        self.ui_user.outmsg_robot_process_state.clear()
        self.ui_user.outmsg_robot_direct_state.clear()
        self.ui_user.outmsg_robot_camera_state.clear()

        self.ui_user.outmsg_robot_curren_rotation_angle.clear()
        self.ui_user.outmsg_robot_current_setpoint_speed.clear()
        self.ui_user.outmsg_robot_current_speed.clear()

        self.ui_user.outmsg_robot_process_state.append(f'<span style="color: white;">{process_state_msg}</span>')
        self.ui_user.outmsg_robot_direct_state.append(f'<span style="color: white;">{direct_state_msg}</span>')
        self.ui_user.outmsg_robot_camera_state.append(f'<span style="color: white;">{camera_state_msg}</span>')

        self.ui_user.outmsg_robot_curren_rotation_angle.append(f'<span style="color: white;">{current_rotation_angle_msg}</span>')
        self.ui_user.outmsg_robot_current_setpoint_speed.append(f'<span style="color: white;">{current_setpoint_speed_msg}</span>')
        self.ui_user.outmsg_robot_current_speed.append(f'<span style="color: white;">{current_speed_msg}</span>')

        if self.is_update_current_max_speed_setpoint is True:
            self.is_update_current_max_speed_setpoint = False
            self.ui_user.inmsg_robot_max_speed.clear()
            self.ui_user.inmsg_robot_max_speed.append(f'<span style="color: white;">{current_max_speed_msg}</span>')


        formatted_notif_msg = self.message_format_convert(packet.hdr.addr.src, info_msg, packet.hdr.epoch_time)
        message_converted = self.message_convert_nor(formatted_notif_msg)
        self.ui_user.outmsg_robot_msg.append(f'{message_converted}')

        return
    
    def packet_parse_debug_msg(self, packet):
        message = packet.debug_msg.message.decode("utf-8")
        formatted_msg = self.message_format_convert(packet.hdr.addr.src, message, packet.hdr.epoch_time)

        if (packet.debug_msg.message_type == proto.MESSAGE_INFO):
            message_converted = self.message_convert_info(formatted_msg)
        elif (packet.debug_msg.message_type == proto.MESSAGE_WARN):
            message_converted = self.message_convert_warning(formatted_msg)
        elif (packet.debug_msg.message_type == proto.MESSAGE_ERR):
            message_converted = self.message_convert_error(formatted_msg)
        elif (packet.debug_msg.message_type == proto.MESSAGE_NOR):
            message_converted = self.message_convert_nor(formatted_msg)

        if (packet.debug_msg.interface_dst == proto.MESSAGE_USER):
            self.ui_user.outmsg_robot_msg.append(f'{message_converted}')
        if (packet.debug_msg.interface_dst == proto.MESSAGE_USER_NOTIF):
            self.ui_user.outmsg_robot_msg_notif.append(f'{message_converted}')
        elif (packet.debug_msg.interface_dst == proto.MESSAGE_PROGRAMMER):
            self.ui_user.outmsg_program_msg.append(f'{message_converted}')

        return
    
    def packet_parse_vision_coordinate_detected(self, packet):
        # print("Coor packet received.")
        # center_coor = packet.vision_coordinate_detected.center_coor 
        # phi = packet.vision_coordinate_detected.phi

        left_down_coor = packet.vision_coordinate_detected.left_down_coor 
        right_down_coor = packet.vision_coordinate_detected.right_down_coor
        is_left_lane_available = packet.vision_coordinate_detected.is_left_lane_available 
        is_right_lane_available = packet.vision_coordinate_detected.is_right_lane_available
        is_crossing_crossroads = packet.vision_coordinate_detected.is_crossing_crossroads



        # coor_msg = f"left_down_coor: {left_down_coor:.1f}. right_down_coor: {right_down_coor:.1f}. is_left_lane_available: {is_left_lane_available}. is_right_lane_available: {is_right_lane_available}."
        coor_msg = f"is_crossing_crossroads: {is_crossing_crossroads}."
        formatted_coor_msg = self.message_format_convert(packet.hdr.addr.src, coor_msg, packet.hdr.epoch_time)
        message_converted = self.message_convert_nor(formatted_coor_msg)
        # self.ui_user.outmsg_robot_msg.append(f'{message_converted}')

        return
    
    def packet_parse_signs_info(self, packet):
        message = f"""== Sign Detection Status ==
        Bus_Stop: {packet.signs_info.signs_detected.bus_stop} | Children_Crossing: {packet.signs_info.signs_detected.children_crossing} | Green_Light: {packet.signs_info.signs_detected.green_light} | Left_Turn_Only: {packet.signs_info.signs_detected.left_turn_only} | No_Stopping: {packet.signs_info.signs_detected.no_stopping} | Red_Light: {packet.signs_info.signs_detected.red_light} | Speed_Limit_40: {packet.signs_info.signs_detected.speed_limit_40} | Stop: {packet.signs_info.signs_detected.stop}
        """

        formatted_msg = self.message_format_convert(packet.hdr.addr.src, message, packet.hdr.epoch_time)
        message_converted = self.message_convert_info(formatted_msg)
        # self.ui_user.outmsg_robot_msg.append(f'{message_converted}')

        # Danh sách bool
        sign_bools = [
            packet.signs_info.signs_detected.bus_stop,
            packet.signs_info.signs_detected.children_crossing,
            packet.signs_info.signs_detected.green_light,
            packet.signs_info.signs_detected.left_turn_only,
            packet.signs_info.signs_detected.no_stopping,
            packet.signs_info.signs_detected.red_light,
            packet.signs_info.signs_detected.speed_limit_40,
            packet.signs_info.signs_detected.stop,
        ]

        self.clear_signs_pics()

        index = 0
        for i, detected in enumerate(sign_bools):
            if detected and index < 6:
                pixmap = QPixmap(f"signs_pic/{self.sign_images[i]}")
                if not pixmap.isNull():
                    # scale ảnh để vừa label, giữ tỉ lệ
                    scaled_pixmap = pixmap.scaled(
                        self.sign_labels[index].width(),
                        self.sign_labels[index].height(),
                        Qt.AspectRatioMode.KeepAspectRatio,
                        Qt.TransformationMode.SmoothTransformation
                    )
                    self.sign_labels[index].setPixmap(scaled_pixmap)
                    self.sign_labels[index].setAlignment(Qt.AlignmentFlag.AlignCenter)
                    index += 1

        return
    
    def packet_parse_direction_list(self, packet):
        cmd_list = [
            packet.direction_list.first,
            packet.direction_list.second,
            packet.direction_list.third,
            packet.direction_list.fourth,
            packet.direction_list.fifth,
            packet.direction_list.sixth,
            packet.direction_list.seventh,
            packet.direction_list.eighth,
            packet.direction_list.ninth,
            packet.direction_list.tenth,
        ]

        # Chuyển từng lệnh thành chuỗi mô tả
        msg_list = [
            f"{i+1}. {self.robot_direction_cmd_def[cmd]}"
            for i, cmd in enumerate(cmd_list)
        ]

        final_msg = "List of Direction Commands:\n" + "\n".join(msg_list)

        self.ui_user.outmsg_action_list.clear()
        self.ui_user.outmsg_action_list.append(final_msg)

        return
    

    packet_parse_def = {
        "none"                              :   {"parser": None},
        "network_ack"                       :   {"parser": None},
        "network_status_get"                :   {"parser": None},
        "netowrk_status_rsp"                :   {"parser": packet_parse_netowrk_status_rsp},
        "robot_cmd_start"                   :   {"parser": None},
        "robot_cmd_stop"                    :   {"parser": None},
        "fuzzy_coef_set"                    :   {"parser": None},
        "fuzzy_coef_get"                    :   {"parser": None},
        "fuzzy_coef_resp"                   :   {"parser": packet_parse_fuzzy_coef_resp},
        "speed_get"                         :   {"parser": None},
        "speed_resp"                        :   {"parser": packet_parse_speed_resp},
        "speed_max_set"                     :   {"parser": packet_parse_speed_max_set_t},
        "speed_max_get"                     :   {"parser": None},
        "speed_max_resp"                    :   {"parser": None},
        "time_get"                          :   {"parser": None},
        "time_set"                          :   {"parser": None},
        "time_rsp"                          :   {"parser": packet_parse_time_rsp},
        "robot_process_state_get"           :   {"parser": None},
        "robot_process_state_set"           :   {"parser": None},
        "robot_process_state_rsp"           :   {"parser": None},
        "robot_direct_state_get"            :   {"parser": None},
        "robot_direct_state_set"            :   {"parser": None},
        "robot_direct_state_rsp"            :   {"parser": None},
        "robot_state_get"                   :   {"parser": None},
        "robot_state_rsp"                   :   {"parser": None},
        "sign_detected"                     :   {"parser": None},
        "vision_coordinate_detected"        :   {"parser": packet_parse_vision_coordinate_detected},
        "robot_control_cmd"                 :   {"parser": None},
        "vision_start"                      :   {"parser": None},
        "vision_stop"                       :   {"parser": None},
        "robot_angle_get"                   :   {"parser": None},
        "robot_angle_rsp"                   :   {"parser": None},
        "robot_max_angle_get"               :   {"parser": None},
        "robot_max_angle_set"               :   {"parser": None},
        "robot_max_angle_rsp"               :   {"parser": None},
        "robot_info_get"                    :   {"parser": None},
        "robot_info_rsp"                    :   {"parser": packet_parse_robot_info_rsp},
        "debug_msg"                         :   {"parser": packet_parse_debug_msg},
        "robot_direction_cmd"               :   {"parser": None},
        "cv_command"                        :   {"parser": None},
        "signs_info"                        :   {"parser": packet_parse_signs_info},
        "direction_list"                    :   {"parser": packet_parse_direction_list},
    }


    robot_process_state_msg_def = {
        proto.ROBOT_STATE_IDLE          :   "IDLE",
        proto.ROBOT_STATE_RUNNING       :   "RUNNING",
    }

    robot_direct_state_msg_def = {
        proto.ROBOT_STATE_STOP          :   "STOP",
        proto.ROBOT_STATE_MOVE_FORWARD  :   "MOVE FORWARD",
        proto.ROBOT_STATE_TURN_RIGHT    :   "TURN RIGHT",
        proto.ROBOT_STATE_TURN_LEFT     :   "TURN LEFT",
        proto.ROBOT_STATE_MOVE_BACKWARD :   "MOVE BACKWARD",
        proto.ROBOT_STATE_FOLLOW_LANE   :   "FOLLOW LANE",
        proto.ROBOT_STATE_STOP_TEMP     :   "TEMPORARY STOP",
    }

    robot_camera_state_msg_def = {
        proto.ROBOT_CAMERA_STATE_0      :   "ZERO",
        proto.ROBOT_CAMERA_STATE_P_30   :   "P30",
        proto.ROBOT_CAMERA_STATE_P_45   :   "P45",
        proto.ROBOT_CAMERA_STATE_N_30   :   "N30",
        proto.ROBOT_CAMERA_STATE_N_45   :   "N45",
        proto.ROBOT_CAMERA_STATE_MAX    :   "MAX",
    }

    robot_camera_state_colors_def = {
        proto.ROBOT_CAMERA_STATE_0      :   "aqua",
        proto.ROBOT_CAMERA_STATE_P_30   :   "lime",
        proto.ROBOT_CAMERA_STATE_P_45   :   "yellow",
        proto.ROBOT_CAMERA_STATE_N_30   :   "magenta",
        proto.ROBOT_CAMERA_STATE_N_45   :   "pink",
    }

    robot_direction_cmd_def = {
        proto.DIRECTION_CMD_MOVE_FORWARD        :   "MOVE FORWARD",
        proto.DIRECTION_CMD_MOVE_TURN_LEFT      :   "TURN LEFT",
        proto.DIRECTION_CMD_MOVE_TURN_RIGHT     :   "TURN RIGHT",
        proto.DIRECTION_CMD_MOVE_FOLLOW_LANE    :   "FOLLOW LANE",
        proto.DIRECTION_CMD_NONE                :   "NONE",
    }

    
    def clear_signs_pics(self):
        for label in self.sign_labels:
            label.clear()


    def validate_input(self, row, col):
        table = self.sender()
        item = table.item(row, col)

        if item is None:            
            return

        try:
            value = float(item.text().strip()) 
            if value < -1 or value > 1:
                self.ui_user.outmsg_program_msg.append(f'<span style="color: red;">[Error] Invalid Input. Input value must be between -1 and 1!</span>')
                item = QTableWidgetItem("0")
                item.setTextAlignment(Qt.AlignCenter)
                table.setItem(row, col, item)           
            else:
                return
        except ValueError:
            self.ui_user.outmsg_program_msg.append(f'<span style="color: red;">[Error] Invalid Input. Input value must be between -1 and 1!</span>')
            table.blockSignals(True)
            table.setItem(row, col, QTableWidgetItem(""))
            table.blockSignals(False)

        
    def validate_nor_coef_input(self, row, col):
        table = self.sender()
        item = table.item(row, col)

        if item is None:            
            return
        
        try:
            value = float(item.text().strip()) 
            if value < -999999999 or value > 999999999:
                self.ui_user.outmsg_program_msg.append(f'<span style="color: red;">[Error] Invalid Input. Input value must be a real number.')
            else:
                return
        except ValueError:
            self.ui_user.outmsg_program_msg.append(f'<span style="color: red;">[Error] Invalid Input. Input value must be a real number.')
            table.blockSignals(True)
            table.setItem(row, col, QTableWidgetItem(""))
            table.blockSignals(False)
    

    def set_robot_fuzzy_coef(self):
        packet = proto.network_packet_t()

        rule_5_fuzzy = ["NB", "NS", "ZE", "PS", "PB"]
        
        for i, rule_name in enumerate(rule_5_fuzzy):
            row_data = self.get_row_data_from_table(self.ui_user.table_def_in_error, i, 4)

            rule_obj = getattr(packet.fuzzy_coef_set.fuzzy_coef.e_in_rule, rule_name)
            rule_obj.left = row_data[0]
            rule_obj.center_left = row_data[1]
            rule_obj.center_right = row_data[2]
            rule_obj.right = row_data[3]

        for i, rule_name in enumerate(rule_5_fuzzy):
            row_data = self.get_row_data_from_table(self.ui_user.table_def_in_derivative_error, i, 4)

            rule_obj = getattr(packet.fuzzy_coef_set.fuzzy_coef.e_dot_in_rule, rule_name)
            rule_obj.left = row_data[0]
            rule_obj.center_left = row_data[1]
            rule_obj.center_right = row_data[2]
            rule_obj.right = row_data[3]

        for i, rule_name in enumerate(rule_5_fuzzy):
            row_data = self.get_row_data_from_table(self.ui_user.table_def_in_rotation_angle, i, 4)

            rule_obj = getattr(packet.fuzzy_coef_set.fuzzy_coef.theta_in_rule, rule_name)
            rule_obj.left = row_data[0]
            rule_obj.center_left = row_data[1]
            rule_obj.center_right = row_data[2]
            rule_obj.right = row_data[3]

        rule_3_positive_fuzzy = ["PS", "PM", "PB"]

        for i, rule_name in enumerate(rule_3_positive_fuzzy):
            row_data = self.get_row_data_from_table(self.ui_user.table_def_in_speed, i, 4)

            rule_obj = getattr(packet.fuzzy_coef_set.fuzzy_coef.velo_in_rule, rule_name)
            rule_obj.left = row_data[0]
            rule_obj.center_left = row_data[1]
            rule_obj.center_right = row_data[2]
            rule_obj.right = row_data[3]

        rule_3_fuzzy = ["NE", "ZE", "PO"]

        for i, rule_name in enumerate(rule_3_fuzzy):
            row_data = self.get_row_data_from_table(self.ui_user.table_def_in_phi, i, 4)

            rule_obj = getattr(packet.fuzzy_coef_set.fuzzy_coef.phi_in_rule, rule_name)
            rule_obj.left = row_data[0]
            rule_obj.center_left = row_data[1]
            rule_obj.center_right = row_data[2]
            rule_obj.right = row_data[3]

        out_robot_speed = self.get_row_data_from_table(self.ui_user.table_def_out_speed, 0, 3)
        out_rotation_angle = self.get_row_data_from_table(self.ui_user.table_def_out_rotation_angle, 0, 7)
        out_rotation_angle_3_inputs = self.get_row_data_from_table(self.ui_user.table_def_out_rotation_angle_3_inputs, 0, 9)
        following_normalization_coef = self.get_row_data_from_table(self.ui_user.table_def_nor_coef, 0, 2)
        turning_normalization_coef = self.get_row_data_from_table(self.ui_user.table_def_nor_coef, 1, 2)
        phi_normalization_coef = self.get_row_data_from_table(self.ui_user.table_def_nor_coef, 2, 1)

        packet.fuzzy_coef_set.fuzzy_coef.velo_out_rule.PS = out_robot_speed[0]
        packet.fuzzy_coef_set.fuzzy_coef.velo_out_rule.PM = out_robot_speed[1]
        packet.fuzzy_coef_set.fuzzy_coef.velo_out_rule.PB = out_robot_speed[2]

        packet.fuzzy_coef_set.fuzzy_coef.theta_out_rule.NB = out_rotation_angle[0]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_rule.NM = out_rotation_angle[1]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_rule.NS = out_rotation_angle[2]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_rule.ZE = out_rotation_angle[3]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_rule.PS = out_rotation_angle[4]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_rule.PM = out_rotation_angle[5]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_rule.PB = out_rotation_angle[6]

        packet.fuzzy_coef_set.fuzzy_coef.theta_out_3_input_rule.NB  = out_rotation_angle_3_inputs[0]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_3_input_rule.NM  = out_rotation_angle_3_inputs[1]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_3_input_rule.NS  = out_rotation_angle_3_inputs[2]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_3_input_rule.NVS = out_rotation_angle_3_inputs[3]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_3_input_rule.ZE  = out_rotation_angle_3_inputs[4]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_3_input_rule.PVS = out_rotation_angle_3_inputs[5]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_3_input_rule.PS  = out_rotation_angle_3_inputs[6]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_3_input_rule.PM  = out_rotation_angle_3_inputs[7]
        packet.fuzzy_coef_set.fuzzy_coef.theta_out_3_input_rule.PB  = out_rotation_angle_3_inputs[8]

        packet.fuzzy_coef_set.fuzzy_coef.nor_coef.e_nor = 150
        packet.fuzzy_coef_set.fuzzy_coef.nor_coef.e_dot_nor = 600
        packet.fuzzy_coef_set.fuzzy_coef.nor_coef.e_nor_following_state = following_normalization_coef[0]
        packet.fuzzy_coef_set.fuzzy_coef.nor_coef.e_dot_nor_following_state = following_normalization_coef[1]
        packet.fuzzy_coef_set.fuzzy_coef.nor_coef.e_nor_turning_state = turning_normalization_coef[0]
        packet.fuzzy_coef_set.fuzzy_coef.nor_coef.e_dot_nor_turning_state = turning_normalization_coef[1]
        packet.fuzzy_coef_set.fuzzy_coef.nor_coef.e_phi_nor = phi_normalization_coef[0]

        self.vfr.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet)

        return
    
    def get_row_data_from_table(self, table, row, size):
        row_data = []
        for col in range(size):
            item = table.item(row, col)
            value = float(item.text())
            row_data.append(value)
        return row_data
    

    def get_robot_fuzzy_coef(self):
        packet = proto.network_packet_t()
        packet.fuzzy_coef_get.dummy = 0
        self.vfr.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet)
        return
    

    def sync_robot_time(self):
        packet = proto.network_packet_t()
        packet.time_set.epoch_time = np.uint64(time.time() * 1000)
        self.vfr.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet)
        return
    

    def update_robot_status(self):
        max_angle_text = self.ui_user.inmsg_max_rotation_angle.toPlainText().strip()
        max_speed_text = self.ui_user.inmsg_robot_max_speed.toPlainText().strip()

        max_angle_valid = False;
        max_speed_valid = False;

        try:
            max_angle = float(max_angle_text)
            max_speed = float(max_speed_text)
            # if(max_angle <= 0 or max_angle > 55 ):
            #     self.ui_user.outmsg_robot_msg.append(f'<span style="color: yellow;">[Warning] Invalid max angle input. Must be a positive real number.</span>')
            # else:
            #     max_angle_valid = True

            max_angle_valid = True

            if(max_speed <= 0):
                self.ui_user.outmsg_robot_msg_notif.append(f'<span style="color: yellow;">[Warning] Invalid max speed input. Must be a positive real number.</span>')
            else:
                max_speed_valid = True
        except:
            self.ui_user.outmsg_robot_msg_notif.append(f'<span style="color: yellow;">[Warning] Invalid input. Must be a positive real number.</span>')
            return
        
        if((not max_angle_valid) or (not max_speed_valid)):
            return
        
        packet_max_angle_set = proto.network_packet_t()
        packet_max_angle_set.robot_max_angle_set.angle = max_angle

        packet_max_speed_set = proto.network_packet_t()
        packet_max_speed_set.speed_max_set.speed = max_speed

        self.vfr.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet_max_angle_set)
        # print(f'Speed max" {max_speed}')
        self.vfr.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet_max_speed_set)

        return


    def update_graph(self, error, phi, speed, angle, time_last, camera_state):
        # Cập nhật thời gian
        if not self.graph_first_start:
            self.graph_first_start = True
            self.graph_current_time = 0
        else:
            self.graph_current_time += (time_last / 1000)

        # Cập nhật giá trị hiện tại
        self.current_error = error
        self.current_phi = phi
        self.current_speed = speed
        self.current_angle = angle

        # Nếu trạng thái camera thay đổi hoặc chưa có curve ban đầu
        if self.camera_state != camera_state or self.current_curve_error is None:
            self.camera_state = camera_state
            pen_color = self.robot_camera_state_colors_def.get(self.camera_state, 'aqua')
            pen = pg.mkPen(color=pen_color, width=2, style=pg.QtCore.Qt.SolidLine)

            # Tạo mới các curve cho từng biểu đồ
            self.current_curve_error = pg.PlotCurveItem(pen=pen)
            self.current_curve_phi = pg.PlotCurveItem(pen=pen)
            self.current_curve_speed = pg.PlotCurveItem(pen=pen)
            self.current_curve_angle = pg.PlotCurveItem(pen=pen)

            # Thêm các curve mới vào widget
            self.plot_widget_error.addItem(self.current_curve_error)
            self.plot_widget_phi.addItem(self.current_curve_phi)
            self.plot_widget_speed.addItem(self.current_curve_speed)
            self.plot_widget_angle.addItem(self.current_curve_angle)

            # Lưu các curve cùng mảng dữ liệu để quản lý
            self.curves_error.append(([], [], self.current_curve_error))     # time, data, curve
            self.curves_phi.append(([], [], self.current_curve_phi))
            self.curves_speed.append(([], [], self.current_curve_speed))
            self.curves_angle.append(([], [], self.current_curve_angle))

        # Thêm dữ liệu mới vào curve hiện tại
        self.curves_error[-1][0].append(self.graph_current_time)
        self.curves_error[-1][1].append(self.current_error)
        self.curves_error[-1][2].setData(self.curves_error[-1][0], self.curves_error[-1][1])

        self.curves_phi[-1][0].append(self.graph_current_time)
        self.curves_phi[-1][1].append(self.current_phi)
        self.curves_phi[-1][2].setData(self.curves_phi[-1][0], self.curves_phi[-1][1])

        self.curves_speed[-1][0].append(self.graph_current_time)
        self.curves_speed[-1][1].append(self.current_speed)
        self.curves_speed[-1][2].setData(self.curves_speed[-1][0], self.curves_speed[-1][1])

        self.curves_angle[-1][0].append(self.graph_current_time)
        self.curves_angle[-1][1].append(self.current_angle)
        self.curves_angle[-1][2].setData(self.curves_angle[-1][0], self.curves_angle[-1][1])

        # Tính Y-range động cho từng biểu đồ
        def update_y_range(widget, data_list, min_pad=-1):
            flat_data = [v for curve in data_list for v in curve[1]]
            if flat_data:
                y_min = min(flat_data)
                y_max = max(flat_data)
                if y_min == y_max:
                    y_min -= 1
                    y_max += 1
                widget.setYRange(y_min if min_pad == -1 else min_pad, y_max, padding=0.1)

        update_y_range(self.plot_widget_error, self.curves_error)
        update_y_range(self.plot_widget_phi, self.curves_phi)
        update_y_range(self.plot_widget_speed, self.curves_speed, min_pad=0)
        update_y_range(self.plot_widget_angle, self.curves_angle)

        # X-range (chạy liên tục trong 20s gần nhất)
        visible_duration = 20  # giây
        for widget in [self.plot_widget_error, self.plot_widget_phi, self.plot_widget_speed, self.plot_widget_angle]:
            widget.setXRange(
                self.graph_current_time - visible_duration,
                self.graph_current_time,
                padding=0
            )

    
    def clear_chart(self):
        self.graph_first_start = False

        self.graph_data_time.clear()
        self.graph_data_error.clear()
        self.graph_data_phi.clear()
        self.graph_data_speed.clear()
        self.graph_data_angle.clear()

        # Cập nhật đường cong bằng data rỗng
        self.plot_curve_error.setData([], [])
        self.plot_curve_phi.setData([], [])
        self.plot_curve_speed.setData([], [])
        self.plot_curve_angle.setData([], [])

        # Reset lại range nếu cần
        self.plot_widget_error.enableAutoRange()
        self.plot_widget_phi.enableAutoRange()
        self.plot_widget_speed.enableAutoRange()
        self.plot_widget_angle.enableAutoRange()

        # Xóa dữ liệu trong từng curve đang hiển thị
        for curve in self.curves_error:
            curve[0].clear()  # x
            curve[1].clear()  # y
            curve[2].setData([], [])

        for curve in self.curves_phi:
            curve[0].clear()
            curve[1].clear()
            curve[2].setData([], [])

        for curve in self.curves_speed:
            curve[0].clear()
            curve[1].clear()
            curve[2].setData([], [])

        for curve in self.curves_angle:
            curve[0].clear()
            curve[1].clear()
            curve[2].setData([], [])


if __name__ == "__main__":
    app = QApplication([])
    window = VFR_GUI_main()
    window.show()
    sys.exit(app.exec())

