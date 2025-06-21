import serial
import serial.tools.list_ports
import time
import threading
import sys
import protocol_pb2 as proto
import numpy as np
import cv2
import pycuda.autoinit  # Khởi tạo CUDA driver
from utils.yolo_with_plugins import TrtYOLO
import multiprocessing
import datetime
import os

class VFR:
    DEVICE_ADDR_UNSPECIFIED     = 0x00
    DEVICE_ADDR_APP             = 0x01
    DEVICE_ADDR_TX2             = 0x02
    DEVICE_ADDR_CONTROLLER      = 0x03
    DEVICE_ADDR_DEBUG           = 0x04
    DEVICE_ADDR_BCAST           = 0x05

    DEVICE_TYPE_UNSPECIFIED     = 0x00
    DEVICE_TYPE_APP             = 0x01
    DEVICE_TYPE_TX2             = 0x02
    DEVICE_TYPE_CONTROLLER      = 0x03

    SERIAL_FRAME_SOF            = 0xAA55CDEF
    SERIAL_FRAME_SOF_LSB        = 0xEF
    SERIAL_FRAME_SOF_MID_LOW    = 0xCD
    SERIAL_FRAME_SOF_MID_HIGH   = 0x55
    SERIAL_FRAME_SOF_MSB        = 0xAA
    SERIAL_FRAME_SOF_SIZE       = 8
    SERIAL_FRAME_CHECKSUM_SIZE  = 1

    SERIAL_USB_NAME = "USB-SERIAL CH340"

    SERIAL_TIMEOUT_PER_FRAME    = 0.025

    SERIAL_STATE_IDLE               = 0x00
    SERIAL_STATE_FIND_LSB_SOF       = 0x01
    SERIAL_STATE_FIND_MID_LOW_SOF   = 0x02
    SERIAL_STATE_FIND_MID_HIGH_SOF  = 0x03
    SERIAL_STATE_FIND_MSB_SOF       = 0x04
    SERIAL_STATE_FIND_LENGTH        = 0x05
    SERIAL_STATE_FIND_DATA          = 0x06

    IDX_SOF_LSB         = 0
    IDX_SOF_MID_LOW     = 1
    IDX_SOF_MID_HIGH    = 2
    IDX_SOF_MBS         = 3
    IDX_LEN_LSB         = 4
    IDX_LEN_MSB         = 7
    IDX_DATA            = 8

    FZ_CTRLR_COORDINATE_LEFT_SETPOINT             = 317.0  
    FZ_CTRLR_COORDINATE_RIGHT_SETPOINT            = 1029.0 
    FZ_CTRLR_COORDINATE_TURN_LEFT_LEFT_SETPOINT   = 208.0  
    FZ_CTRLR_COORDINATE_TURN_RIGHT_RIGHT_SETPOINT = 1117.0 

    VISION_TIMEOUT_30_FPS = (1.0/30.0)

    GST_PIPELINE = (
        "udpsrc port=8554 buffer-size=100000000 ! "
        "tsdemux ! h264parse ! nvv4l2decoder disable-dpb=true ! "
        "nvvidconv ! video/x-raw(memory:NVMM), format=NV12, width=1280, height=720 ! "
        "nvvidconv ! video/x-raw, format=RGBA ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink sync=false async=false max-buffers=1 drop=true"
    )


    def __init__(
        self,
        com_port        = None,
        com_name        = SERIAL_USB_NAME,
        com_rate        = 115200,
        src             = DEVICE_ADDR_TX2,
        dst             = DEVICE_ADDR_CONTROLLER,
        connected       = False,
        autoconnect     = True,
        debug           = False,
        gui_process     = False,
    ):
        super().__init__()

        # Declare local UART Network Process variable
        self.com_port                   = com_port
        self.com_name                   = com_name
        self.com_rate                   = com_rate
        self.src                        = src
        self.dst                        = dst
        self.connected                  = connected
        self.autoconnect                = autoconnect
        self.serial                     = None
        self.uart_buffer                = bytearray()
        self.encoded_packet             = bytearray()
        self.encoded_packet_lenth       = 0
        self.frame_without_checksum     = bytearray()
        self.frame_checksum             = 0x00
        self.start_time                 = None
        self.elapsed_time               = None
        self.debug                      = debug
        self.gui_process                = gui_process
        self.serial_state               = self.SERIAL_STATE_IDLE
        self.thread_start               = False
        self.cnt                        = 0
        self.vision_start               = False
        self.vision_time_start          = 0
        self.vision_time_duration       = 0
        self.vision_cv_sign_ready       = False
        self.vision_cv_sign_process     = False
        self.cam_ret                    = False
        self.cam_frame                  = None
        self.cv_process_crossroads      = False

        # Declare UART Process thread
        self.thread = threading.Thread(target=self.read_packet, name="PacketReceiver", daemon=True) 
        
        # Initialize UART
        self.serial = serial.Serial('/dev/ttyTHS2', baudrate=self.com_rate, timeout=1)
        self.start_receiving_and_processing_packet()


    def checksum(self, buff):
        sum = 0
        for byte in buff:
            sum += byte
        return (sum & 0xFF)  # Truncate to 8 bits
    
    def serial_send(self, buf):
        try:
            time.sleep(0.005)
            self.serial.write(buf)
            return True
        except Exception as e:
            return False


    def send_packet(self, dst, packet):
        if self.connected:
            packet.hdr.addr.src = self.src
            packet.hdr.addr.dst = dst
            # packet.hdr.epoch_time = np.uint64((time.time() + (7 * 3600)) * 1000)
            packet.hdr.epoch_time = np.uint64(time.time() * 1000)
            
            encode_packet = packet.SerializeToString()

            serial_frame = bytearray()
            serial_frame += self.SERIAL_FRAME_SOF.to_bytes(4, "little")
            serial_frame += len(encode_packet).to_bytes(4, "little")
            serial_frame += encode_packet
            checksum = self.checksum(serial_frame)
            serial_frame += checksum.to_bytes(1, "little")

            if self.serial_send(serial_frame):
                return True
            return False

    def read_packet(self):
        self.start_time = None
        self.serial_state = self.SERIAL_STATE_FIND_LSB_SOF
        while self.connected:
            try:
                if self.serial.in_waiting > 0:
                    byte = self.serial.read(1)
                    self.uart_buffer.extend(byte)
                    self.start_time = time.time()

                if(len(self.uart_buffer) >= 1):
                    if (self.serial_state == self.SERIAL_STATE_FIND_LSB_SOF):
                        if (self.uart_buffer[self.IDX_SOF_LSB] == self.SERIAL_FRAME_SOF_LSB):
                            self.serial_state = self.SERIAL_STATE_FIND_MID_LOW_SOF
                        else:
                            self.uart_buffer.clear()
                        continue
                        
                                
                    if self.start_time is not None:
                        self.elapsed_time = time.time() - self.start_time
                        if (self.elapsed_time > self.SERIAL_TIMEOUT_PER_FRAME):
                            self.uart_buffer.clear()
                            self.encoded_packet.clear()
                            self.frame_without_checksum.clear()
                            self.start_time = None
                            self.encoded_packet_lenth = 0
                            self.frame_checksum = 0x00
                            self.serial_state = self.SERIAL_STATE_FIND_LSB_SOF
                            continue

                        if (self.serial_state == self.SERIAL_STATE_FIND_MID_LOW_SOF):
                            if (len(self.uart_buffer) >= 2):
                                if (self.uart_buffer[self.IDX_SOF_MID_LOW] == self.SERIAL_FRAME_SOF_MID_LOW):
                                    self.serial_state = self.SERIAL_STATE_FIND_MID_HIGH_SOF
                                else:
                                    self.uart_buffer.clear()
                                    self.start_time = None
                                    self.serial_state = self.SERIAL_STATE_FIND_LSB_SOF
                            continue

                        if (self.serial_state == self.SERIAL_STATE_FIND_MID_HIGH_SOF):
                            if (len(self.uart_buffer) >= 3):
                                if (self.uart_buffer[self.IDX_SOF_MID_HIGH] == self.SERIAL_FRAME_SOF_MID_HIGH):
                                    self.serial_state = self.SERIAL_STATE_FIND_MSB_SOF
                                else:
                                    self.uart_buffer.clear()
                                    self.start_time = None
                                    self.serial_state = self.SERIAL_STATE_FIND_LSB_SOF
                            continue

                        if (self.serial_state == self.SERIAL_STATE_FIND_MSB_SOF):
                            if (len(self.uart_buffer) >= 4):
                                if (self.uart_buffer[self.IDX_SOF_MBS] == self.SERIAL_FRAME_SOF_MSB):
                                    self.serial_state = self.SERIAL_STATE_FIND_LENGTH
                                else:
                                    self.uart_buffer.clear()
                                    self.start_time = None
                                    self.serial_state = self.SERIAL_STATE_FIND_LSB_SOF
                            continue

                        elif (self.serial_state == self.SERIAL_STATE_FIND_LENGTH):
                            if ((len(self.uart_buffer)) >= 8):
                                self.encoded_packet_lenth = int.from_bytes(bytes(self.uart_buffer[self.IDX_LEN_LSB : (self.IDX_LEN_MSB + 1)]), "little",)
                                self.serial_state = self.SERIAL_STATE_FIND_DATA
                            continue

                        elif (self.serial_state == self.SERIAL_STATE_FIND_DATA):
                            if ((self.encoded_packet_lenth > 0) and (len(self.uart_buffer) >= (self.SERIAL_FRAME_SOF_SIZE + self.encoded_packet_lenth + self.SERIAL_FRAME_CHECKSUM_SIZE))):
                                self.encoded_packet = self.uart_buffer[self.SERIAL_FRAME_SOF_SIZE : (self.SERIAL_FRAME_SOF_SIZE + self.encoded_packet_lenth)]
                                self.frame_without_checksum = self.uart_buffer[ : (self.SERIAL_FRAME_SOF_SIZE + self.encoded_packet_lenth)]
                                self.frame_checksum = self.uart_buffer[self.SERIAL_FRAME_SOF_SIZE + self.encoded_packet_lenth]
                                self.check_and_process_packet()
                                self.uart_buffer.clear()
                                self.encoded_packet.clear()
                                self.frame_without_checksum.clear()
                                self.start_time = None
                                self.encoded_packet_lenth = 0
                                self.frame_checksum = 0x00
                                self.serial_state = self.SERIAL_STATE_FIND_LSB_SOF
                            continue
    
            except Exception  as e:
                time.sleep(0.01)
                while not self.connected:
                    time.sleep(0.01)
                    continue
                self.encoded_packet.clear()
                self.frame_without_checksum.clear()
                self.start_time = None
                self.encoded_packet_lenth = 0
                self.frame_checksum = 0x00
                self.serial_state = self.SERIAL_STATE_FIND_LSB_SOF
                continue
                

    def check_and_process_packet(self):
        checksum = self.checksum(self.frame_without_checksum)
        if (checksum != self.frame_checksum):
            return

        packet = proto.network_packet_t()
        packet.ParseFromString(bytes(self.encoded_packet))

        if ((packet.hdr.addr.dst != self.src) and (packet.hdr.addr.dst != self.DEVICE_ADDR_BCAST)):
            return
        
        try:
            if packet.WhichOneof("params") != None:
                packet_tag = packet.WhichOneof("params")
        except Exception as e:
            pass
            
        self.packet_parse_def[packet_tag]["parser"](self, packet)        
        return
        

    def start_receiving_and_processing_packet(self):
        self.connected = True
        print("Start receiving and processing packet.")
        if not self.thread_start:
            self.thread_start = True
            self.thread.start()
            # self.thread_cv_sign.start()

    def stop_receiving_and_processing_packet(self):
        self.serial.close()
        self.connected = False
        self.uart_buffer.clear()
        self.encoded_packet.clear()
        self.frame_without_checksum.clear()
        self.start_time = None
        self.encoded_packet_lenth = 0
        self.frame_checksum = 0x00
        self.serial_state = self.SERIAL_STATE_FIND_LSB_SOF

    def update_cv_process(self, angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2, is_cross_road, is_left_lane_available, is_right_lane_available):
        packet = proto.network_packet_t()
        packet.vision_coordinate_detected.left_down_coor            = x_left_1
        packet.vision_coordinate_detected.right_down_coor           = x_right_1
        packet.vision_coordinate_detected.left_up_coor              = x_left_2
        packet.vision_coordinate_detected.right_up_coor             = x_right_2
        packet.vision_coordinate_detected.center_coor               = x_center
        packet.vision_coordinate_detected.phi                       = angle
        packet.vision_coordinate_detected.is_crossing_crossroads    = is_cross_road
        packet.vision_coordinate_detected.is_left_lane_available    = is_left_lane_available
        packet.vision_coordinate_detected.is_right_lane_available   = is_right_lane_available
        # print("Sending CV results.")
        self.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet)
        # self.send_packet(proto.DEVICE_ADDR_BCAST, packet)

    def update_sign_process(self, detected_flag, obey_flag):
        packet = proto.network_packet_t()
        
        packet.signs_info.signs_detected.bus_stop          = False
        packet.signs_info.signs_detected.children_crossing = False
        packet.signs_info.signs_detected.green_light       = False
        packet.signs_info.signs_detected.left_turn_only    = False
        packet.signs_info.signs_detected.no_stopping       = False
        packet.signs_info.signs_detected.red_light         = False
        packet.signs_info.signs_detected.speed_limit_40    = False
        packet.signs_info.signs_detected.stop              = False

        packet.signs_info.signs_obey.bus_stop          = False
        packet.signs_info.signs_obey.children_crossing = False
        packet.signs_info.signs_obey.green_light       = False
        packet.signs_info.signs_obey.left_turn_only    = False
        packet.signs_info.signs_obey.no_stopping       = False
        packet.signs_info.signs_obey.red_light         = False
        packet.signs_info.signs_obey.speed_limit_40    = False
        packet.signs_info.signs_obey.stop              = False
        
        bools_detected  = np.array(detected_flag).astype(bool)
        bools_obey      = np.array(obey_flag).astype(bool)

        for name, value in zip(self.sign_names_def, bools_detected):
            setattr(packet.signs_info.signs_detected, name, bool(value))

        for name, value in zip(self.sign_names_def, bools_obey):
            setattr(packet.signs_info.signs_obey, name, bool(value))

        # print("Sending Signs results.")
        self.send_packet(proto.DEVICE_ADDR_BCAST, packet)


    def update_sign_obey_process(self, obey_flag):
        packet = proto.network_packet_t()
        packet.signs_detected_obey.bus_stop          = False
        packet.signs_detected_obey.children_crossing = False
        packet.signs_detected_obey.green_light       = False
        packet.signs_detected_obey.left_turn_only    = False
        packet.signs_detected_obey.no_stopping       = False
        packet.signs_detected_obey.red_light         = False
        packet.signs_detected_obey.speed_limit_40    = False
        packet.signs_detected_obey.stop              = False
        
        bools = np.array(obey_flag).astype(bool)

        for name, value in zip(self.sign_names_def, bools):
            setattr(packet.signs_detected_obey, name, bool(value))

        # print("Sending Signs results.")
        self.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet)


    def packet_parse_network_status_get(self, packet_recv):
        packet = proto.network_packet_t()
        packet.netowrk_status_rsp.dummy = 0
        self.send_packet(packet_recv.hdr.addr.src, packet)


    def packet_parse_vision_start(self, packet_recv):
        self.vision_start           = True
        self.vision_cv_sign_ready   = True
        self.vision_cv_sign_process = False
        print("Start vision.")


    def packet_parse_vision_stop(self, packet_recv):
        self.vision_start           = False
        self.vision_cv_sign_ready   = False
        self.vision_cv_sign_process = False
        print("Stop vision.")

    def packet_parse_cv_command(self, packet_recv):
        self.cv_process_crossroads = packet_recv.cv_command.process_crossroads


    packet_parse_def = {
        "none"                          :   {"parser": None},
        "network_ack"                   :   {"parser": None},
        "network_status_get"            :   {"parser": packet_parse_network_status_get},
        "netowrk_status_rsp"            :   {"parser": None},
        "robot_cmd_start"               :   {"parser": None},
        "robot_cmd_stop"                :   {"parser": None},
        "fuzzy_coef_set"                :   {"parser": None},
        "fuzzy_coef_get"                :   {"parser": None},
        "fuzzy_coef_resp"               :   {"parser": None},
        "speed_get"                     :   {"parser": None},
        "speed_resp"                    :   {"parser": None},
        "speed_max_set"                 :   {"parser": None},
        "speed_max_get"                 :   {"parser": None},
        "speed_max_resp"                :   {"parser": None},
        "time_get"                      :   {"parser": None},
        "time_set"                      :   {"parser": None},
        "time_rsp"                      :   {"parser": None},
        "robot_process_state_get"       :   {"parser": None},
        "robot_process_state_set"       :   {"parser": None},
        "robot_process_state_rsp"       :   {"parser": None},
        "robot_direct_state_get"        :   {"parser": None},
        "robot_direct_state_set"        :   {"parser": None},
        "robot_direct_state_rsp"        :   {"parser": None},
        "robot_state_get"               :   {"parser": None},
        "robot_state_rsp"               :   {"parser": None},
        "sign_detected"                 :   {"parser": None},
        "vision_coordinate_detected"    :   {"parser": None},
        "robot_control_cmd"             :   {"parser": None},
        "vision_start"                  :   {"parser": packet_parse_vision_start},
        "vision_stop"                   :   {"parser": packet_parse_vision_stop},
        "robot_angle_get"               :   {"parser": None},
        "robot_angle_rsp"               :   {"parser": None},
        "robot_max_angle_get"           :   {"parser": None},
        "robot_max_angle_set"           :   {"parser": None},
        "robot_max_angle_rsp"           :   {"parser": None},
        "robot_info_get"                :   {"parser": None},
        "robot_info_rsp"                :   {"parser": None},
        "debug_msg"                     :   {"parser": None},
        "robot_direction_cmd"           :   {"parser": None},
        "signs_detected"                :   {"parser": None},
        "cv_command"                    :   {"parser": packet_parse_cv_command},
    }


    sign_names_def = [
        "bus_stop",
        "children_crossing",
        "green_light",
        "left_turn_only",
        "no_stopping",
        "red_light",
        "speed_limit_40",
        "stop"
    ]



def pre_process(image):
    # start_time = time.time()
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    blur_median = cv2.medianBlur(gray_image, 5)
    alpha = 2.25  # contrast > 1 → tăng contrast mạnh
    beta = -50    # thay đổi độ sáng
    # high_contrast = cv2.convertScaleAbs(blur_median, alpha=alpha, beta=beta)
    # inverted = 255 - high_contrast
    inverted = 255 - blur_median
    thresh_val, binary = cv2.threshold(inverted, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # end_time = time.time()
    # print("Time preprocess: {}".format(end_time - start_time))
    return binary

def find_nearest_white_points(edge_image, x_ref, y_ref):
    y = y_ref-520 #Do đã crop ảnh nên trong ảnh mới Y sẽ có giá trị từ 0 - 199
    row_pixels = edge_image[y, :]  # Lấy toàn bộ dòng y cần detect
    
    # Tìm các vị trí x có giá trị 255 (trắng)
    white_x_positions = np.where(row_pixels == 255)[0]

    if len(white_x_positions) == 0:
        # print("Không tìm thấy điểm trắng nào trên hàng này.")
        return 0, 1279

    # Tách điểm bên trái và bên phải x_ref
    left_points = [x for x in white_x_positions if x < x_ref] #tìm tất cả điểm trắng bên trái
    right_points = [x for x in white_x_positions if x > x_ref] #tìm tất cả điểm trắng bên phải

    # Lấy điểm gần nhất bên trái và bên phải
    x_left = max(left_points) if left_points else 0
    if len(left_points) > 10:
        for i in range(len(left_points)-1, 9, -1):
            if (left_points[i] - left_points[i-10] != 10):
                x_left = left_points[i-1]
            else:
                break

    x_right= min(right_points) if right_points else 1279
    if len(right_points) > 10:
        for i in range(len(right_points)-10):
            if (right_points[i+10] - right_points[i] != 10):
                x_right = right_points[i+1]
            else:
                break
    return x_left, x_right

#Tính toán góc và tính chính
def find_angle(edge_image,y_down, y_up, center, center_secondary):
    #tìm 4 điểm thuộc lane cần detect
    # x_left_1, x_left_2 là 2 điểm thuộc lane trái
    # x_right_1, x_right_2 là 2 điểm thuộc lane phải
    x_left_1, x_right_1 = find_nearest_white_points(edge_image, center, y_down)
    x_left_2, x_right_2 = find_nearest_white_points(edge_image, center, y_up)


    # if ((x_left_1 == 0)|(x_left_2==0))&(x_right_1<x_right_2): #Nếu không nhìn thấy Lane bên trái và x_right_1 nhỏ hơn x_right_2 => cần detect lại
    #     x_left_1, x_right_1 = find_nearest_white_points(edge_image, center_secondary, y_down) 
    #     x_left_2, x_right_2 = find_nearest_white_points(edge_image, center_secondary, y_up)
    # elif ((x_right_1 == 1279)|(x_right_2 == 1279))&(x_left_1 > x_left_2): #Nếu không nhìn thấy Lane bên phải và x_left_1 lớn hơn x_left_2 => cần detect lại
    #     x_left_1, x_right_1 = find_nearest_white_points(edge_image, center_secondary, y_down)
    #     x_left_2, x_right_2 = find_nearest_white_points(edge_image, center_secondary, y_up)
    
    #tìm điểm trung tâm
    x_center = (x_left_1 + x_right_1)/2 
    x_up = (x_left_2 + x_right_2)/2

    #tính toán góc
    tan = (x_center- x_up)/(y_down-y_up)
    angle = np.arctan(tan)*180/3.1415
    return angle, x_center, x_up, x_left_1, x_right_1, x_left_2, x_right_2

#Tìm 2 điểm có thể là điểm trung tâm
def find_center_road(image, thres_value):
    histogram = np.sum(image, axis=0)  

    current_length = 0  # Chiều dài của đoạn hiện tại
    count = 0

    max_length = 0  # Đoạn dài nhất
    start_index = 0  # Vị trí bắt đầu của đoạn dài nhất
    end_index = 0  # Vị trí kết thúc của đoạn dài nhất
    
    max_length_secondary = 0 # Đoạn dài thứ hai
    start_index_secondary = 0 # Vị trí bắt đầu của đoạn dài thứ hai
    end_index_secondary = 0 # Vị trí kết thúc của đoạn dài thứ hai

    for i in range(0, 1279):
        if (histogram[i] < thres_value):
            if current_length == 0: #Chiều dài của đoạn hiện tại bằng 0 có nghĩa là đang bắt đầu một đoạn mới
                start_index_tmp = i  # Lưu lại vị trí bắt đầu của đoạn mới
            current_length += 1 #Tăng biến chiều dài hiện tại
            count = 0 #Reset biến count
        else:
            if current_length > 0:
                count +=1 #Đếm số lần Fail
                if count == 5: 
                    if current_length > max_length: #Nếu chiều dài của đoạn hiện tại lớn hơn chiều dài đoạn max hiện tại

                        max_length_secondary = max_length #Đoạn max hiện tại sẽ trở thành đoạn dài thứ 2 No.2
                        start_index_secondary = start_index #Lưu lại vị trí bắt đầu của đoạn dài thứ 2 No.2
                        end_index_secondary = end_index  #Lưu lại vị trí kết thúc của đoạn dài thứ 2 No.2

                        max_length = current_length #Giá trị của đoạn dài nhất No.1 bằng chiều dài của đoạn hiện tại
                        start_index = start_index_tmp #Lưu lại vị trí bắt đầu của đoạn dài nhất No.1
                        end_index = i - 6  #Lưu lại vị trí kết thúc của đoạn dài nhất No.1; trừ cho 6 là do phải đếm số lần Fail

                    elif current_length > max_length_secondary:# Nếu chiều dài của đoạn hiện tại không lớn hơn chiều dài Max nhưng lại lớn hơn đoạn dài thứ 2 No.2
                        max_length_secondary = current_length #Giá trị của đoạn dài thứ 2No.2 bằng chiều dài của đoạn hiện tại
                        start_index_secondary = start_index_tmp #Lưu lại vị trí bắt đầu của đoạn dài thứ 2 No.2
                        end_index_secondary = i - 6  #Lưu lại vị trí kết thúc của đoạn dài thứ 2 No.2

                    current_length = 0 #Reset biến chiều dài của đoạn hiện tại
                    count = 0 #Reset lại biến đếm

    # Kiểm tra nếu đoạn cuối cùng là đoạn dài nhất
    # Do ở đoạn code ở phía trên nếu đoạn cuối cùng kéo dài tới hết bức ảnh hay giá trị X = 1279 thì sẽ bỏ sót việc kiểm tra đoạn cuối cùng
    if current_length > max_length:
        max_length_secondary = max_length
        start_index_secondary = start_index
        end_index_secondary = end_index 

        max_length = current_length
        start_index = start_index_tmp
        end_index = len(histogram) - 1

    # Kiểm tra nếu đoạn cuối cùng là đoạn dài thứ 2 No.2
    elif current_length > max_length_secondary:
        max_length_secondary = current_length
        start_index_secondary = start_index_tmp
        end_index_secondary = len(histogram) - 1 

    #Tính toán giá trị các điểm trung tâm dựa theo các đoạn vừa tìm được
    center = (start_index + end_index)//2 #Điểm trung tâm dựa theo đoạn dài nhất No.1
    center_secondary = (start_index_secondary+end_index_secondary)//2 #Điểm trung tâm dựa theo đoạn dài thứ 2 No.2
    
    if hasattr(find_center_road, "prev_center"):
        if abs(center - find_center_road.prev_center) > 400:
            center = find_center_road.prev_center
            start_index = find_center_road.prev_star_index
            end_index = find_center_road.prev_end_index

    # Lưu lại giá trị center hiện tại
    find_center_road.prev_center = center
    find_center_road.prev_star_index = start_index
    find_center_road.prev_end_index = end_index
    return center, center_secondary, start_index, end_index

def detect_in_cross_road(image, left_base, right_base):
    msk = image.copy()
    w_box = 40
    h_box = 40
    AREA_CON = 50
    y = 0
    size = 480//h_box
    x_left = [0] * size
    x_right = [0] * size 
    found_contour_left = [False]* size
    found_contour_right = [False]* size
    count = 0
    while y < image.shape[0]:
        # Left side
        best_cx = 0
        min_diff = float('inf')
        img = image[y:y+h_box, left_base-w_box:left_base+w_box]
        contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > AREA_CON:  # chỉ giữ contour có diện tích > 100 pixel
                found_contour_left[count]=True
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    diff = abs(cx - w_box)
                    if diff < min_diff:
                        best_cx = cx
                        min_diff = diff
        if best_cx != 0:
            left_base = left_base - w_box + best_cx
            x_left[count] = left_base
        else:
            x_left[count] = left_base
        # Right side
        best_cx = 0
        min_diff = float('inf')
        img = image[y:y+h_box, right_base-w_box:right_base+w_box]
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > AREA_CON:  # chỉ giữ contour có diện tích > 100 pixel
                found_contour_right[count]=True
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    diff = abs(cx - w_box)
                if diff < min_diff:
                    best_cx = cx
                    min_diff = diff
        if best_cx != 0: 
            right_base = right_base - w_box + best_cx
            x_right[count] = right_base 
        else:
            x_right[count] = right_base
        # cv2.rectangle(msk, (left_base - w_box, y), (left_base + w_box, y + h_box), (255, 255, 255), 2)
        # cv2.rectangle(msk, (right_base - w_box, y), (right_base + w_box, y + h_box), (255, 255, 255), 2)
        y += h_box
        count+=1
    if found_contour_left[size-1] and found_contour_right[size-1]:
        is_crossing = False
    else:
        is_crossing = True
    is_left_lane_available = not (found_contour_left[size-1] == 0 or found_contour_left[280//h_box-1] == 0)
    is_right_lane_available = not (found_contour_right[size-1] == 0 or found_contour_right[280//h_box-1] == 0)
    contour_left = 0
    contour_right = 0
    finish_left = False
    finish_right = False
    pos_left = 0
    pos_right = 0
    for i in range(size):
        #left side
        if not finish_left:
            if found_contour_left[i]:
                contour_left = 0
            elif not found_contour_left[i]:
                contour_left+=1
            if contour_left == 5 or i == (size-1):
                x1_left_contour = x_left[i-contour_left]
                x2_left_contour = x_left[i-contour_left-1]
                pos_left = h_box*(i-contour_left+1)
                finish_left = True
        #right side
        if not finish_right:
            if found_contour_right[i]:
                contour_right= 0
            elif not found_contour_right[i]:
                contour_right+=1
            if contour_right == 5 or i == (size-1):
                x1_right_contour = x_right[i-contour_right]
                x2_right_contour = x_right[i-contour_right-1]
                pos_right = h_box*(i-contour_right+1)
                finish_right = True

    slope_left = (x1_left_contour - x2_left_contour)/h_box
    slope_right = (x1_right_contour - x2_right_contour)/h_box

    x_left_1 = max(x1_left_contour + slope_left * (480 - pos_left), 0) 
    if x_left_1 == 0:
        slope_left = (x1_left_contour)/(480-pos_left)
        x_left_2 = slope_left*(200)
    else:
        x_left_2 = max(x1_left_contour + slope_left*(280 - pos_left), 0)

    x_right_1 = min(x1_right_contour + slope_right*(480 - pos_right), 1279)
    if x_right_1 == 1279:
        # print(x1_right_contour, pos_right)
        slope_right = (x1_right_contour-1279)/(480-pos_right)
        x_right_2 = 1279 + slope_right*200
    else:
        x_right_2 = min(x1_right_contour + slope_right*(280 - pos_right), 1279)

    x_center = (x_left_1 + x_right_1)//2
    x_center_up = (x_left_2+x_right_2)//2
    tan = (x_center- x_center_up)/(200)
    angle = np.arctan(tan)*180/3.1415

    # #Draw line
    # cv2.line(msk, (0, 280), (1279,280), color=(0, 0, 0), thickness=4)
    # cv2.line(msk, (0, 280), (1279,280), color=(255, 255, 255), thickness=2)
    # cv2.line(msk, (int(x_left_1), size*h_box), (int(x1_left_contour),pos_left), color=(0, 0, 0), thickness=4)
    # cv2.line(msk, (int(x_left_1), size*h_box), (int(x1_left_contour),pos_left), color=(255, 255, 255), thickness=2)
    # cv2.line(msk, (int(x_right_1), size*h_box), (int(x1_right_contour),pos_right), color=(0, 0, 0), thickness=4)
    # cv2.line(msk, (int(x_right_1), size*h_box), (int(x1_right_contour),pos_right), color=(255, 255, 255), thickness=2)
    # cv2.line(msk, (int(x_center), size*h_box), (int(x_center_up),280), color=(0, 0, 0), thickness=4)
    # cv2.line(msk, (int(x_center), size*h_box), (int(x_center_up),280), color=(255, 255, 255), thickness=2)
    # cv2.imshow("original", msk)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    
    return angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2, is_crossing, is_left_lane_available, is_right_lane_available

def detect_in_cross_road_v2(image, left_base, right_base):
    msk = image.copy()
    w_box = 40
    h_box = 40
    AREA_CON = 50
    size = 480//h_box
    y = 0
    x_left = [0] * size
    x_right = [0] * size 
    found_contour_left = [False]* size
    found_contour_right = [False]* size
    count = 0
    while y < image.shape[0]:
        # Left side
        best_cx = 0
        min_diff = float('inf')
        img = image[y:y+h_box, left_base-w_box:left_base+w_box]
        contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > AREA_CON:  # chỉ giữ contour có diện tích > 100 pixel
                found_contour_left[count]=True
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    diff = abs(cx - w_box)
                    if diff < min_diff:
                        best_cx = cx
                        min_diff = diff
        if best_cx != 0:
            left_base = left_base - w_box + best_cx
            x_left[count] = left_base
        else:
            x_left[count] = left_base
        # Right side
        best_cx = 0
        min_diff = float('inf')
        img = image[y:y+h_box, right_base-w_box:right_base+w_box]
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > AREA_CON:  # chỉ giữ contour có diện tích > 100 pixel
                found_contour_right[count]=True
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    diff = abs(cx - w_box)
                if diff < min_diff:
                    best_cx = cx
                    min_diff = diff
        if best_cx != 0: 
            right_base = right_base - w_box + best_cx
            x_right[count] = right_base 
        else:
            x_right[count] = right_base
        # cv2.rectangle(msk, (left_base - w_box, y), (left_base + w_box, y + h_box), (255, 255, 255), 2)
        # cv2.rectangle(msk, (right_base - w_box, y), (right_base + w_box, y + h_box), (255, 255, 255), 2)
        y += h_box
        count+=1
    if found_contour_left[size-1] and found_contour_right[size-1]:
        is_crossing = False
    else:
        is_crossing = True

    is_left_lane_available = not (found_contour_left[size-1] == 0 or found_contour_left[280//h_box-1] == 0)
    is_right_lane_available = not (found_contour_right[size-1] == 0 or found_contour_right[280//h_box-1] == 0)

    x_left_1 = max(x_left[size - 1], 0) 
    x_left_2 = max(x_left[280//h_box], 0)
    x_right_1 = min(x_right[size - 1], 1279)
    x_right_2 = min(x_right[280//h_box], 1279)
    x_center = (x_left_1 + x_right_1)//2
    x_center_up = (x_left_2 + x_right_2)//2
    tan = (x_center- x_center_up)/(200)
    angle = np.arctan(tan)*180/3.1415

    #angle = x_center = x_center_up = x_left_1 = x_right_1 = x_left_2 = x_right_2 = 0
    return angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2, is_crossing, is_left_lane_available, is_right_lane_available

def detect_in_cross_road_v3(image, left_base, right_base, count_is_crossing):
    msk = image.copy()
    w_box = 40
    h_box = 10
    AREA_CON = 50
    SLOPE_CON = 0.15
    size = 480//h_box
    y = 0
    x_left = [0] * size
    x_right = [0] * size 
    found_contour_left = [False]* size
    found_contour_right = [False]* size
    count = 0
    while y < image.shape[0]:
        # Left side
        best_cx = 0
        min_diff = float('inf')
        img = image[y:y+h_box, left_base-w_box:left_base+w_box]
        contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > AREA_CON:  # chỉ giữ contour có diện tích > 100 pixel
                found_contour_left[count]=True
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    diff = abs(cx - w_box)
                    if diff < min_diff:
                        best_cx = cx
                        min_diff = diff
        if best_cx != 0:
            left_base = left_base - w_box + best_cx
            x_left[count] = left_base
        else:
            x_left[count] = left_base
        # Right side
        best_cx = 0
        min_diff = float('inf')
        img = image[y:y+h_box, right_base-w_box:right_base+w_box]
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > AREA_CON:  # chỉ giữ contour có diện tích > 100 pixel
                found_contour_right[count]=True
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    diff = abs(cx - w_box)
                if diff < min_diff:
                    best_cx = cx
                    min_diff = diff
        if best_cx != 0: 
            right_base = right_base - w_box + best_cx
            x_right[count] = right_base 
        else:
            x_right[count] = right_base
        # cv2.rectangle(msk, (left_base - w_box, y), (left_base + w_box, y + h_box), (255, 255, 255), 2)
        # cv2.rectangle(msk, (right_base - w_box, y), (right_base + w_box, y + h_box), (255, 255, 255), 2)
        y += h_box
        count+=1
    
    x1,x2 = find_nearest_white_points(image, (left_base+right_base)//2, 999)
    x1_up, x2_up = find_nearest_white_points(image, (left_base+right_base)//2, 800)
    if x1 == 0 or x2 == 1279 or x1_up == 0 or x2_up == 1279:
        is_crossing = True
        count_is_crossing = 0
    elif x1 != 0 and x2 != 1279 and x1_up != 0 and x2_up != 1279:
        count_is_crossing += 1
        if count_is_crossing == 5:
            count_is_crossing =0
            is_crossing = False
        else:
            is_crossing = True

    
    is_left_lane_available = not (found_contour_left[size-1] == 0 or found_contour_left[280//h_box-1] == 0)
    is_right_lane_available = not (found_contour_right[size-1] == 0 or found_contour_right[280//h_box-1] == 0)

    slope_left = [0]* (size-1)
    slope_right = [0]* (size-1)

    for i in range(size-1):
        slope_left[i] = (x_left[i] - x_left[i+1])/h_box
        slope_right[i] = (x_right[i] - x_right[i+1])/h_box

    count_left = 1
    count_right= 1
    # slope_left_sum = slope_left[0]
    # slope_right_sum = slope_right[0]

    slope_left_sum = 0.9
    slope_right_sum = -0.9

    left_finish = False
    right_finish = False
    for i in range(size-1):
        if not left_finish:
            # if abs(slope_left[i] - slope_left[i+1]) < SLOPE_CON:
            #     count_left+=1
            #     slope_left_sum += slope_left[i+1]
            if 0.4 <abs(slope_left[i]) <=1:
                count_left+=1
                slope_left_sum += slope_left[i]
            # else:
            #     left_finish = True
        if not right_finish:
            # if abs(slope_right[i] - slope_right[i+1]) < SLOPE_CON:
            #     count_right+=1
            #     slope_right_sum += slope_right[i+1]
            if 0.4 <abs(slope_right[i]) <=1:
                count_right+=1
                slope_right_sum += slope_right[i]
            # else:
            #     right_finish = True

    slope_left_main = slope_left_sum/count_left
    slope_right_main = slope_right_sum/count_right
    x_left_1 = max(x_left[0] - slope_left_main * (480), 0) 
    x_left_2 = max(x_left[0] - slope_left_main * (280), 0)
    x_right_1 = min(x_right[0] - slope_right_main*(480), 1279)
    x_right_2 = min(x_right[0] - slope_right_main*(280), 1279)
    x_center = (x_left_1 + x_right_1)//2
    x_center_up = (x_left_2 + x_right_2)//2
    tan = (x_center- x_center_up)/(200)
    angle = np.arctan(tan)*180/3.1415        

    # cv2.line(msk, (0, 280), (1279,280), color=(0, 0, 0), thickness=4)
    # cv2.line(msk, (0, 280), (1279,280), color=(255, 255, 255), thickness=2)
    # cv2.line(msk, (int(x_left_1), size*h_box), (int(x_left[0]),0), color=(0, 0, 0), thickness=4)
    # cv2.line(msk, (int(x_left_1), size*h_box), (int(x_left[0]),0), color=(255, 255, 255), thickness=2)
    # cv2.line(msk, (int(x_right_1), size*h_box), (int(x_right[0]),0), color=(0, 0, 0), thickness=4)
    # cv2.line(msk, (int(x_right_1), size*h_box), (int(x_right[0]),0), color=(255, 255, 255), thickness=2)
    # cv2.line(msk, (int(x_center), size*h_box), (int(x_center_up),280), color=(0, 0, 0), thickness=4)
    # cv2.line(msk, (int(x_center), size*h_box), (int(x_center_up),280), color=(255, 255, 255), thickness=2)
   
    # cv2.imshow("original", msk)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    #angle = x_center = x_center_up = x_left_1 = x_right_1 = x_left_2 = x_right_2 = 0
    #print(angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2)
    return angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2, is_crossing, is_left_lane_available, is_right_lane_available, count_is_crossing


def cv_process(img, process_crossroads, is_crossing_crossroads, count_is_crossing):
    count_is_crossing_temp = 0

    if not process_crossroads:
        # print("Is following lane.")
        roi = img[520:720, :]
        pre_process_img = pre_process(roi) 
        center, center_secondary, start_index, end_index = find_center_road(pre_process_img, 300)
        angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2 = find_angle(pre_process_img, 719, 520, center, center_secondary)

        is_left_lane_available = not (x_left_1 == 0 or x_left_2 == 0)
        is_right_lane_available = not (x_right_1 == 1279 or x_right_2 == 1279)
        is_crossing = False

    else:
        if not is_crossing_crossroads:
            # print("Is not crossing and following lane.")
            roi = img[520:720, :]
            pre_process_img = pre_process(roi) 
            center, center_secondary, start_index, end_index = find_center_road(pre_process_img, 300)
            angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2 = find_angle(pre_process_img, 719, 520, center, center_secondary)

            if x_left_1 == 0 or x_right_1 == 1279 or x_left_2==0 or x_right_2==1279:
                is_crossing = True
            else: 
                is_crossing = False

            is_left_lane_available = not (x_left_1 == 0 or x_left_2 == 0)
            is_right_lane_available = not (x_right_1 == 1279 or x_right_2 == 1279)

        else:
            # print("Is crossing.")
            roi = img[240:720, :]
            pre_process_img = pre_process(roi)
            center, center_secondary, start_index, end_index = find_center_road(pre_process_img, 1000)

            #v1
            # angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2, is_crossing, is_left_lane_available, is_right_lane_available = detect_in_cross_road(pre_process_img, start_index, end_index)
            #v2
            # angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2, is_crossing, is_left_lane_available, is_right_lane_available = detect_in_cross_road_v2(pre_process_img, start_index, end_index)
            #v3
            angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2, is_crossing, is_left_lane_available, is_right_lane_available, count_is_crossing_temp = detect_in_cross_road_v3(pre_process_img, start_index, end_index, count_is_crossing)

    return angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2, is_crossing, is_left_lane_available, is_right_lane_available, count_is_crossing_temp

'''# -----------Obey the sign----------------'''
def obey_sign(flag: np.ndarray,  cmd: np.ndarray , temp: np.ndarray, cnt: np.ndarray):
    C = 5
    D = -150
    for i in range(len(temp)):
        if flag[i] == 0:                  
           cmd[i] = 0 
        else :                              # flag[i] == 1  
            if temp[i] == 1:       
                if cmd[i] == 0:
                    cnt[i] += 1
                    if cnt[i] == C:               
                        cmd[i] = 1
                elif cmd[i] == 1:
                    cnt[i] = 0
            elif temp[i] == 0:
                if cmd[i] == 1:
                    cnt[i] += -1
                    if cnt[i] == D:              
                        cmd[i] = 0
                elif cmd[i] == 0:
                    cnt[i] = 0                           
    return cnt, cmd

'''# -----------Uart obey the sign----------------'''
def uart_cmd(pre_cmd: np.ndarray, cmd: np.ndarray, uart_cmd: np.ndarray):
    for i in range(len(cmd)):
        if pre_cmd[i] == 0 and cmd[i] == 1:
            uart_cmd[i] = 1
        else:
            uart_cmd[i] = 0
    return uart_cmd

    '''# --------FLag Sign Detect----------'''
# @staticmethod
def flag_for_sign(id: np.ndarray, cnt: np.ndarray, flag: np.ndarray):
    A = 10
    B = -5
    for i in range(len(id)):
        #-----ID[i] == 1-----
        if id[i] == 1:                      
            if flag[i] == 0:                 
                cnt[i] += 1                 
                if cnt[i] == A:               
                    flag[i] = 1                
            elif flag[i] == 1:
                cnt[i] = 0
        #-----ID[i] == 0-----
        else:
            if flag[i] == 1:                
                cnt[i] += -1                  
                if cnt[i] == B:               
                    flag[i] = 0  
            elif flag[i] == 0:
                cnt[i] = 0
    return cnt, flag 

'''# --------Function to get the color of the bounding box (BGR)----------'''
# @staticmethod
def color_for_label(label):
    if label == 0:          # Bus_stop
        color = (148,0,211)
    elif label == 1:        # Children_crossing
        color = (255,215,0)
    elif label == 2:        # Green_light
        color = (0,200,0)
    elif label == 3:        # Left_turn_only
        color = (255,0,0)
    elif label == 4:        # No_Stopping
        color = (30,105,200)
    elif label == 5:        # Red_light
        color = (0,0,205)
    elif label == 6:        # Speed_limit_40
        color = (95,158,160)
    elif label == 7:        # Stop
        color = (199,21,133)
    return color
     
def cv_process_lane(frame_queue, lane_result_queue, type_queue, init_queue):
    print("Start CV Lane Processing")
    process_crossroads = False
    is_crossing_crossroads = False
    count_is_crossing_main = 0

    while True:
        
        if not init_queue.empty():
            init = init_queue.get()
            process_crossroads = init
            is_crossing_crossroads = init
            print("Get initialize value. process_crossroads: {}, is_crossing_crossroads: {}".format(
            process_crossroads, is_crossing_crossroads))

        if not type_queue.empty():
            process_crossroads = type_queue.get()

            if process_crossroads is True:
                print("Process crossroads.")
            else:
                print("Process lane.")

        if not frame_queue.empty():
            frame = frame_queue.get()
            angle_r, x_center_r, x_center_up_r, x_left_1_r, x_right_1_r, x_left_2_r, x_right_2_r, is_crossing_r, is_left_lane_available_r, is_right_lane_available_r, count_is_crossing_main = cv_process(frame, process_crossroads, is_crossing_crossroads, count_is_crossing_main)
            is_crossing_crossroads = is_crossing_r
            lane_result_queue.put((angle_r, x_center_r, x_center_up_r, x_left_1_r, x_right_1_r, x_left_2_r, x_right_2_r, is_crossing_r, is_left_lane_available_r, is_right_lane_available_r))


def cv_process_sign(frame_queue_sign, sign_result_queue):
        print("Start CV Signs Processing")

        '''--------------------SIGNS DETECTION------------------------'''
        WINDOW_NAME = 'YOLOv4Tiny'

        '''# --------Class-names----------'''
        CLASS_NAMES = [
            "Bus_Stop",  
            "Children_Crossing",
            "Green_Light",
            "Left_Turn_Only",
            "No_Stopping",
            "Red_Light",
            "Speed_Limit_40",
            "Stop"
        ]

    #     Initialize the model yolov4-tiny
        model_name = "yolov4-tiny-custom-8class"
        input_size = 416
        num_classes = 8
        conf_thresh = 0.5
        trt_file = f"yolo/{model_name}.trt"
        trt_yolo = TrtYOLO(model_name, num_classes)
        
        '''# --------Draw line---------- '''    
        PERCENT = 0.65
        P1 = int(1280 * PERCENT)



        Cnt = np.array([0, 0, 0, 0, 0, 0, 0, 0])            # Đếm lên và xuống chống nhiễu
        Flag = np.array([0, 0, 0, 0, 0, 0, 0, 0])           # 0: Không detect  |  1: Detect (đã chống nhiễu)
        Cnt_cmd = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        Cmd = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        #Pre_cmd = np.array([0, 0, 0, 0, 0, 0, 0, 0])             
        # Uart_to_tx2 = np.array([0, 0, 0, 0, 0, 0, 0, 0]) 

        print("Signs processing initialized.")
        while True:
            if not frame_queue_sign.empty():
                frame = frame_queue_sign.get()
                Id = np.array([0, 0, 0, 0, 0, 0, 0, 0])             # 0: Không detect  |  1: Detect (mỗi farme)
                Temp = np.array([0, 0, 0, 0, 0, 0, 0, 0])           # 0: Không thỏa đk  |  1: Thỏa đk (mỗi farme)  
                
                '''----------Sign Detection----------'''
                # Inference
                boxes, scores, classes = trt_yolo.detect(frame, conf_thresh)
                # Vẽ bounding box và nhãn
                for box, score, cls in zip(boxes, scores, classes):
                    x1, y1, x2, y2 = box
                    Id[int(cls)] = 1                            # 1: Detect (mỗi farme)
                    if x1 > P1 and (y2-y1)>60 and score > 0.6: # Nếu bounding box nằm trong vùng P1 và chiều cao lớn hơn 100 pixel
                        Temp[int(cls)] = 1
                        if CLASS_NAMES[int(cls)] == "Stop" and (y2-y1) < 75:
                            Temp[int(cls)] = 0
                        
                    #label = f"{CLASS_NAMES[int(cls)]} {score:.2f}"
                    #colorBox = tx2.color_for_label(int(cls))
                    #cv2.rectangle(frame, (x1, y1), (x2, y2), colorBox, 2)
                    #cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colorBox, 2)
                    # if x1 > P1 and (x2-x1)>100 and int(cls) == 7:
                    #     cv2.putText(frame, "STOP", (30, 80),
                    #     cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                Cnt, Flag = flag_for_sign(Id, Cnt, Flag)         # Chống nhiễu
                Cnt_cmd, Cmd = obey_sign(Flag, Cmd, Temp, Cnt_cmd)      # Chống nhiễu lệnh
                #Uart_to_tx2 = uart_cmd(Pre_cmd, Cmd, Uart_to_tx2)                # Gửi lệnh đến tx2
                #cv2.imshow(WINDOW_NAME, frame)
                sign_result_queue.put((Flag, Cmd))


def video_writer_process(frame_queue, video_path, frame_size, fps=20):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(video_path, fourcc, fps, frame_size)

    print(f"Start recording video.")
    
    while True:
        frame = frame_queue.get()
        if frame is None:
            print("Stop recording video.")
            break
        out.write(frame)

    out.release()


if __name__ == "__main__":

    multiprocessing.set_start_method('spawn')

    frame_queue_sign = multiprocessing.Queue(maxsize=5)
    sign_result_queue = multiprocessing.Queue(maxsize=5)
    sign_p = multiprocessing.Process(target=cv_process_sign, args=(frame_queue_sign, sign_result_queue))
    sign_p.start()

    frame_queue = multiprocessing.Queue(maxsize=5)
    lane_result_queue = multiprocessing.Queue(maxsize=5)
    type_queue = multiprocessing.Queue(maxsize=5)
    init_queue = multiprocessing.Queue(maxsize=5)
    lane_p = multiprocessing.Process(target=cv_process_lane, args=(frame_queue, lane_result_queue, type_queue, init_queue))
    lane_p.start()


    filename = "output_video.mp4"
    video_path = os.path.join(os.getcwd(), filename)
    video_queue = multiprocessing.Queue(maxsize=20)
    video_p = multiprocessing.Process(target=video_writer_process, args=(video_queue, video_path, (1280, 720)))
    video_p.start()

    record_video = True
    tx2 = VFR()

    main_vision_start = False

    time.sleep(1)

    try:
        while True:
            cv_process_crossroads = False
            if tx2.vision_start:
                main_vision_start = True

                if not init_queue.full():
                    init_cv = False
                    init_queue.put(init_cv)
                    print("Inint process cv.")

                print("Start getting frame")
                cap = cv2.VideoCapture(tx2.GST_PIPELINE, cv2.CAP_GSTREAMER)

                if not cap.isOpened():
                    print("Can not receive video from GStreamer!")
                    tx2.vision_start = False
                    continue

                while tx2.vision_start:
                    tx2.cam_ret, tx2.cam_frame = cap.read()

                    if not tx2.cam_ret:
                        print("Receive frame failed.!")
                        tx2.vision_start            = False
                        tx2.vision_cv_sign_ready    = False
                        tx2.vision_cv_sign_process  = False
                        continue


                    # if record_video is True:
                    if not video_queue.full():
                        video_queue.put(tx2.cam_frame.copy())
                    
                    # print("Process next frame.")
                    if(cv_process_crossroads != tx2.cv_process_crossroads):
                        cv_process_crossroads = tx2.cv_process_crossroads
                        if not type_queue.full():
                            type_queue.put(cv_process_crossroads)
                            print("Command cv_process_crossroads value: {}".format(cv_process_crossroads))

                    if not frame_queue.full():
                        frame_queue.put(tx2.cam_frame.copy())
                        # print("Put frame for lane processing.")

                    if not frame_queue_sign.full():
                        frame_queue_sign.put(tx2.cam_frame.copy())
                        # print("Put frame for sign processing.")

                    if not lane_result_queue.empty():
                        lane_result = lane_result_queue.get()
                        angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2, is_crossing_crossroads, is_left_lane_available, is_right_lane_available = lane_result
                        tx2.update_cv_process(angle, x_center, x_center_up, x_left_1, x_right_1, x_left_2, x_right_2, is_crossing_crossroads, is_left_lane_available, is_right_lane_available)
                    
                    if not sign_result_queue.empty():
                        sign_result = sign_result_queue.get()
                        sign_detected, sign_cmds = sign_result
                        tx2.update_sign_process(sign_detected, sign_cmds)
                    
         else:
                if (main_vision_start is True):
                    main_vision_start = False
                    video_queue.put(None)
                    # if record_video is True:
                        # video_p.join()

                time.sleep(0.5)
    except KeyboardInterrupt:
        print("Stopping tx2 handler...")
        tx2.stop_receiving_and_processing_packet()




