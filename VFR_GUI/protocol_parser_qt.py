from PySide6.QtCore import QObject, Signal
import serial
import serial.tools.list_ports
import time
import threading
import sys
sys.path.append('../../Intergrated_protocol/python_output')
sys.path.append('../../Intergrated_protocol/nanopb/generator/proto')
import protocol_pb2 as proto
import numpy as np

class VFR(QObject):
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

    SERIAL_TIMEOUT_PER_FRAME    = 0.010

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

    # Declare QT Signal
    signal_packet_process      = Signal(object) 

    def __init__(
        self,
        com_port        = None,
        com_name        = SERIAL_USB_NAME,
        com_rate        = 115200,
        src             = DEVICE_ADDR_APP,
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
        self.cnt = 0

        # Declare UART Process thread
        self.thread = threading.Thread(target=self.read_packet, name="PacketReceiver", daemon=True) 

        if not self.gui_process:
            if com_port is None:
                self.com_port = self.detect_com_port()
                self.serial = serial.Serial(self.com_port, baudrate=self.com_rate, timeout=1)
                self.start_receiving_and_processing_packet()


    def checksum(self, buff):
        sum = 0
        for byte in buff:
            sum += byte
        return (sum & 0xFF)  # Truncate to 8 bits
    

    def detect_com_port(self):
        ports = list(serial.tools.list_ports.comports())
        if not self.gui_process:
            if len(ports) == 0:
                print("No port found.")
                return None
            if not self.debug:
                for port in ports:
                        if self.com_name in port.description:
                            self.connected = True
                            return port.device
            else:
                return ports[0].device
            return None
        else:
            return ports
        

    def connect_com_port(self):
        self.serial = serial.Serial(self.com_port, baudrate=self.com_rate, timeout=1)
        self.start_receiving_and_processing_packet()

    def disconnect_com_port(self):
        self.serial.close()
        self.stop_receiving_and_processing_packet()
    

    def serial_send(self, buf):
        try:
            time.sleep(0.020)
            self.serial.write(buf)
            return True
        except Exception as e:
            print(f"Error: {e}")
            return False


    def send_packet(self, dst, packet):
        if self.connected:
            packet.hdr.addr.src = self.src
            packet.hdr.addr.dst = dst
            packet.hdr.epoch_time = np.uint64((time.time() + (7 * 3600)) * 1000)

            encode_packet = packet.SerializeToString()

            serial_frame = bytearray()
            serial_frame += self.SERIAL_FRAME_SOF.to_bytes(4, "little")
            serial_frame += len(encode_packet).to_bytes(4, "little")
            serial_frame += encode_packet
            checksum = self.checksum(serial_frame)
            serial_frame += checksum.to_bytes(1, "little")
            # print(f"------------------------------------------------------------")
            # print(f"Encoded packet length: {len(encode_packet)}")
            # print(f"Total frame length (include checksum): {len(serial_frame)}")
            # print(f"Checksum: {checksum}")

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

                        if (self.serial_state == self.SERIAL_STATE_FIND_LSB_SOF):
                            if (self.uart_buffer[self.IDX_SOF_LSB] == self.SERIAL_FRAME_SOF_LSB):
                                self.serial_state = self.SERIAL_STATE_FIND_MID_LOW_SOF
                            else:
                                self.uart_buffer.clear()
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
                                # print(f"uart_buffer len: {len(self.uart_buffer)}")
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
        
        self.signal_packet_process.emit(packet)


    def start_receiving_and_processing_packet(self):
        self.connected = True
        if not self.thread_start:
            self.thread_start = True
            self.thread.start()

    def stop_receiving_and_processing_packet(self):
        self.connected = False
        self.uart_buffer.clear()
        self.encoded_packet.clear()
        self.frame_without_checksum.clear()
        self.start_time = None
        self.encoded_packet_lenth = 0
        self.frame_checksum = 0x00
        self.serial_state = self.SERIAL_STATE_FIND_LSB_SOF


    
    def packet_robot_cmd_start(self):
        packet = proto.network_packet_t()
        packet.robot_cmd_start.dummy = 0
        
        if self.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet):
            return True
        return False
    

    def packet_network_status_get(self, dst):
        packet = proto.network_packet_t()
        packet.network_status_get.dummy = 0
        
        if self.send_packet(dst, packet):
            return True
        return False
    








