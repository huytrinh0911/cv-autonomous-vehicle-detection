import serial
import time
import protocol_pb2 as proto
import serial.tools.list_ports
from protocol_parser import VFR

v = VFR(src = proto.DEVICE_ADDR_APP, dst = proto.DEVICE_ADDR_CONTROLLER, debug = True, gui_process = False)

print('\n------------------- Testing network ---------------------')
def main():
    while True:
        user_input = input("Nhập 'S'/'D' để gửi gói network_get đến Controller/TX2, , 'Q' để thoát chương trình: ").strip().upper()

        if user_input == "S":
            packet = proto.network_packet_t()
            packet.network_status_get.dummy = 1
            v.send_packet(proto.DEVICE_ADDR_CONTROLLER, packet)
        elif user_input == "D":
            packet = proto.network_packet_t()
            packet.network_status_get.dummy = 1
            v.send_packet(proto.DEVICE_ADDR_TX2, packet)
        elif user_input == "Q":
            print("Đang thoát chương trình...")
            break
        else:
            print("Lựa chọn không hợp lệ. Vui lòng nhập 'S', 'D' hoặc 'Q'.")

if __name__ == "__main__":
    main()