import serial
import time
import protocol_pb2 as proto
import serial.tools.list_ports
from protocol_parser import VFR

v = VFR(debug=True)

print('\n------------------- Testing Start Robot cmd ---------------------')
# v.robot_cmd_start()
if (v.packet_robot_cmd_start()):
    print('Send CMD Start successfully')
    while True:
        data_to_send = input("Nhập dữ liệu để gửi (hoặc 'exit' để thoát): ")
        if data_to_send.lower() == 'exit':
            break
else:
    print('Send CMD Start failed')



