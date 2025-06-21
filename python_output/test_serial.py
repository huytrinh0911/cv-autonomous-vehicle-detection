import serial
import serial.tools.list_ports
import time

# Hàm để tự động phát hiện cổng COM
def detect_com_port():
    ports = list(serial.tools.list_ports.comports())
    if len(ports) == 0:
        print("Không tìm thấy cổng COM nào.")
        return None
    for port in ports:
        print(f"Phát hiện cổng COM: {port.device}")
    return ports[0].device  # Chọn cổng COM đầu tiên

    # ports = list(serial.tools.list_ports.comports())
    # if len(ports) == 0:
    #     print("No port found.")
    #     return None
    # name = "USB-SERIAL CH340"
    # for port in ports:
    #     if name in port.description:
    #         print(f"COM detected: {port.device}")
    #         print(f"COM name: {port.description}")
    #         return port.device
    # # return ports[0].device  # Chọn cổng COM đầu tiên
    # return None  # Chọn cổng COM đầu tiên

# Cấu hình truyền UART
def uart_communication():
    com_port = detect_com_port()
    if com_port is None:
        return

    try:
        # Mở kết nối UART với cổng COM phát hiện được
        ser = serial.Serial(com_port, baudrate=115200, timeout=1)

        # Gửi và nhận dữ liệu qua UART
        while True:
            data_to_send = input("Nhập dữ liệu để gửi (hoặc 'exit' để thoát): ")
            if data_to_send.lower() == 'exit':
                break

            # Gửi dữ liệu
            ser.write(data_to_send.encode())

            # Nhận dữ liệu phản hồi
            if ser.in_waiting > 0:
                received_data = ser.read(ser.in_waiting).decode()
                print(f"Dữ liệu nhận được: {received_data}")

        # Đóng kết nối UART khi kết thúc
        ser.close()
    except Exception as e:
        print(f"Lỗi: {e}")

# Chạy chương trình
if __name__ == '__main__':
    uart_communication()
