import cv2
import numpy as np
import matplotlib.pyplot as plt


l_h = 0
l_s = 0
l_v = 0
u_h = 255
u_s = 255
#Day
# u_v = 108
#Night
u_v = 90

lower = np.array([l_h, l_s, l_v])
upper = np.array([u_h, u_s, u_v])

def pre_process(image):
    roi = image[520:720, :]
    gray_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur_median = cv2.medianBlur(gray_image, 7)
    alpha = 2.25  # contrast > 1 → tăng contrast mạnh
    beta = -50     # không thay đổi độ sáng
    high_contrast = cv2.convertScaleAbs(blur_median, alpha=alpha, beta=beta)
    inverted = 255 - high_contrast
    thresh_val, binary = cv2.threshold(inverted, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return binary


def find_nearest_white_points(edge_image, x_ref, y_ref):
    y = y_ref-520 #Do đã crop ảnh nên trong ảnh mới Y sẽ có giá trị từ 0 - 199
    row_pixels = edge_image[y, :]  # Lấy toàn bộ dòng y cần detect
    
    # Tìm các vị trí x có giá trị 255 (trắng)
    white_x_positions = np.where(row_pixels == 255)[0]

    if len(white_x_positions) == 0:
        print("Không tìm thấy điểm trắng nào trên hàng này.")
        return 0, 1279

    # Tách điểm bên trái và bên phải x_ref
    left_points = [x for x in white_x_positions if x < x_ref] #tìm tất cả điểm trắng bên trái
    right_points = [x for x in white_x_positions if x > x_ref] #tìm tất cả điểm trắng bên phải

    # Lấy điểm gần nhất bên trái và bên phải
    x_left = max(left_points) if left_points else 0
    x_right= min(right_points) if right_points else 1279

    return x_left, x_right

#Tính toán góc và tính chính
def find_angle(edge_image,y_down, y_up, center, center_secondary):
    #tìm 4 điểm thuộc lane cần detect
    # x_left_1, x_left_2 là 2 điểm thuộc lane trái
    # x_right_1, x_right_2 là 2 điểm thuộc lane phải
    x_left_1, x_right_1 = find_nearest_white_points(edge_image, center, y_down)
    x_left_2, x_right_2 = find_nearest_white_points(edge_image, center, y_up)


    if ((x_left_1 == 0)|(x_left_2==0))&(x_right_1<x_right_2): #Nếu không nhìn thấy Lane bên trái và x_right_1 nhỏ hơn x_right_2 => cần detect lại
        x_left_1, x_right_1 = find_nearest_white_points(edge_image, center_secondary, y_down) 
        x_left_2, x_right_2 = find_nearest_white_points(edge_image, center_secondary, y_up)
    elif ((x_right_1 == 1279)|(x_right_2 == 1279))&(x_left_1 > x_left_2): #Nếu không nhìn thấy Lane bên phải và x_left_1 lớn hơn x_left_2 => cần detect lại
        x_left_1, x_right_1 = find_nearest_white_points(edge_image, center_secondary, y_down)
        x_left_2, x_right_2 = find_nearest_white_points(edge_image, center_secondary, y_up)
    
    #tìm điểm trung tâm
    x_center = (x_left_1 + x_right_1)/2 
    x_up = (x_left_2 + x_right_2)/2

    #tính toán góc
    tan = (x_center- x_up)/(y_down-y_up)
    angle = np.arctan(tan)*180/3.1415
    return angle, x_center, x_left_1, x_right_1, x_left_2, x_right_2

#Tìm 2 điểm có thể là điểm trung tâm
def find_center_road(image):
    histogram = np.sum(image, axis=0)  

    plt.figure(figsize=(10, 6))  # Kích thước của biểu đồ
    plt.bar(np.arange(histogram.shape[0]), histogram, color='blue', alpha=0.7)  # Vẽ histogram dưới dạng cột
    plt.title('Histogram của ảnh (10x20)', fontsize=14)
    plt.xlabel('Vị trí cột', fontsize=12)
    plt.ylabel('Tổng pixel theo cột', fontsize=12)
    plt.grid(True)  # Hiển thị lưới
    plt.show()


    current_length = 0  # Chiều dài của đoạn hiện tại
    count = 0

    max_length = 0  # Đoạn dài nhất
    start_index = 0  # Vị trí bắt đầu của đoạn dài nhất
    end_index = 0  # Vị trí kết thúc của đoạn dài nhất
    
    max_length_secondary = 0 # Đoạn dài thứ hai
    start_index_secondary = 0 # Vị trí bắt đầu của đoạn dài thứ hai
    end_index_secondary = 0 # Vị trí kết thúc của đoạn dài thứ hai

    for i in range(0, 1279):
        if (histogram[i] < 5000):
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
    return center, center_secondary


def cv_process(img):
    pre_process_img = pre_process(img) 
    center, center_secondary = find_center_road(pre_process_img)
    return find_angle(pre_process_img, 719, 520, center, center_secondary)


if __name__ == "__main__":

    image = cv2.imread("r_frame_007.jpg")
    angle, x_center, x_left_1, x_right_1, x_left_2, x_right_2 = cv_process(image)
    print("Angle: {}\nCenter: {}\nLeft1: {}\nRight1: {}\nLeft2: {}\nRight2: {}".format(
    angle, x_center, x_left_1, x_right_1, x_left_2, x_right_2))
