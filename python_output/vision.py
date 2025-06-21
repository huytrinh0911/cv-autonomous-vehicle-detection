import cv2
import numpy as np
import matplotlib.pyplot as plt


class VFR_CV:
    l_h = 0
    l_s = 0
    l_v = 0
    u_h = 255
    u_s = 255
    #Day
    # u_v = 108
    #Night
    u_v = 90


    def __init__(self):
        super().__init__()
        self.lower = np.array([self.l_h, self.l_s, self.l_v])
        self.upper = np.array([self.u_h, self.u_s, self.u_v])

    def canny(self, image):
        roi = image[520:720, :]
        hsv_transformed_frame = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_transformed_frame, self.lower, self.upper)
        blur = cv2.GaussianBlur(mask, (5, 5), 0)
        return blur

    def find_nearest_white_points(self, edge_image, x_ref, y_ref):
        y = y_ref -520  
        row_pixels = edge_image[y, :]  # Lấy toàn bộ dòng y cần detect
        
        # Tìm các vị trí x có giá trị 255 (trắng)
        white_x_positions = np.where(row_pixels == 255)[0]

        if len(white_x_positions) == 0:
            print("Không tìm thấy điểm trắng nào trên hàng này.")
            return 0, 1279

        # Tách điểm bên trái và bên phải x_ref
        left_points = [x for x in white_x_positions if x < x_ref]
        right_points = [x for x in white_x_positions if x > x_ref]

        # Lấy điểm gần nhất bên trái và bên phải
        x_left = max(left_points) if left_points else 0
        x_right= min(right_points) if right_points else 1279

        return x_left, x_right

    def find_angle(self, edge_image,y_down, y_up, center, center_secondary):
        #tìm 4 điểm thuộc lane
        x_left_1, x_right_1 = self.find_nearest_white_points(edge_image, center, y_down)
        x_left_2, x_right_2 = self.find_nearest_white_points(edge_image, center, y_up)
        if ((x_left_1 == 0)|(x_left_2==0))&(x_right_1<x_right_2):
            x_left_1, x_right_1 = self.find_nearest_white_points(edge_image, center_secondary, y_down)
            x_left_2, x_right_2 = self.find_nearest_white_points(edge_image, center_secondary, y_up)
        elif ((x_right_1 == 1279)|(x_right_2 == 1279))&(x_left_1 > x_left_2):
            x_left_1, x_right_1 = self.find_nearest_white_points(edge_image, center_secondary, y_down)
            x_left_2, x_right_2 = self.find_nearest_white_points(edge_image, center_secondary, y_up)
        
        #tìm điểm trung tâm
        x_center = (x_left_1 + x_right_1)/2
        x_up = (x_left_2 + x_right_2)/2

        #tính toán góc
        tan = (x_center- x_up)/(y_down-y_up)
        angle = np.arctan(tan)*180/3.1415
        return angle, x_center, x_left_1, x_right_1, x_left_2, x_right_2

    def find_center_road(self, image):
        histogram = np.sum(image, axis=0)  

        plt.figure(figsize=(10, 6))  # Kích thước của biểu đồ
        plt.bar(np.arange(histogram.shape[0]), histogram, color='blue', alpha=0.7)  # Vẽ histogram dưới dạng cột
        plt.title('Histogram của ảnh (10x20)', fontsize=14)
        plt.xlabel('Vị trí cột', fontsize=12)
        plt.ylabel('Tổng pixel theo cột', fontsize=12)
        plt.grid(True)  # Hiển thị lưới
        plt.show()

        max_length = 0  # Đoạn dài nhất
        current_length = 0  # Chiều dài của đoạn hiện tại
        start_index = 0  # Vị trí bắt đầu của đoạn dài nhất
        end_index = 0  # Vị trí kết thúc của đoạn dài nhất
        start_index_secondary = 0
        end_index_secondary = 0
        max_length_secondary = 0

        for i in range(0, 1279):
            if (histogram[i] < 3000):
                if current_length == 0:
                    start_index_tmp = i  # Lưu lại vị trí bắt đầu của đoạn mới
                current_length += 1
            else:
                if current_length > max_length:
                    max_length_secondary = max_length
                    start_index_secondary = start_index
                    end_index_secondary = end_index  # Đoạn kết thúc tại chỉ số trước cột này
                    max_length = current_length
                    start_index = start_index_tmp
                    end_index = i - 1  # Đoạn kết thúc tại chỉ số trước cột này
                elif current_length > max_length_secondary:
                    max_length_secondary = current_length
                    start_index_secondary = start_index_tmp
                    end_index_secondary = i - 1  # Đoạn kết thúc tại chỉ số trước cột này
                current_length = 0

        # Kiểm tra nếu đoạn cuối cùng là dài nhất
        if current_length > max_length:
            max_length = current_length
            start_index = start_index_tmp
            end_index = len(histogram) - 1
        elif current_length > max_length_secondary:
            max_length_secondary = current_length
            start_index_secondary = start_index_tmp
            end_index_secondary = i  # Đoạn kết thúc tại chỉ số trước cột này
        center = (start_index + end_index)//2
        center_secondary = (start_index_secondary+end_index_secondary)//2
        return center, center_secondary

    
    def cv_process(self, img):
        canny_image = self.canny(img) 
        center, center_secondary = self.find_center_road(canny_image)
        return self.find_angle(canny_image, 719, 520, center, center_secondary)
    

if __name__ == "__main__":
    vision = VFR_CV()

    image = cv2.imread("r_frame_004.jpg")
    angle, x_center, x_left_1, x_right_1, x_left_2, x_right_2 = vision.cv_process(image)
    print("Angle: {}\nCenter: {}\nLeft1: {}\nRight1: {}\nLeft2: {}\nRight2: {}".format(
    angle, x_center, x_left_1, x_right_1, x_left_2, x_right_2))
