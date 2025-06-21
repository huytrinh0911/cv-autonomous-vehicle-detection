import cv2
import numpy as np
import time

# === Hàm tiền xử lý dùng CPU ===
def pre_process_cpu(image):
    roi = image[520:720, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur_median = cv2.medianBlur(gray, 25)
    blur_bilateral = cv2.bilateralFilter(blur_median, d=9, sigmaColor=75, sigmaSpace=100)
    blur_gaussian = cv2.GaussianBlur(blur_bilateral, (5, 5), 0)
    contrast = cv2.convertScaleAbs(blur_gaussian, alpha=2.0, beta=20)
    inverted = 255 - contrast
    blur_final = cv2.medianBlur(inverted, 25)
    _, binary = cv2.threshold(blur_final, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return binary

# === Hàm tiền xử lý dùng GPU (CUDA) ===
def pre_process_cuda(image):
    roi = image[520:720, :]
    gpu_image = cv2.cuda_GpuMat()
    gpu_image.upload(roi)

    # Chuyển ảnh sang grayscale
    gpu_gray = cv2.cuda.cvtColor(gpu_image, cv2.COLOR_BGR2GRAY)

    # Median Blur: dùng CPU vì OpenCV không hỗ trợ median trên CUDA
    gray = gpu_gray.download()
    blur_median = cv2.medianBlur(gray, 25)
    gpu_blur_median = cv2.cuda_GpuMat()
    gpu_blur_median.upload(blur_median)

    # Bilateral filter
    gpu_bilateral = cv2.cuda.bilateralFilter(gpu_blur_median, d=9, sigmaColor=75, sigmaSpace=100)

    # Gaussian Blur
    gpu_gaussian = cv2.cuda.createGaussianFilter(cv2.CV_8UC1, cv2.CV_8UC1, (5,5), 0)
    gpu_blur = gpu_gaussian.apply(gpu_bilateral)

    # Tăng contrast
    blur_cpu = gpu_blur.download()
    contrast = cv2.convertScaleAbs(blur_cpu, alpha=2.0, beta=20)

    # Invert và median blur lần nữa
    inverted = 255 - contrast
    blur_final = cv2.medianBlur(inverted, 25)

    # Threshold
    _, binary = cv2.threshold(blur_final, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return binary

# === Load ảnh đầu vào ===
image = cv2.imread("your_image.jpg")  # Thay bằng đường dẫn ảnh thật của bạn

# === Xử lý bằng GPU ===
start_gpu = time.time()
binary_cuda = pre_process_cuda(image)
end_gpu = time.time()
print("GPU (CUDA) Processing Time: {:.4f} seconds".format(end_gpu - start_gpu))

# === Xử lý bằng CPU ===
start_cpu = time.time()
binary_cpu = pre_process_cpu(image)
end_cpu = time.time()
print("CPU Processing Time:       {:.4f} seconds".format(end_cpu - start_cpu))

