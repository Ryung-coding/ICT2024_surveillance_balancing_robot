import cv2

# 여러 인덱스를 시도하여 연결 가능한 카메라를 찾는 코드
def find_camera_index():
    for i in range(10):  # 인덱스 0부터 9까지 시도
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Camera found at index {i}")
            cap.release()
        else:
            print(f"No camera found at index {i}")

if __name__ == "__main__":
    find_camera_index()
