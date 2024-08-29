import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import cv2
import socketio
import base64
import threading
import numpy as np
import pytesseract
import time
import re
from datetime import datetime
import os

# data 초기값 설정
# 서울과학기술대학교 초기 좌표 설정
data = {'Long': 127.0772, 'Lat': 37.6315, 'Alt': 50,
        'r': -1.0, 'p': -1.0, 'y': -1.0,
        'PL_NUM': '0000', 'PL_STATE': -1,
        'u1': -1.0, 'u2': -1.0, 'u3': -1.0,
        'V1': 0.0, 'V2': 0.0,
        'v1': 0.0, 'v2': 0.0, 'v3': 0.0,
        'SIV': 0, 'FIX': 0}

class CameraClient:
    def __init__(self, server_url, room_id):
        self.server_url = server_url
        self.room_id = room_id
        self.cap = cv2.VideoCapture(0)  # 0 is default camera
        self.sio = socketio.Client()
        self.isConnected = False

        # Socket.IO 이벤트 핸들러 등록
        self.sio.on('connect', self.connect)
        self.sio.on('roomJoined', self.on_room_joined)
        self.sio.on('disconnect', self.disconnect)
    
    def begin(self):
        EventThread = threading.Thread(target=self.wait_for_events)
        EventThread.start()
        self.sio.connect(self.server_url)

    def wait_for_events(self):
        self.sio.wait()

    def connect(self):
        self.sio.emit('joinOrCreateRoom', [self.room_id, 0])

    def on_room_joined(self, room):
        if str(room) == self.room_id:
            self.isConnected = True

    def disconnect(self):
        self.isConnected = False

    def start_sending_SensorData(self, data):
        message = {
            'room': self.room_id,
            'datas': {
                'type': 3,
                'datas': list(data.values())
            }
        }
        self.sio.emit('message', message)
            
        threading.Timer(0.25, self.start_sending_SensorData, args=(data,)).start()

    def start_sending_ImageData(self):
        _, frame = self.cap.read()
        
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        image_base64 = base64.b64encode(buffer).decode('utf-8')

        data = {
            'room': self.room_id,
            'datas': {
                'type': 2,
                'datas': [image_base64, '']
            }
        }

        self.sio.emit('message', data)
            
        threading.Timer(0.1, self.start_sending_ImageData).start()

class ImageProcessor:
    def __init__(self):
        self.isProcessing = False
        self.detect_list = []

    # 이미지 전처리
    def preprocess_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_blurred = cv2.GaussianBlur(gray, ksize=(5, 5), sigmaX=0)
        img_blur_thresh = cv2.adaptiveThreshold(
            img_blurred,
            maxValue=255.0,
            adaptiveMethod=cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            thresholdType=cv2.THRESH_BINARY_INV,
            blockSize=15,
            C=5
        )
        return img_blur_thresh

    # 컨투어 사각형 만들기
    def find_contours(self, img_blur_thresh):
        contours, _ = cv2.findContours(
            img_blur_thresh,
            mode=cv2.RETR_LIST,
            method=cv2.CHAIN_APPROX_SIMPLE
        )
        return contours

    def filter_contours(self, contours, img):
        contours_dict = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            contours_dict.append({
                'contour': contour,
                'x': x,
                'y': y,
                'w': w,
                'h': h,
                'cx': x + (w / 2),
                'cy': y + (h / 2)
            })
        return contours_dict

    # 번호판 찾기 필터링
    def find_possible_contours(self, contours_dict):
        MIN_AREA = 80
        MIN_WIDTH, MIN_HEIGHT = 2, 8
        MIN_RATIO, MAX_RATIO = 0.25, 1.0
        possible_contours = []

        cnt = 0
        for d in contours_dict:
            area = d['w'] * d['h']
            ratio = d['w'] / d['h']

            if area > MIN_AREA and d['w'] > MIN_WIDTH and d['h'] > MIN_HEIGHT and MIN_RATIO < ratio < MAX_RATIO:
                d['idx'] = cnt
                cnt += 1
                possible_contours.append(d)

        return possible_contours

    def find_chars(self, contour_list):
        MAX_DIAG_MULTIPLYER = 5
        MAX_ANGLE_DIFF = 12.0
        MAX_AREA_DIFF = 0.5
        MAX_WIDTH_DIFF = 0.8
        MAX_HEIGHT_DIFF = 0.2
        MIN_N_MATCHED = 3

        matched_result_idx = []

        for d1 in contour_list:
            matched_contours_idx = []
            for d2 in contour_list:
                if d1['idx'] == d2['idx']:
                    continue

                dx = abs(d1['cx'] - d2['cx'])
                dy = abs(d1['cy'] - d2['cy'])

                diagonal_length1 = np.sqrt(d1['w'] ** 2 + d1['h'] ** 2)

                distance = np.linalg.norm(np.array([d1['cx'], d1['cy']]) - np.array([d2['cx'], d2['cy']]))
                if dx == 0:
                    angle_diff = 90
                else:
                    angle_diff = np.degrees(np.arctan(dy / dx))
                area_diff = abs(d1['w'] * d1['h'] - d2['w'] * d2['h']) / (d1['w'] * d1['h'])
                width_diff = abs(d1['w'] - d2['w']) / d1['w']
                height_diff = abs(d1['h'] - d2['h']) / d1['h']

                if distance < diagonal_length1 * MAX_DIAG_MULTIPLYER and angle_diff < MAX_ANGLE_DIFF and area_diff < MAX_AREA_DIFF and width_diff < MAX_WIDTH_DIFF and height_diff < MAX_HEIGHT_DIFF:
                    matched_contours_idx.append(d2['idx'])

            matched_contours_idx.append(d1['idx'])

            if len(matched_contours_idx) < MIN_N_MATCHED:
                continue

            matched_result_idx.append(matched_contours_idx)

        return matched_result_idx

    # 번호판 문자 인식
    def extract_plate_text(self, plate_imgs):
        MIN_AREA = 80
        MIN_WIDTH, MIN_HEIGHT = 2, 8
        MIN_RATIO, MAX_RATIO = 0.25, 1.0

        plate_chars = []
        
        for i, plate_img in enumerate(plate_imgs):
            plate_img = cv2.resize(plate_img, dsize=(0, 0), fx=1.6, fy=1.6)
            _, plate_img = cv2.threshold(plate_img, thresh=0.0, maxval=255.0, type=cv2.THRESH_BINARY | cv2.THRESH_OTSU)

            contours, _ = cv2.findContours(plate_img, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

            plate_min_x, plate_min_y = plate_img.shape[1], plate_img.shape[0]
            plate_max_x, plate_max_y = 0, 0

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)

                area = w * h
                ratio = w / h

                if (area > MIN_AREA and w > MIN_WIDTH and h > MIN_HEIGHT and 
                    MIN_RATIO < ratio < MAX_RATIO):
                    plate_min_x = min(plate_min_x, x)
                    plate_min_y = min(plate_min_y, y)
                    plate_max_x = max(plate_max_x, x + w)
                    plate_max_y = max(plate_max_y, y + h)
            
            

            img_result = plate_img[plate_min_y:plate_max_y, plate_min_x:plate_max_x]

            if img_result is not None and img_result.size != 0:
                img_result = cv2.GaussianBlur(img_result, ksize=(3, 3), sigmaX=0)
                _, img_result = cv2.threshold(img_result, thresh=0.0, maxval=255.0, type=cv2.THRESH_BINARY | cv2.THRESH_OTSU)
                img_result = cv2.copyMakeBorder(img_result, top=10, bottom=10, left=10, right=10, borderType=cv2.BORDER_CONSTANT, value=(0, 0, 0))
                chars = pytesseract.image_to_string(img_result, lang='kor', config='--psm 7 --oem 3')
                result_chars = ''.join(c for c in chars if ord('가') <= ord(c) <= ord('호') or c.isdigit())

                if re.match(r"^\d{2,3}[가-호]\d{4}$", result_chars):
                    plate_chars.append(result_chars)

        return plate_chars

    def process_image(self, camera_client):

        ret, frame = camera_client.cap.read()
        if not ret:
            return

        img_blur_thresh = self.preprocess_image(frame)
        contours = self.find_contours(img_blur_thresh)
        contours_dict = self.filter_contours(contours, frame)
        possible_contours = self.find_possible_contours(contours_dict)
        result_idx = self.find_chars(possible_contours)

        matched_result = []
        for idx_list in result_idx:
            matched_result.append(np.take(possible_contours, idx_list))

        PLATE_WIDTH_PADDING = 1.3
        PLATE_HEIGHT_PADDING = 1.5
        MIN_PLATE_RATIO = 3
        MAX_PLATE_RATIO = 10

        plate_imgs = []
        plate_infos = []

        for i, matched_chars in enumerate(matched_result):
            sorted_chars = sorted(matched_chars, key=lambda x: x['cx'])

            plate_cx = (sorted_chars[0]['cx'] + sorted_chars[-1]['cx']) / 2
            plate_cy = (sorted_chars[0]['cy'] + sorted_chars[-1]['cy']) / 2
            plate_width = (sorted_chars[-1]['x'] + sorted_chars[-1]['w'] - sorted_chars[0]['x']) * PLATE_WIDTH_PADDING
            plate_height = int(np.mean([d['h'] for d in sorted_chars]) * PLATE_HEIGHT_PADDING)

            triangle_height = sorted_chars[-1]['cy'] - sorted_chars[0]['cy']
            triangle_hypotenus = np.linalg.norm(
                np.array([sorted_chars[0]['cx'], sorted_chars[0]['cy']]) - np.array([sorted_chars[-1]['cx'], sorted_chars[-1]['cy']])
            )

            angle = np.degrees(np.arcsin(triangle_height / triangle_hypotenus))

            rotation_matrix = cv2.getRotationMatrix2D(center=(plate_cx, plate_cy), angle=angle, scale=1.0)
            img_rotated = cv2.warpAffine(img_blur_thresh, M=rotation_matrix, dsize=(frame.shape[1], frame.shape[0]))

            # 번호판 영역 crop
            img_cropped = cv2.getRectSubPix(
                img_rotated,
                patchSize=(int(plate_width), int(plate_height)),
                center=(int(plate_cx), int(plate_cy))
            )

            # 번호판 비율 범위 넘어가면 해당 이미지 무시
            if MIN_PLATE_RATIO <= img_cropped.shape[1] / img_cropped.shape[0] <= MAX_PLATE_RATIO:
                plate_imgs.append(img_cropped)
                plate_infos.append({
                    'x': int(plate_cx - plate_width / 2),
                    'y': int(plate_cy - plate_height / 2),
                    'w': int(plate_width),
                    'h': int(plate_height)
                })

        plate_chars = self.extract_plate_text(plate_imgs)

        # 번호판 영역에 라운딩박스, 텍스트 표시(뒤 4자리만)
        for info, chars in zip(plate_infos, plate_chars):
            if chars not in self.detect_list:
                self.detect_list.append(chars)
                cv2.rectangle(frame, pt1=(info['x'], info['y']), pt2=(info['x'] + info['w'], info['y'] + info['h']), color=(255, 0, 0), thickness=2)
                cv2.putText(frame, chars[-4:], (info['x'], info['y'] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

                save_dir = os.path.expanduser("Desktop/dubal_ws/src/dubal_eye_and_web/Plates_img")
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)

                # 파일 이름 생성
                now = datetime.now()
                timestamp = now.strftime('%H:%M:%S')
                filename = f"{save_dir}/{timestamp}({len(self.detect_list)}).jpg"

                # 이미지 저장
                cv2.imwrite(filename, frame)

        return plate_chars

    def loop(self, client):
        while True:
            start_time = time.time()  # 시작 시간 기록
            plate_lists = self.process_image(client)

            if len(plate_lists) > 0:
                data['PL_NUM'] = plate_lists[0]
            
            data['PL_STATE'] = len(self.detect_list)
    
            elapsed_time = time.time() - start_time  # 실행 시간 계산
            if elapsed_time < 0.5:
                time.sleep(max(0, 0.5 + start_time - time.time()))


# GPS 데이터 콜백 함수
def gps_callback(msg):
    global data
    data['Long'] = msg.position[0]
    data['Long'] = 127.0772
    data['Lat'] = msg.position[1]
    data['Lat'] = 37.6315
    data['Alt'] = msg.position[2]
    data['SIV'] = msg.position[3]
    data['FIX'] = msg.position[4]
    print(f"Updated GPS Data: {data}")

# IMU 데이터 콜백 함수
def imu_callback(msg):
    global data
    data['r'] = msg.position[0]
    data['p'] = msg.position[1]
    data['y'] = msg.position[1]*180/3.1415926535
    print(f"Updated IMU Data: {data}")

# Controller Input 데이터 콜백 함수
def controller_callback(msg):
    global data
    data['u1'] = msg.position[0]  # pos0 값을 u1에 저장
    data['u2'] = msg.position[1]  # pos1 값을 u2에 저장
    data['u3'] = msg.velocity[0]  # vel0 값을 u3에 저장
    print(f"Updated Controller Input Data: {data}")

def main():
    rclpy.init()
    node = Node('sensor_data_subscriber')
    node.create_subscription(JointState, 'gps_data', gps_callback, 10)
    node.create_subscription(JointState, 'imu_data', imu_callback, 10)
    node.create_subscription(JointState, 'controller_input_data', controller_callback, 10)
    
    pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'


    image_processor = ImageProcessor()
    client = CameraClient('http://13.125.65.10:5024', 'AA:11:BB:22:CC:33')

    client.begin()

    while not client.isConnected:
        time.sleep(0.05)

    client.start_sending_SensorData(data)
    client.start_sending_ImageData()

    ImageThread = threading.Thread(target=image_processor.loop, args=(client,))
    ImageThread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()