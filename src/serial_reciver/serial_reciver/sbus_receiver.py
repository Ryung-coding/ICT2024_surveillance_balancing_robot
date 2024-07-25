import serial # 시리얼 통신
import struct # 시리얼 패킷 분석

class SBUSReceiver:
    def __init__(self, port, baudrate):
        self.port = port # 시리얼 포트
        self.baudrate = baudrate # 보드레이트
        self.packet_format = '<16hB'  # 패킷 구성 : 16개의 int16_t & 1개의 uint8_t 
        self.packet_size = struct.calcsize(self.packet_format) # 패킷 바이트 사이즈
        self.packet = {} # 패킷 저장 dictionary
        self.ser = serial.Serial(port, baudrate, timeout=1)

    def update(self):
        if self.ser.in_waiting >= self.packet_size + 2:  # Start bytes (2) 포함
            first_byte = self.ser.read(1)
            if first_byte == b'\xFF':
                second_byte = self.ser.read(1)
                if second_byte == b'\xFF':
                    data = self.ser.read(self.packet_size)
                    if len(data) == self.packet_size:
                        unpacked_data = struct.unpack(self.packet_format, data)
                        self.packet = {
                            'ch1': unpacked_data[0],
                            'ch2': unpacked_data[1],
                            'ch3': unpacked_data[2],
                            'ch4': unpacked_data[3],
                            'ch5': unpacked_data[4],
                            'ch6': unpacked_data[5],
                            'ch7': unpacked_data[6],
                            'ch8': unpacked_data[7],
                            'ch9': unpacked_data[8],
                            'ch10': unpacked_data[9],
                            'ch11': unpacked_data[10],
                            'ch12': unpacked_data[11],
                            'ch13': unpacked_data[12],
                            'ch14': unpacked_data[13],
                            'ch15': unpacked_data[14],
                            'ch16': unpacked_data[15],
                            'sbus_signal': unpacked_data[16]
                        }
                        return True
        return False

    def close(self):
        if self.ser.is_open:
            self.ser.close()
