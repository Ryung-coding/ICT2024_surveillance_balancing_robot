import serial 
import struct 

class TeensyReceiver:
    def __init__(self, port, baudrate):
        self.port = port 
        self.baudrate = baudrate 
        self.packet_format = '<18h3iH8B'  
        self.packet_size = struct.calcsize(self.packet_format) 
        self.packet = {}
        self.ser = serial.Serial(port, baudrate, timeout=1)

    def update(self):
        if self.ser.in_waiting >= self.packet_size + 2:  
            first_byte = self.ser.read(1)
            if first_byte == b'\xFF':
                second_byte = self.ser.read(1)
                if second_byte == b'\xFF':
                    data = self.ser.read(self.packet_size)
                    if len(data) == self.packet_size:
                        unpacked_data = struct.unpack(self.packet_format, data)
                        self.packet = {
                            'Long': unpacked_data[18],
                            'Lat': unpacked_data[19],
                            'Alt': unpacked_data[20],
                            'SIV': unpacked_data[22],
                            'FIX': unpacked_data[23],
                            'year': unpacked_data[21],
                            'month': unpacked_data[24],
                            'day': unpacked_data[25],
                            'hour': unpacked_data[26],
                            'minute': unpacked_data[27],
                            'second': unpacked_data[28],
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
                            'ch17': unpacked_data[16],
                            'ch18': unpacked_data[17],
                            'sbus_signal': unpacked_data[29]
                        }
                        return True
        return False

    def close(self):
        if self.ser.is_open:
            self.ser.close()