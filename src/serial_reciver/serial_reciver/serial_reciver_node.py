import serial
import struct
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import radians

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

import serial
import struct

class IMUReceiver:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.packet = {'r': -1.0, 'p': -1.0, 'y': -1.0, 'gx': -1.0, 'gy': -1.0, 'gz': -1.0}

    def checksum(self, *args):
        cs = 0
        for byte in args:
            cs += byte
        return cs & 0xFF  # Ensure it fits in one byte

    def update(self):
        if self.ser.in_waiting >= 13:
            data1 = self.ser.read(1)
            if data1 == b'\x02': # STX 0x02
                data2 = self.ser.read(1)
                if data2 == b'\x09': # Length 0x09
                    data3 = self.ser.read(1)
                    if data3 == b'\x2a': # Device ID (42)
                        data4 = self.ser.read(1)
                        if data4 == b'\xf0': # Command 0xF0
                            data5 = self.ser.read(1)

                            if data5 == b'\x35': # rpy Index (53)
                                data = self.ser.read(8)

                                r = struct.unpack('<h', data[0:2])[0]
                                p = struct.unpack('<h', data[2:4])[0]
                                y = struct.unpack('<h', data[4:6])[0]
                                
                                if data[6] == self.checksum(0x2a, 0xf0, 0x35, *data[:6]):
                                    self.packet['r'] = radians(r/100.0)
                                    self.packet['p'] = radians(p/100.0)
                                    self.packet['y'] = radians(y/100.0)
                                    return True
                            
                            elif data5 == b'\x34': # gyro Index (52) 
                                data = self.ser.read(8)

                                gx = struct.unpack('<h', data[0:2])[0]
                                gy = struct.unpack('<h', data[2:4])[0]
                                gz = struct.unpack('<h', data[4:6])[0]

                                if data[6] == self.checksum(0x2a, 0xf0, 0x34, *data[:6]):
                                    self.packet['gx'] = radians(gx/10.0)
                                    self.packet['gy'] = radians(gy/10.0)
                                    self.packet['gz'] = radians(gz/10.0)
                                    return True
                    
        return False
    
    def close(self):
        if self.ser.is_open:
            self.ser.close()


class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')
        self.teensy_receiver = TeensyReceiver('/dev/ttyACM0', 921600)
        self.imu_receiver = IMUReceiver('/dev/ttyUSB0', 921600)

        self.sbus_pub = self.create_publisher(JointState, 'sbus_data', 10)
        self.gps_pub = self.create_publisher(JointState, 'gps_data', 10)
        self.imu_pub = self.create_publisher(JointState, 'imu_data', 10)

    def publish_data(self):

        while rclpy.ok():

            if self.teensy_receiver.update():
                #SBUS
                sbus_msg = JointState()
                sbus_msg.header.stamp = self.get_clock().now().to_msg()
                sbus_msg.name = ['Heading', 'Thrust', 'Leg','Connect', 'Kill']
                sbus_msg.position = [
                                    float(self.teensy_receiver.packet['ch1']), 
                                    float(self.teensy_receiver.packet['ch2']), 
                                    float(self.teensy_receiver.packet['ch5']),
                                    float(self.teensy_receiver.packet['ch6']), 
                                    float(self.teensy_receiver.packet['ch8'])
]
                self.sbus_pub.publish(sbus_msg)

                #GPS
                gps_msg = JointState()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.name = ['Long', 'Lat', 'Alt', 'SIV','FIX', 'year', 'month', 'day','hour', 'minute', 'second']
                gps_msg.position = [
                                    float(self.teensy_receiver.packet['Long']), 
                                    float(self.teensy_receiver.packet['Lat']), 
                                    float(self.teensy_receiver.packet['Alt']), 
                                    float(self.teensy_receiver.packet['SIV']), 
                                    float(self.teensy_receiver.packet['FIX']), 
                                    float(self.teensy_receiver.packet['year']), 
                                    float(self.teensy_receiver.packet['month']), 
                                    float(self.teensy_receiver.packet['day']), 
                                    float(self.teensy_receiver.packet['hour']), 
                                    float(self.teensy_receiver.packet['minute']), 
                                    float(self.teensy_receiver.packet['second'])
                                ]
                self.gps_pub.publish(gps_msg)

            if self.imu_receiver.update():
                imu_msg = JointState()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.name = ['roll[rad]', 'pitch[rad]', 'yaw[rad]']

                imu_msg.position = [
                                    float(self.imu_receiver.packet['r']), 
                                    float(self.imu_receiver.packet['p']), 
                                    float(self.imu_receiver.packet['y'])
                                    ]

                imu_msg.velocity = [
                                    float(self.imu_receiver.packet['gx']), 
                                    float(self.imu_receiver.packet['gy']), 
                                    float(self.imu_receiver.packet['gz'])
                                    ]
                self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    
    publisher = DataPublisher()
    try:
        publisher.publish_data()
    except KeyboardInterrupt:
        pass
    finally:
        publisher.teensy_receiver.close()
        publisher.imu_receiver.close()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
