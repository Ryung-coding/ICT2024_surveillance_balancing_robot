import serial
import struct
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu

class SBUSReceiver:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.packet_format = '<16hB'
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
                            'ch1': float(unpacked_data[0]),
                            'ch2': float(unpacked_data[1]),
                            'ch5': float(unpacked_data[4]),
                            'ch8': float(unpacked_data[7])
                        }
                        return True
        return False

    def close(self):
        if self.ser.is_open:
            self.ser.close()

class IMUReceiver:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.packet = {'r': -1.0, 'p': -1.0, 'y': -1.0}

    def update(self):
        if self.ser.in_waiting >= 13:
            data1 = self.ser.read(1)
            if data1 == b'\x02':
                data2 = self.ser.read(1)
                if data2 == b'\x09':
                    data3 = self.ser.read(1)
                    if data3 == b'\x2a':
                        data4 = self.ser.read(1)
                        if data4 == b'\xf0':
                            data5 = self.ser.read(1)
                            if data5 == b'\x35':
                                data = self.ser.read(8)
                                r = struct.unpack('<h', data[0:2])[0]
                                p = struct.unpack('<h', data[2:4])[0]
                                y = struct.unpack('<h', data[4:6])[0]
                                self.packet = {
                                    'r': float(r) / 100.0,
                                    'p': float(p) / 100.0,
                                    'y': float(y) / 100.0
                                }
                                
                                return True
        return False

    def close(self):
        if self.ser.is_open:
            self.ser.close()

class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')
        self.sbus_receiver = SBUSReceiver('/dev/ttyACM0', 921600) #ttyACM0
        self.imu_receiver = IMUReceiver('/dev/ttyUSB1', 921600) #ttyUSB0'
        self.sbus_pub = self.create_publisher(JointState, 'sbus_data', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)

    def publish_data(self):
        while rclpy.ok():
            if self.sbus_receiver.update():
                sbus_msg = JointState()
                sbus_msg.header.stamp = self.get_clock().now().to_msg()
                sbus_msg.name = ['channel1', 'channel2', 'channel5', 'channel8']
                sbus_msg.position = [self.sbus_receiver.packet['ch1'], 
                                     self.sbus_receiver.packet['ch2'], 
                                     self.sbus_receiver.packet['ch5'], 
                                     self.sbus_receiver.packet['ch8']]
                self.sbus_pub.publish(sbus_msg)

            if self.imu_receiver.update():
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0 
                imu_msg.orientation.z = 0.0 
                imu_msg.orientation.w = 0.0 
                imu_msg.angular_velocity.x = self.imu_receiver.packet['r']
                imu_msg.angular_velocity.y = self.imu_receiver.packet['p']
                imu_msg.angular_velocity.z = self.imu_receiver.packet['y']
                self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    
    publisher = DataPublisher()
    try:
        publisher.publish_data()
    except KeyboardInterrupt:
        pass
    finally:
        publisher.sbus_receiver.close()
        publisher.imu_receiver.close()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
