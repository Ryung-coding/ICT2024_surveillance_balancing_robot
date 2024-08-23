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
                                    self.packet['r'] = r/100.0
                                    self.packet['p'] = p/100.0
                                    self.packet['y'] = y/100.0
                                    return True
                            
                            elif data5 == b'\x34': # gyro Index (52) 
                                data = self.ser.read(8)

                                gx = struct.unpack('<h', data[0:2])[0]
                                gy = struct.unpack('<h', data[2:4])[0]
                                gz = struct.unpack('<h', data[4:6])[0]

                                if data[6] == self.checksum(0x2a, 0xf0, 0x34, *data[:6]):
                                    self.packet['gx'] = gx/10.0
                                    self.packet['gy'] = gy/10.0
                                    self.packet['gz'] = gz/10.0
                                    return True
                    
        return False
    
    def close(self):
        if self.ser.is_open:
            self.ser.close()