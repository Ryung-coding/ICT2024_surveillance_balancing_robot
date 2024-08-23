import serial.tools.list_ports
import json
import time
from teensy_receiver import TeensyReceiver
from imu_receiver import IMUReceiver

def select_serial_port():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    if not ports:
        print("No serial ports found")
        return None
    
    print("Available serial ports:")
    print("0: Exit")
    for i, port in enumerate(ports):
        print(f"{i + 1}: {port}")

    selected_port1 = None
    while selected_port1 is None:
        try:
            selection = int(input("Select a serial port teensy. (0 to exit, 1-{0}): ".format(len(ports))))
            if selection == 0:
                break
            elif 1 <= selection <= len(ports):
                selected_port1 = ports[selection - 1]
            else:
                print("Invalid selection. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

    if selected_port1 is not None:
        selected_port2 = None
        while selected_port2 is None:
            try:
                selection = int(input("Select a serial port IMU. (0 to exit, 1-{0}): ".format(len(ports))))
                if selection == 0:
                    break
                elif 1 <= selection <= len(ports):
                    selected_port2 = ports[selection - 1]
                else:
                    print("Invalid selection. Please try again.")
            except ValueError:
                print("Invalid input. Please enter a number.")
    else:
        selected_port2 = None
    
    return selected_port1, selected_port2

if __name__ == "__main__":
    port1, port2 = select_serial_port()

    data = {'r': -1.0, 'p': -1.0, 'y': -1.0,
            'gx': -1.0, 'gy': -1.0, 'gz': -1.0,
            'ch1': -1, 'ch2': -1, 'ch3': -1, 'ch4': -1,
            'ch5': -1, 'ch6': -1, 'ch7': -1, 'ch8': -1,
            'ch9': -1, 'ch10': -1, 'ch11': -1, 'ch12': -1,
            'ch13': -1, 'ch14': -1, 'ch15': -1, 'ch16': -1,
            'ch17': -1, 'ch18': -1,
            'sbus_signal': -1,
            'Long': -1, 'Lat': -1, 'Alt': -1,
            'SIV': -1, 'FIX': -1,
            'year': -1, 'month': -1, 'day': -1, 'hour': -1, 'minute': -1, 'second': -1
            }

    if port1 and port2:
        teensy_receiver = TeensyReceiver(port1, 921600)
        imu_receiver = IMUReceiver(port2, 921600)
        last_print_time = time.time()
        t = time.time()
        dt = 0
        
        try:
            while True:
                if teensy_receiver.update():  # TeensyReceiver ì‹¤í–‰
                    data['ch1'] = teensy_receiver.packet['ch1']
                    data['ch2'] = teensy_receiver.packet['ch2']
                    data['ch3'] = teensy_receiver.packet['ch3']
                    data['ch4'] = teensy_receiver.packet['ch4']
                    data['ch5'] = teensy_receiver.packet['ch5']
                    data['ch6'] = teensy_receiver.packet['ch6']
                    data['ch7'] = teensy_receiver.packet['ch7']
                    data['ch8'] = teensy_receiver.packet['ch8']
                    data['ch9'] = teensy_receiver.packet['ch9']
                    data['ch10'] = teensy_receiver.packet['ch10']
                    data['ch11'] = teensy_receiver.packet['ch11']
                    data['ch12'] = teensy_receiver.packet['ch12']
                    data['ch13'] = teensy_receiver.packet['ch13']
                    data['ch14'] = teensy_receiver.packet['ch14']
                    data['ch15'] = teensy_receiver.packet['ch15']
                    data['ch16'] = teensy_receiver.packet['ch16']
                    data['ch17'] = teensy_receiver.packet['ch17']
                    data['ch18'] = teensy_receiver.packet['ch18']
                    data['sbus_signal'] = teensy_receiver.packet['sbus_signal']
                    data['Long'] = teensy_receiver.packet['Long']
                    data['Lat'] = teensy_receiver.packet['Lat']
                    data['Alt'] = teensy_receiver.packet['Alt']
                    data['SIV'] = teensy_receiver.packet['SIV']
                    data['FIX'] = teensy_receiver.packet['FIX']
                    data['year'] = teensy_receiver.packet['year']
                    data['month'] = teensy_receiver.packet['month']
                    data['day'] = teensy_receiver.packet['day']
                    data['hour'] = teensy_receiver.packet['hour']
                    data['minute'] = teensy_receiver.packet['minute']
                    data['second'] = teensy_receiver.packet['second']

                if imu_receiver.update():  # IMUReceiver ì‹¤í–‰
                    data['r'] = imu_receiver.packet['r']
                    data['p'] = imu_receiver.packet['p']
                    data['y'] = imu_receiver.packet['y']
                    data['gx'] = imu_receiver.packet['gx']
                    data['gy'] = imu_receiver.packet['gy']
                    data['gz'] = imu_receiver.packet['gz']

                current_time = time.time()
                if current_time - last_print_time >= 0.01:  # 100Hz ì£¼ê¸°ë¡œ ë°ì´í„° ì¶œë ¥
                    print(json.dumps(data, indent=4))
                    last_print_time = current_time

        except KeyboardInterrupt:
            print("\nTerminating program.")
        finally:
            teensy_receiver.close()
    else:
        print("No port selected. Terminating program.")