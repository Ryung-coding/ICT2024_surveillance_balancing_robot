import serial.tools.list_ports
import json
import time
from sbus_receiver import SBUSReceiver
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
            selection = int(input("Select a serial port SBUS. (0 to exit, 1-{0}): ".format(len(ports))))
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
            'ch1': -1, 'ch2': -1, 'ch3': -1, 'ch4': -1,
            'ch5': -1, 'ch6': -1, 'ch7': -1, 'ch8': -1,
            'ch9': -1, 'ch10': -1, 'ch11': -1, 'ch12': -1,
            'ch13': -1, 'ch14': -1, 'ch15': -1, 'ch16': -1,
            'sbus_signal': 99
            }

    if port1 and port2:
        sbus_receiver = SBUSReceiver(port1, 921600)
        imu_receiver = IMUReceiver(port2, 921600)
        last_print_time = time.time()
        
        try:
            while True:
                if sbus_receiver.update():  # SBUSReceiver 실행
                    data['ch1'] = sbus_receiver.packet['ch1']
                    data['ch2'] = sbus_receiver.packet['ch2']
                    data['ch3'] = sbus_receiver.packet['ch3']
                    data['ch4'] = sbus_receiver.packet['ch4']
                    data['ch5'] = sbus_receiver.packet['ch5']
                    data['ch6'] = sbus_receiver.packet['ch6']
                    data['ch7'] = sbus_receiver.packet['ch7']
                    data['ch8'] = sbus_receiver.packet['ch8']
                    data['ch9'] = sbus_receiver.packet['ch9']
                    data['ch10'] = sbus_receiver.packet['ch10']
                    data['ch11'] = sbus_receiver.packet['ch11']
                    data['ch12'] = sbus_receiver.packet['ch12']
                    data['ch13'] = sbus_receiver.packet['ch13']
                    data['ch14'] = sbus_receiver.packet['ch14']
                    data['ch15'] = sbus_receiver.packet['ch15']
                    data['ch16'] = sbus_receiver.packet['ch16']
                    data['sbus_signal'] = sbus_receiver.packet['sbus_signal']

                if imu_receiver.update():  # IMUReceiver 실행
                    data['r'] = imu_receiver.packet['r']
                    data['p'] = imu_receiver.packet['p']
                    data['y'] = imu_receiver.packet['y']

                current_time = time.time()
                if current_time - last_print_time >= 0.01:  # 100Hz 주기로 데이터 출력
                    print(json.dumps(data, indent=4))
                    last_print_time = current_time

        except KeyboardInterrupt:
            print("\nTerminating program.")
        finally:
            sbus_receiver.close()
    else:
        print("No port selected. Terminating program.")
