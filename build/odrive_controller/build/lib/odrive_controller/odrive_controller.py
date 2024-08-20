#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import odrive
from odrive.enums import *
import time

ODRIVE_SERIAL_NUMBER = "3682387E3333"

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/joint0_velocity_controller/commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # ODrive 초기화
        self.odrv0 = odrive.find_any(serial_number=ODRIVE_SERIAL_NUMBER)
        self.odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        print("속도제어설정 완료!")

        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        print("폐루프설정 완료!")

    def listener_callback(self, msg):
        # msg.data[0]이 속도 명령을 담고 있다고 가정
        velocity = msg.data[0]
        print(f"받은 속도 명령: {velocity}")
        self.odrv0.axis0.controller.input_vel = velocity

def main(args=None):
    rclpy.init(args=args)

    odrive_controller = ODriveController()

    rclpy.spin(odrive_controller)

    odrive_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()