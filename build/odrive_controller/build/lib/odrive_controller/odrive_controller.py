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
        
        # M0의 에러 상태 확인 및 처리 (에러 무시)
        self.check_and_clear_errors()

        # 속도 제어 모드 설정
        self.odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        print("속도제어 설정 완료!")

        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        print("폐루프 설정 완료!")

        # 리미트 설정 무시
        self.odrv0.axis0.config.enable_watchdog = False  # 워치독 타이머 비활성화 (오류로 인한 자동 멈춤 방지)

    def check_and_clear_errors(self):
        axis_error = self.odrv0.axis0.error
        motor_error = self.odrv0.axis0.motor.error

        # AxisError.MOTOR_FAILED 무시
        if axis_error & AxisError.MOTOR_FAILED:
            print(f"AxisError.MOTOR_FAILED 감지: {hex(axis_error)}. 에러를 무시하고 계속 동작합니다.")
            self.odrv0.axis0.clear_errors()  # 오류 클리어

        # MotorError.CURRENT_SENSE_SATURATION 무시
        if motor_error & MotorError.CURRENT_SENSE_SATURATION:
            print(f"MotorError.CURRENT_SENSE_SATURATION 감지: {hex(motor_error)}. 에러를 무시하고 계속 동작합니다.")
            self.odrv0.axis0.clear_errors()  # 오류 클리어

    def listener_callback(self, msg):
        # Hall 상태 출력
        hall_state = self.odrv0.axis0.encoder.hall_state
        print(f"현재 Hall 상태: {hall_state}")
        
        # Hall 상태가 0이어도 무시하고 동작 유지
        if hall_state == 0:
            print("경고: Hall 상태가 0입니다. 하지만 무시하고 계속 동작합니다.")

        # 오류 확인 및 처리 (무시)
        self.check_and_clear_errors()
        
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
