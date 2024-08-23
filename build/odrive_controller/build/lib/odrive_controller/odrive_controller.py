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

        self.subscription_joint0 = self.create_subscription(
            Float64MultiArray,
            '/joint0_velocity_controller/commands',
            self.listener_callback_joint0,
            10)

        # self.subscription_joint1 = self.create_subscription(
        #     Float64MultiArray,
        #     '/joint1_velocity_controller/commands',
        #     self.listener_callback_joint1,
        #     10)

        self.odrv0 = odrive.find_any(serial_number=ODRIVE_SERIAL_NUMBER)
        self.check_and_clear_errors()
        self.odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.config.enable_watchdog = False

        # self.odrv0.axis1.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        # self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        # self.odrv0.axis1.config.enable_watchdog = False

    def check_and_clear_errors(self):
        for axis in [self.odrv0.axis0]:  #, self.odrv0.axis1]:
            axis_error = axis.error
            motor_error = axis.motor.error

            if axis_error & AxisError.MOTOR_FAILED:
                axis.clear_errors()

            if motor_error & MotorError.CURRENT_SENSE_SATURATION:
                axis.clear_errors()

    def listener_callback_joint0(self, msg):
        hall_state = self.odrv0.axis0.encoder.hall_state
        
        if hall_state == 0:
            pass

        self.check_and_clear_errors()
        
        velocity = msg.data[0]
        self.odrv0.axis0.controller.input_vel = velocity

    # def listener_callback_joint1(self, msg):
    #     hall_state = self.odrv0.axis1.encoder.hall_state
        
    #     if hall_state == 0:
    #         pass

    #     self.check_and_clear_errors()
        
    #     velocity = msg.data[0]
    #     self.odrv0.axis1.controller.input_vel = velocity

def main(args=None):
    rclpy.init(args=args)

    odrive_controller = ODriveController()
    rclpy.spin(odrive_controller)
    odrive_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
