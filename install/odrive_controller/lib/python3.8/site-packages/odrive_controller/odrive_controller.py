#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import odrive
from odrive.enums import *
import time

ODRIVE_SERIAL_NUMBER = "3682387E3333"

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')

        self.subscription_joint0 = self.create_subscription(Float64MultiArray,'/joint0_velocity_controller/commands',self.listener_callback_joint0,10)
        # self.subscription_joint0 = self.create_subscription(Float64MultiArray,'/joint1_velocity_controller/commands',self.listener_callback_joint1,10)

        self.publisher_dubal_data = self.create_publisher(JointState, 'dubal_data', 10)

        self.odrv0 = odrive.find_any(serial_number=ODRIVE_SERIAL_NUMBER)
        self.check_and_clear_errors()

        self.odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.config.enable_watchdog = False
        self.initial_position = self.odrv0.axis0.encoder.pos_estimate
        # self.odrv0.axis1.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        # self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        # self.odrv0.axis1.config.enable_watchdog = False

    def check_and_clear_errors(self):
        for axis in [self.odrv0.axis0]:
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
        
        current_velocity = self.odrv0.axis0.encoder.vel_estimate
        current_position = self.odrv0.axis0.encoder.pos_estimate - self.initial_position

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["axis0"]
        joint_state_msg.position = [current_position]
        joint_state_msg.velocity = [current_velocity]
        
        self.publisher_dubal_data.publish(joint_state_msg)

    # def listener_callback_joint1(self, msg):
    #     hall_state = self.odrv0.axis1.encoder.hall_state
        
    #     if hall_state == 0:
    #         pass

    #     self.check_and_clear_errors()
        
    #     velocity = msg.data[0]
    #     self.odrv0.axis1.controller.input_vel = velocity
        
    #     current_velocity = self.odrv0.axis1.encoder.vel_estimate
    #     current_position = self.odrv0.axis1.encoder.pos_estimate - self.initial_position

    #     joint_state_msg = JointState()
    #     joint_state_msg.header.stamp = self.get_clock().now().to_msg()
    #     joint_state_msg.name = ["axis1"]
    #     joint_state_msg.position = [current_position]
    #     joint_state_msg.velocity = [current_velocity]
        
    #     self.publisher_dubal_data.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)

    odrive_controller = ODriveController()
    
    try:
        rclpy.spin(odrive_controller)
    except KeyboardInterrupt:
        pass
    finally:
        odrive_controller.odrv0.axis0.controller.input_vel = 0
        # odrive_controller.odrv0.axis1.controller.input_vel = 0
        
        odrive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
