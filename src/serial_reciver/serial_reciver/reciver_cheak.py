import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu  # 표준 메시지 임포트
from geometry_msgs.msg import Twist  # Turtlesim에 명령을 보내기 위한 메시지 임포트

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.sbus_subscription = self.create_subscription(
            JointState,
            'sbus_data',
            self.sbus_callback,
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu_data',
            self.imu_callback,
            10
        )
        
        # Turtlesim에 명령을 보내기 위한 퍼블리셔 생성
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.twist_msg = Twist()  # Twist 메시지 초기화

    def sbus_callback(self, msg):
        sbus_data = msg.position
        # ch2의 값을 0~1 사이로 변환하여 x축 속도로 설정
        self.twist_msg.linear.x = (1796-sbus_data[1])/1000  # ch2
        # ch1의 값을 0~1 사이로 변환하여 회전각속도로 설정
        self.twist_msg.angular.z = (2000-sbus_data[0])/1000  # ch1
        
        # Turtlesim에 명령 발행
        self.cmd_vel_publisher.publish(self.twist_msg)
        
        print("SBUS:", sbus_data)

    def imu_callback(self, msg):
        imu_data = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        print("IMU:", imu_data)

def main(args=None):
    rclpy.init(args=args)
    
    subscriber = DataSubscriber()
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
