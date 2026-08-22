import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GridNavigator(Node):
    def __init__(self):
        super().__init__('grid_navigator')

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # MiR100의 가제보 모터 플러그인과 연결된 토픽으로 속도 명령 퍼블리시
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_gz', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.state = 'MOVE_FORWARD'
        self.start_time = self.get_clock().now()
        self.get_logger().info('AMR 격자 이동을 시작합니다: 전진 중...')
        
    def timer_callback(self):
        msg = Twist()
        now = self.get_clock().now()
        # 경과 시간 계산 (초 단위)
        elapsed_time = (now - self.start_time).nanoseconds / 1e9

        if self.state == 'MOVE_FORWARD':
            if elapsed_time < 4.0:  # 4초 동안 직진
                msg.linear.x = 0.5  # 0.5 m/s 속도
            else:
                self.state = 'TURN_LEFT'
                self.start_time = now
                self.get_logger().info('경로 변경: 90도 좌회전 중...')
                
        elif self.state == 'TURN_LEFT':
            if elapsed_time < 1.57:  # 약 90도 회전 (1.57초 동안 1.0 rad/s)
                msg.angular.z = 1.0  
            else:
                self.state = 'STOP'
                self.get_logger().info('목적지 도착: 정지합니다.')
                
        elif self.state == 'STOP':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = GridNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
