#!/usr/bin/env python3

import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ConveyorController(Node):
    def __init__(self):
        super().__init__('conveyor_controller')
        
        # 1. ros_gz_bridge 자동 실행 (서브프로세스 띄우기)
        bridge_cmd = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/belt_surface/link/base/track_cmd_vel@std_msgs/msg/Float64@gz.msgs.Double',
            '--ros-args', '-r', '/model/belt_surface/link/base/track_cmd_vel:=/conveyor/cmd_vel'
        ]
        
        self.get_logger().info('🌉 ros_gz_bridge를 코드 내부에서 자동으로 실행합니다...')
        self.bridge_process = subprocess.Popen(bridge_cmd)
        
        # 브릿지가 정상 구동될 때까지 1초 대기
        time.sleep(1.0)
        
        # 2. ROS 2 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Float64, '/conveyor/cmd_vel', 10)
        
        # 3. 1초마다 컨베이어 속도 명령을 전송할 타이머 설정
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.target_speed = 0.5  # m/s
        self.get_logger().info(f'🚀 컨베이어 제어 노드 작동 시작! (목표 속도: {self.target_speed} m/s)')

    def timer_callback(self):
        msg = Float64()
        msg.data = self.target_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'속도 명령 전송: {msg.data} m/s')

    def stop(self):
        # 노드가 종료될 때 정지 명령 발행
        msg = Float64()
        msg.data = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('🛑 컨베이어 정지 명령 전송')

        # 백그라운드로 실행해 둔 ros_gz_bridge 프로세스 안전하게 종료
        if hasattr(self, 'bridge_process') and self.bridge_process:
            self.get_logger().info('🌉 ros_gz_bridge 프로세스를 종료합니다...')
            self.bridge_process.terminate()
            self.bridge_process.wait()

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료 요청 (Ctrl+C)')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()