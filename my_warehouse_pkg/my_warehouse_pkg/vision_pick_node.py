#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty  # 💡 수정: Bool 대신 Empty 메시지 임포트
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionPickNode(Node):
    def __init__(self):
        super().__init__('vision_pick_node')
        
        # 1. 관절 제어 토픽 발행자
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
            
        # 💡 2. 가제보 Detachable Joint 제어 토픽 (붙이기 / 떼기)
        self.attach_pub = self.create_publisher(Empty, '/vacuum_gripper/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/vacuum_gripper/detach', 10)
            
        # 3. 카메라 영상 수신자
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
            
        self.bridge = CvBridge()
        
        self.joint_names = [
            'fixed_arm_shoulder_pan_joint', 'fixed_arm_shoulder_lift_joint',
            'fixed_arm_elbow_joint', 'fixed_arm_wrist_1_joint',
            'fixed_arm_wrist_2_joint', 'fixed_arm_wrist_3_joint'
        ]
        
        self.state = 'LOOK'
        
        # 앞서 수정한 완벽한 좌표들 
        # (주의: pose_pick은 상자를 너무 세게 누르지 않고 1~2cm 상공에 떠 있는 Hovering 상태여야 합니다)
        self.pose_look = [0.0, -1.3464, 1.3090, -1.5708, -1.5708, -1.5708]
        
        # 2) 접근 자세: 박스 위 10cm 대기 (몸통 0.0, 손목 -1.5708 일치)
        # self.pose_approach = [0.0, -1.30, 1.40, -1.67, -1.5708, -1.5708] 
        self.pose_approach = [-0.251327, -1.110781, 1.309000, -1.822127, -1.57080008, -1.822127] 
        
        # 3) 피킹 자세: 박스 표면 밀착 (몸통 0.0, 손목 -1.5708 일치)
        # self.pose_pick = [0.0, -1.45, 1.65, -1.70, -1.5708, -1.5708]    
        self.pose_pick = [-0.251327, -0.639542, 1.309000, -2.199119, -1.570800, -1.780240]    
        
        # 4) 상차 자세: 물건을 들고 위로 복귀
        # self.pose_lift = [0.0, -1.20, 1.20, -1.5708, -1.5708, -1.5708]
        self.pose_lift = [-0.251327, -1.110781, 1.309000, -1.822127, -1.57080008, -1.822127] 
        
        self.timer = self.create_timer(1.0, self.init_pose)
        self.get_logger().info("🚀 시각 인지 및 픽앤플레이스 노드 시작!")

    def init_pose(self):
        self.move_to_pose(self.pose_look, time_sec=2.0)
        self.timer.cancel()

    def move_to_pose(self, joint_angles, time_sec=2.0):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
        msg.points.append(point)
        self.trajectory_pub.publish(msg)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 💡 [주의] 만약 0.1kg짜리 빨간색 테스트 박스를 사용 중이라면 색상 필터(HSV)를 빨간색으로 바꿔야 합니다!
        # 아래는 기존의 갈색 종이 상자(Cardboard) 기준의 필터입니다.
        lower_brown = np.array([10, 40, 40])
        upper_brown = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_brown, upper_brown)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours and self.state == 'LOOK':
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 15000:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    offset_x = cX - 320
                    offset_y = cY - 240
                    
                    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)
                    cv2.circle(frame, (cX, cY), 8, (255, 0, 0), -1)
                    
                    if abs(offset_x) < 50 and abs(offset_y) < 50:
                        self.get_logger().info("🎯 스토퍼 도달! 픽업을 시작합니다.")
                        self.execute_pick_sequence()

        cv2.imshow("Cardboard Box Tracking", frame)
        cv2.waitKey(1)

    def execute_pick_sequence(self):
        self.state = 'EXECUTING'
        self.get_logger().info("1. 목표 상공으로 이동...")
        self.move_to_pose(self.pose_approach, time_sec=2.0)
        self.create_timer(2.5, self._step_pick)

    def _step_pick(self):
        self.get_logger().info("2. 하강하여 상자 위(Hovering) 대기!")
        self.move_to_pose(self.pose_pick, time_sec=1.5)
        
        # 로봇 팔이 다 내려갈 때까지(1.5초) 기다렸다가 Attach 신호 전송
        self.create_timer(3.0, self._attach_box)

    def _attach_box(self):
        self.get_logger().info("🧲 가제보 접착(Attach) 신호 전송!")
        # 💡 Empty 메시지를 생성해서 한 번 쏴줍니다.
        msg = Empty()
        self.attach_pub.publish(msg)
        
        # 관절이 박스에 고정(조인트 생성)될 때까지 1초 대기 후 들어올리기
        self.create_timer(1.0, self._step_lift)

    def _step_lift(self):
        self.get_logger().info("3. 상자 픽업 완료, 다시 들어올리기!")
        self.move_to_pose(self.pose_lift, time_sec=2.0)
        self.create_timer(3.0, self._reset_state)

    def _reset_state(self):
        self.state = 'LOOK'
        self.get_logger().info("✨ 작업 완료. 다음 상자 대기 중...")
        
        # (원한다면 여기서 self.detach_pub.publish(Empty()) 를 쏴서 상자를 툭 떨어뜨릴 수도 있습니다!)

def main(args=None):
    rclpy.init(args=args)
    node = VisionPickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()