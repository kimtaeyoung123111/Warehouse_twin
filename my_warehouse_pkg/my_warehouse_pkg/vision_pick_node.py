#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionPickNode(Node):
    def __init__(self):
        super().__init__('vision_pick_node')
        
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        
        self.joint_names = [
            'fixed_arm_shoulder_pan_joint', 'fixed_arm_shoulder_lift_joint',
            'fixed_arm_elbow_joint', 'fixed_arm_wrist_1_joint',
            'fixed_arm_wrist_2_joint', 'fixed_arm_wrist_3_joint'
        ]
        
        self.state = 'LOOK'
        
        # 💡 관절 각도 하드코딩 (박스 높이가 30cm나 되므로, 팔이 너무 깊게 내려가지 않도록 수정!)
        self.pose_look = [0, -1.5708, 1.5708, -1.5708, -1.5708, 1.5708]
        self.pose_approach = [-1.5708, -1.30, 1.40, -1.67, -1.57, 0.0] # 박스 위 10cm 대기
        self.pose_pick = [-1.5708, -1.45, 1.65, -1.70, -1.57, 0.0]    # 박스 표면 밀착
        self.pose_lift = [-1.5708, -1.20, 1.20, -1.57, -1.57, 0.0]
        
        self.timer = self.create_timer(1.0, self.init_pose)
        self.get_logger().info("🚀 거대 종이상자 추적 비전 노드 시작!")

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
        
        # 💡 갈색/황토색(골판지 색상) 필터링 범위
        # 조명에 따라 안 잡히면 H, S, V 수치를 조금씩 조절해야 합니다.
        lower_brown = np.array([10, 40, 40])
        upper_brown = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_brown, upper_brown)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours and self.state == 'LOOK':
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # 박스가 화면에 크게 들어왔을 때 (면적 15,000 이상)
            if area > 15000:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    offset_x = cX - 320
                    offset_y = cY - 240
                    
                    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)
                    cv2.circle(frame, (cX, cY), 8, (255, 0, 0), -1)
                    
                    # 박스가 계속 이동 중임을 터미널에 출력
                    self.get_logger().info(f"🚚 박스 이동 중... [중심 오차 X: {offset_x}, Y: {offset_y}]")
                    
                    # 박스가 카메라 정중앙(스토퍼 위치)에 와서 멈추면 피킹 실행!
                    if abs(offset_x) < 50 and abs(offset_y) < 50:
                        self.get_logger().info("🎯 스토퍼 도달 확인! 박스를 잡습니다!")
                        self.execute_pick_sequence()

        # 디버깅용 화면
        cv2.imshow("Cardboard Box Tracking", frame)
        cv2.imshow("Color Mask", mask)
        cv2.waitKey(1)

    def execute_pick_sequence(self):
        self.state = 'EXECUTING'
        self.get_logger().info("1. 목표 상공으로 이동...")
        self.move_to_pose(self.pose_approach, time_sec=2.0)
        self.create_timer(2.5, self._step_pick)

    def _step_pick(self):
        self.get_logger().info("2. 하강하여 흡착 시작!")
        self.move_to_pose(self.pose_pick, time_sec=1.5)
        self.create_timer(2.0, self._step_lift)

    def _step_lift(self):
        self.get_logger().info("3. 픽업 완료, 다시 들어올리기!")
        self.move_to_pose(self.pose_lift, time_sec=2.0)
        self.create_timer(3.0, self._reset_state)

    def _reset_state(self):
        self.state = 'LOOK'
        self.get_logger().info("✨ 작업 완료. 다음 박스 대기 중...")

def main(args=None):
    rclpy.init(args=args)
    node = VisionPickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()