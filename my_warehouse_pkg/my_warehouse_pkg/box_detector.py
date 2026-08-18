import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.br = CvBridge()
        self.get_logger().info("👀 시각 인지 노드 시작: 빨간색 박스 탐색 중...")

    def image_callback(self, msg):
        # 1. ROS 이미지를 OpenCV 이미지로 변환
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        
        # 2. 빨간색 추출을 위해 HSV 색상 공간으로 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 빨간색 범위 마스크 생성 (OpenCV에서 빨간색은 양 끝단에 걸쳐 있음)
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        mask = mask1 + mask2
        
        # 3. 박스 윤곽선 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] != 0:
                # 박스 중심점 픽셀 계산
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # 시각적 표시 그리기
                cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)
                
                # 이미지 중앙(320, 240)을 기준으로 한 오차 추정
                offset_x = cX - 320
                offset_y = cY - 240
                self.get_logger().info(f"🎯 타겟 발견! 오차 -> X: {offset_x}px, Y: {offset_y}px")
        
        # 화면에 카메라 뷰 띄우기
        cv2.imshow("Robot Camera View", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()