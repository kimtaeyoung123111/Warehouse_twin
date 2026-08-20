#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import math

from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

from tf2_ros import Buffer, TransformListener

class VisionMoveItPickNode(Node):
    def __init__(self):
        super().__init__('vision_pick_node')

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 1. 가제보 Detachable Joint 제어 토픽 (붙이기 / 떼기)
        self.attach_pub = self.create_publisher(Empty, '/vacuum_gripper/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/vacuum_gripper/detach', 10)
        
        # 2. 초기 대기 자세 세팅용 관절 제어 토픽
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
            
        # 3. 카메라 영상 수신자
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
            
        self.bridge = CvBridge()
        self.state = 'LOOK'
        
        # 4. MoveIt 2 액션 클라이언트
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.group_name = 'ur_manipulator' 
        self.end_effector_link = 'fixed_arm_wrist_3_link'

        # 💡 [초기 대기 자세] 컨베이어 벨트를 내려다보는 관절 각도 (사용자 지정값 100% 반영)
        self.joint_names = [
            'fixed_arm_shoulder_pan_joint', 'fixed_arm_shoulder_lift_joint',
            'fixed_arm_elbow_joint', 'fixed_arm_wrist_1_joint',
            'fixed_arm_wrist_2_joint', 'fixed_arm_wrist_3_joint'
        ]
        self.pose_look = [0.0, -1.487772, 1.3090, -1.5708, -1.5708, -1.5708]

        # 기본 컨베이어 중앙 기준 기본 좌표 (X=0.5m, Y=0.0m)
        self.base_pick_x = 0.50
        self.base_pick_y = 0.00
        
        self.get_logger().info("🚀 스마트 시각 인지 및 MoveIt 2 픽앤플레이스 노드 시작!")
        self.get_logger().info("🚀 통신망 연결 대기 중... (모터 제어기 응답 확인 대기)")

        self.connection_timer = self.create_timer(0.5, self.check_connection_and_init)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 타겟 Z 기본값 설정
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.43

    def get_box_world_pose(self, box_frame_id="box"):
        try:
            trans = self.tf_buffer.lookup_transform('world', box_frame_id, rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            target_z = z + 0.05 + 0.01  
            return x, y, target_z
        except Exception as e:
            return None, None, None

    def check_connection_and_init(self):
        if self.trajectory_pub.get_subscription_count() > 0:
            self.get_logger().info("✅ 제어기 통신 연결 확인 완료! 초기 셋업 가동")
            self.connection_timer.cancel()  
            self.init_system()              
        else:
            self.get_logger().info("⏳ 제어기 연결을 기다리는 중입니다...")

    def init_system(self):
        self.get_logger().info("🔓 초기 조인트 강제 해제 (Detach)...")
        msg = Empty()
        self.detach_pub.publish(msg)
        cmd = 'gz topic -t "/vacuum_gripper/detach" -m gz.msgs.Empty -p ""'
        subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        self.get_logger().info("👁️ 컨베이어 벨트 감시 대기 위치(Look Pose)로 이동...")
        self.move_joints(self.pose_look, time_sec=2.0)

        self.get_logger().info("🏭 컨베이어 벨트 가동 시작!")
        self.start_conveyor()

    def move_joints(self, joint_angles, time_sec=2.0):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
        msg.points.append(point)
        self.trajectory_pub.publish(msg)

    # 💡 컨베이어 토픽명 사용자 지정값 100% 반영
    def start_conveyor(self):
        cmd = 'gz topic -t "/model/belt_surface/link/base/track_cmd_vel" -m gz.msgs.Double -p "data: 0.5"'
        subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def stop_conveyor(self):
        cmd = 'gz topic -t "/model/belt_surface/link/base/track_cmd_vel" -m gz.msgs.Double -p "data: 0.0"'
        subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def move_to_pose(self, x, y, z, roll=0.0, pitch=3.14159, yaw=3.14159):
        self.get_logger().info(f"📍 MoveIt 2 3D 좌표 이동 명령: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

        if not self.move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("⚠️ MoveIt 2 '/move_action' 서버가 연결되지 않았습니다.")
            return

        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        constraint = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "world"
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.weight = 1.0

        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        bv.primitives.append(sphere)
        bv.primitive_poses.append(Pose(position=Point(x=x, y=y, z=z)))
        pos_constraint.constraint_region = bv
        constraint.position_constraints.append(pos_constraint)

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "world"
        ori_constraint.link_name = self.end_effector_link
        ori_constraint.orientation.x = qx
        ori_constraint.orientation.y = qy
        ori_constraint.orientation.z = qz
        ori_constraint.orientation.w = qw
        ori_constraint.absolute_x_axis_tolerance = 0.05
        ori_constraint.absolute_y_axis_tolerance = 0.05
        ori_constraint.absolute_z_axis_tolerance = 0.05
        ori_constraint.weight = 1.0
        constraint.orientation_constraints.append(ori_constraint)

        goal_msg.request.goal_constraints.append(constraint)
        self.move_group_client.send_goal_async(goal_msg)

    def image_callback(self, msg):
        if self.state != 'LOOK':
            return
            
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_brown = np.array([10, 40, 40])
        upper_brown = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_brown, upper_brown)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 15000:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    offset_x_px = cX - 320
                    offset_y_px = cY - 240

                    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)
                    cv2.circle(frame, (cX, cY), 8, (255, 0, 0), -1)

                    # if abs(offset_x_px) < 50 and abs(offset_y_px) < 50:
                    #     # 픽셀 오차를 실제 미터(m) 단위 오차로 변환
                    #     scale_factor = 0.0008  
                    #     delta_x = offset_y_px * scale_factor
                    #     delta_y = -offset_x_px * scale_factor

                    #     # 💡 클래스 변수에 안전하게 저장
                    #     self.target_x = self.base_pick_x + delta_x
                    #     self.target_y = self.base_pick_y + delta_y

                    #     self.get_logger().info(f"🎯 상자 동적 포착! 비전 목표 3D 좌표: X={self.target_x:.3f}, Y={self.target_y:.3f}")
                    #     self.stop_conveyor()
                    #     self.state = 'EXECUTING'

                    #     # 상자가 완전히 멈추도록 0.5초 대기 후 _start_pick 실행
                    #     self.seq_timer = self.create_timer(0.5, self._start_pick)

                    if 50 < offset_x_px < 100 and abs(offset_y_px) < 50:
                        scale_factor = 0.0008  
                        
                        # 💡 1. 기본 픽셀 변환값 계산
                        raw_delta_x = offset_y_px * scale_factor
                        raw_delta_y = -offset_x_px * scale_factor

                        # 💡 2. [영점 수동 보정 오프셋] (단위: 미터 m)
                        # 로봇이 상자보다 앞/뒤로 빗나가면 offset_x 조절 (예: 0.03 = 3cm 앞)
                        # 로봇이 상자보다 좌/우로 빗나가면 offset_y 조절 (예: -0.05 = 5cm 오른쪽)
                        manual_offset_x = 0.3   # 👈 테스트하며 조율 (예: 0.02, -0.03 등)
                        manual_offset_y = 0.00   # 👈 테스트하며 조율 (예: 0.05, -0.04 등)

                        # 💡 3. 최종 목표 좌표 계산
                        self.target_x = self.base_pick_x + raw_delta_x + manual_offset_x
                        self.target_y = self.base_pick_y + raw_delta_y + manual_offset_y

                        self.get_logger().info(
                            f"🎯 상자 중앙 포착! 보정된 목표 3D 좌표: X={self.target_x:.3f}, Y={self.target_y:.3f}"
                        )
                        self.state = 'EXECUTING'
                        self.stop_conveyor()

                        # 상자가 완전히 멈추도록 0.5초 대기 후 _start_pick 실행
                        self.seq_timer = self.create_timer(0.5, self._start_pick)

        cv2.imshow("Cardboard Box Tracking", frame)
        cv2.waitKey(1)
        
        # 💡 [버그 삭제] 맨 마지막에 엉뚱하게 실행되던 코드 덩어리를 완전히 삭제했습니다!

    def _start_pick(self):
        self.seq_timer.cancel()
        
        # 1. TF에서 진짜 좌표(X, Y, Z) 추출 시도
        tf_x, tf_y, tf_z = self.get_box_world_pose("box")
        
        if tf_x is not None:
            self.target_x = tf_x
            self.target_y = tf_y
            self.target_z = tf_z  
            self.get_logger().info(f"🎯 TF 상자 위치 완벽 파악: X={tf_x:.3f}, Y={tf_y:.3f}, Z={tf_z:.3f}")
        else:
            self.get_logger().warn("⚠️ TF를 못 찾았습니다! 카메라(비전) 기반 계산 좌표를 사용합니다.")
            # 카메라가 계산한 target_x, target_y는 유지하고 높이만 지정
            self.target_z = 0.9 
            
        # 💡 [핵심 버그 수정] 인자 없이 깔끔하게 호출합니다!
        self.execute_moveit_pick_sequence()

    def execute_moveit_pick_sequence(self):
        # 1단계: 상자(Z) 기준 정확히 0.2m(20cm) 위 상공으로 안전 접근
        approach_z = self.target_z + 0.1
        self.get_logger().info(f"1️⃣ MoveIt 2: 동적 상공 0.2m 접근 (Approach Z={approach_z:.3f}m)")
        self.move_to_pose(self.target_x, self.target_y, approach_z)
        self.seq_timer = self.create_timer(5.0, self._step_descend)

    def _step_descend(self):
        self.seq_timer.cancel()  
        
        # 2단계: 실제 상자 표면 높이로 수직 하강
        self.get_logger().info(f"2️⃣ MoveIt 2: 상자 표면({self.target_z:.3f}m) 밀착 하강 (Pick)")
        self.move_to_pose(self.target_x, self.target_y, self.target_z)
        self.seq_timer = self.create_timer(5, self._attach_box) 

    def _attach_box(self):
        self.seq_timer.cancel()  
        
        # 3단계: 가제보 흡착 신호 발사
        self.get_logger().info("🧲 가제보 접착(Attach) 신호 발사!")
        msg = Empty()
        self.attach_pub.publish(msg)
        self.seq_timer = self.create_timer(2, self._step_lift)

    def _step_lift(self):
        self.seq_timer.cancel()  
        
        # 4단계: 집고 난 뒤 다시 상공 0.2m 높이로 수직 상승
        lift_z = self.target_z + 0.2
        self.get_logger().info(f"3️⃣ MoveIt 2: 상자 0.2m 수직 상승 (Lift Z={lift_z:.3f}m)")
        self.move_to_pose(self.target_x, self.target_y, lift_z)
        self.seq_timer = self.create_timer(3.0, self._reset_state)

    def _reset_state(self):
        self.seq_timer.cancel()  
        
        self.get_logger().info("✨ 픽앤플레이스 완료! 대기 위치 복귀 및 컨베이어 재가동")
        self.move_joints(self.pose_look, time_sec=2.0)
        self.start_conveyor()
        self.seq_timer = self.create_timer(2.5, self._set_state_look)

    def _set_state_look(self):
        self.seq_timer.cancel()  
        self.state = 'LOOK'

def main(args=None):
    rclpy.init(args=args)
    node = VisionMoveItPickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()