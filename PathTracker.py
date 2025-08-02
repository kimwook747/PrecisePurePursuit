# PathTracker.py

import math
from Point import Point
from Config import ControllerConfig, RobotConfig

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class PathTracker:
    def __init__(self):
        self.lookahead_gain = ControllerConfig.LOOKAHEAD_GAIN
        self.min_lookahead_dist = ControllerConfig.MIN_LOOKAHEAD_DIST
        self.kp_speed = ControllerConfig.KP_SPEED
        self.L = RobotConfig.WHEELBASE
        self.base_speed = RobotConfig.BASE_SPEED
        self.min_speed_at_corner = RobotConfig.MIN_SPEED_AT_CORNER
        self.deceleration_factor = RobotConfig.DECELERATION_FACTOR
        self.target_idx = 0

    def _find_target_index(self, robot_position: Point, path: list, lookahead_dist: float):
        min_dist_sq = float('inf')
        
        # 로봇과 가장 가까운 점부터 탐색 시작
        for i in range(self.target_idx, len(path)):
            dist_sq = (robot_position.x - path[i].x)**2 + (robot_position.y - path[i].y)**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                self.target_idx = i
        
        # 가장 가까운 점에서부터 Ld만큼 떨어진 점을 찾음
        while self.target_idx < len(path) - 1:
            dist_to_next = math.sqrt((robot_position.x - path[self.target_idx].x)**2 + (robot_position.y - path[self.target_idx].y)**2)
            if dist_to_next > lookahead_dist:
                return self.target_idx
            self.target_idx += 1
            
        return len(path) - 1

    def calculate_steering_angle(self, robot_position: Point, robot_heading: float, robot_velocity: float, path: list):
        Ld = self.lookahead_gain * robot_velocity + self.min_lookahead_dist
        
        # 경로가 3개 이하의 점으로만 이루어진 경우, 세그먼트 전환 로직 추가
        if len(path) <= 3:
            # W2에 가까워지면 다음 세그먼트를 보도록 target_idx를 강제 업데이트
            dist_to_w2 = math.sqrt((robot_position.x - path[1].x)**2 + (robot_position.y - path[1].y)**2)
            if dist_to_w2 < Ld * 1.5 : # 탐색거리의 1.5배 이내로 들어오면
                self.target_idx = 2
            else:
                self.target_idx = 1
            target_point = path[self.target_idx]
        else: # 부드러운 경로 추종
             target_idx = self._find_target_index(robot_position, path, Ld)
             target_point = path[target_idx]

        alpha = normalize_angle(math.atan2(target_point.y - robot_position.y, target_point.x - robot_position.x) - robot_heading)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), Ld)
        
        return normalize_angle(delta)

    def calculate_acceleration(self, robot_velocity: float, steering_angle: float, current_path_index: int, total_path_points: int, turn_angle: float):
        # 경로가 스무딩되지 않은 경우, 반응 제어만 사용
        if total_path_points <= 3:
            turn_factor = abs(steering_angle) / (math.pi / 2.0)
            target_velocity = self.base_speed - self.deceleration_factor * self.base_speed * turn_factor
        else:
            # 예측 감속 (스무딩된 경로에만 적용)
            predictive_target_v = self.base_speed
            start_decel_progress = 0.3
            end_decel_progress = 0.5
            current_progress = current_path_index / total_path_points

            if start_decel_progress < current_progress < end_decel_progress:
                # turn_angle이 작을수록(급커브) 감속량을 크게 함 (1/turn_angle)
                epsilon = 1e-6
                sharpness = (math.pi / (turn_angle + epsilon))
                # sharpness 값을 0~1 범위로 만들기 위한 스케일링
                weight_value = sharpness / 10.0
                sharpness_factor = min(1.0, sharpness / 10 + weight_value) 

                predictive_target_v = self.base_speed - self.deceleration_factor * self.base_speed * sharpness_factor

            # 반응 제어
            turn_factor = abs(steering_angle) / (math.pi / 2.0)
            reactive_target_v = self.base_speed - self.deceleration_factor * self.base_speed * turn_factor
            
            # 두 방식 중 더 낮은 속도를 목표로 설정
            target_velocity = min(predictive_target_v, reactive_target_v)
        
        target_velocity = max(self.min_speed_at_corner, target_velocity)
        
        velocity_error = target_velocity - robot_velocity
        acceleration = self.kp_speed * velocity_error
        
        return acceleration