# PathTracker.py

import math
from Point import Point
from Config import ControllerConfig, RobotConfig, SimConfig

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
        self.last_corner_idx = -1

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

    def _find_next_corner_angle(self, robot_position: Point, waypoints: list):
        if len(waypoints) < 3:
            return math.pi # 코너 없음

        # 로봇과 가장 가까운 웨이포인트(코너 후보) 찾기
        min_dist_sq = float('inf')
        closest_wp_idx = -1
        # W1(인덱스 0)은 코너가 아니므로 1부터 탐색
        for i in range(1, len(waypoints)):
            # 로봇이 웨이포인트에 가까워질 때만 고려
            dist_sq = (robot_position.x - waypoints[i].x)**2 + (robot_position.y - waypoints[i].y)**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_wp_idx = i
        
        # 가장 가까운 웨이포인트가 코너를 형성하는지 확인 (마지막 점이 아니어야 함)
        if closest_wp_idx <= 0 or closest_wp_idx >= len(waypoints) - 1:
            return math.pi

        w_prev = waypoints[closest_wp_idx - 1]
        w_corner = waypoints[closest_wp_idx]
        w_next = waypoints[closest_wp_idx + 1]

        v_in = (w_prev.x - w_corner.x, w_prev.y - w_corner.y)
        v_out = (w_next.x - w_corner.x, w_next.y - w_corner.y)
        len_in = math.sqrt(v_in[0]**2 + v_in[1]**2)
        len_out = math.sqrt(v_out[0]**2 + v_out[1]**2)
        if len_in < 1e-6 or len_out < 1e-6: return math.pi
        
        v_in_unit = (v_in[0]/len_in, v_in[1]/len_in)
        v_out_unit = (v_out[0]/len_out, v_out[1]/len_out)
        dot_product = max(-1.0, min(1.0, v_in_unit[0] * v_out_unit[0] + v_in_unit[1] * v_out_unit[1]))
        
        return math.acos(dot_product)
    
    def calculate_acceleration(self, robot_velocity: float, steering_angle: float, robot_position: Point, next_corner_idx, corner_points: list):
        # 예측 감속
        predictive_target_v = self.base_speed

        if corner_points[next_corner_idx] and next_corner_idx > self.last_corner_idx:
            next_corner_point = corner_points[next_corner_idx]
            point_a = next_corner_point[0]
            turn_angle = next_corner_point[4]
            dist_to_a = math.sqrt((robot_position.x - point_a.x)**2 + (robot_position.y - point_a.y)**2)

            if dist_to_a < 0.5:
                self.last_corner_idx = next_corner_idx

            decel_zone = SimConfig.DECEL_ZONE
            if dist_to_a < decel_zone:
                progress = 1.0 - (dist_to_a / decel_zone)
                epsilon = 1e-6
                sharpness = (math.pi / (turn_angle + epsilon))                
                weight_value = sharpness / 10.0
                sharpness_factor = min(1.0, sharpness / 10 + weight_value) 
                predictive_target_v = self.base_speed - self.deceleration_factor * self.base_speed * sharpness_factor * progress

        # 반응 제어
        reactive_target_v = self.base_speed - self.deceleration_factor * self.base_speed * (abs(steering_angle) / (math.pi / 2.0))
            
        # 두 방식 중 더 낮은 속도를 목표로 설정
        target_velocity = min(predictive_target_v, reactive_target_v)
        target_velocity = max(self.min_speed_at_corner, target_velocity)
        
        # P-제어기를 이용해 목표 속도에 도달하기 위한 가속도를 계산
        velocity_error = target_velocity - robot_velocity
        acceleration = self.kp_speed * velocity_error
        
        return acceleration