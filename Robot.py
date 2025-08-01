# Robot.py

import math
from Point import Point
from Config import RobotConfig

class Robot:
    def __init__(self, initial_position: Point, initial_heading: float = 0.0, initial_velocity: float = 0.0):
        self.position = initial_position
        self.heading = initial_heading
        self.velocity = initial_velocity
        self.path = [Point(self.position.x, self.position.y)]
        self.L = RobotConfig.WHEELBASE

    def update(self, steering_angle: float, acceleration: float, dt: float):
        self.velocity += acceleration * dt
        self.velocity = max(0.0, self.velocity) # 속도가 음수가 되지 않도록 방지
        
        # 정확한 자전거 운동 모델 공식 적용
        self.heading += (self.velocity / self.L) * math.tan(steering_angle) * dt
        
        dx = self.velocity * math.cos(self.heading) * dt
        dy = self.velocity * math.sin(self.heading) * dt
        self.position = Point(self.position.x + dx, self.position.y + dy)
        self.path.append(Point(self.position.x, self.position.y))

    def get_path(self):
        return self.path

    def get_state(self):
        return self.position, self.heading, self.velocity