# Simulator.py

import math
from Point import Point
from Robot import Robot
from PathTracker import PathTracker
from Config import SimConfig

class Simulator:
    def __init__(self, path: list, turn_angle: float):
        self.path = path
        self.turn_anlge = turn_angle
        self.robot = Robot(initial_position=self.path[0])
        self.path_tracker = PathTracker()
        self.dt = SimConfig.DT
        self.arrival_radius = SimConfig.ARRIVAL_RADIUS

    def is_running(self):
        dist_to_final = math.sqrt((self.robot.position.x - self.path[-1].x)**2 + (self.robot.position.y - self.path[-1].y)**2)
        return dist_to_final > self.arrival_radius

    def run_step(self):
        if not self.is_running():
            return
        position, heading, velocity = self.robot.get_state()
        
        steering_angle = self.path_tracker.calculate_steering_angle(position, heading, velocity, self.path)
        
        current_path_index = self.path_tracker.target_idx
        total_path_points = len(self.path)
        acceleration = self.path_tracker.calculate_acceleration(velocity, steering_angle, current_path_index, total_path_points, self.turn_anlge)
        
        self.robot.update(steering_angle, acceleration, self.dt)