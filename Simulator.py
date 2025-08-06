# Simulator.py

import math
from Point import Point
from Robot import Robot
from PathTracker import PathTracker
from Config import SimConfig

class Simulator:
    def __init__(self, path: list, original_waypoints: list = None, corner_points: list = None):
        self.path = path
        self.original_waypoints = original_waypoints if original_waypoints is not None else path
        self.corner_points = corner_points if corner_points is not None else []
        
        self.robot = Robot(initial_position=self.path[0])
        self.path_tracker = PathTracker()
        self.dt = SimConfig.DT
        self.arrival_radius = SimConfig.ARRIVAL_RADIUS

        self.current_segment_idx = 0

    def is_running(self):
        dist_to_final = math.sqrt((self.robot.position.x - self.path[-1].x)**2 + (self.robot.position.y - self.path[-1].y)**2)
        return dist_to_final > self.arrival_radius

    def run_step(self):
        if not self.is_running():
            return
        position, heading, velocity = self.robot.get_state()
        
        if self.original_waypoints and self.current_segment_idx < len(self.original_waypoints) - 1:
            target_waypoint = self.original_waypoints[self.current_segment_idx + 1]
            dist_to_waypoint = math.sqrt((position.x - target_waypoint.x)**2 + (position.y - target_waypoint.y)**2)

            if dist_to_waypoint < self.arrival_radius:
                self.current_segment_idx += 1

        steering_angle = self.path_tracker.calculate_steering_angle(position, heading, velocity, self.path)
        
        if len(self.path) > len(self.original_waypoints):
            next_corner_idx = self.current_segment_idx + 1
        else:
            next_corner_idx = 0
        acceleration = self.path_tracker.calculate_acceleration(
            velocity, steering_angle, position, next_corner_idx, self.corner_points
        )
        
        self.robot.update(steering_angle, acceleration, self.dt)