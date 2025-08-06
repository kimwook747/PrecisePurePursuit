# Visualizer.py

import matplotlib.pyplot as plt
import math
from Point import Point
from Config import VisConfig

class Visualizer:
    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
    
    def update_plot_realtime(self, waypoints, smooth_path, robot, title, auto_scale):
        self.ax.cla()
        if auto_scale == False:            
            self.ax.set_xlim(VisConfig.X_MIN, VisConfig.X_MAX)
            self.ax.set_ylim(VisConfig.Y_MIN, VisConfig.Y_MAX)
            #self.ax.set_title(f'Path Following Simulation (Turn Angle: {math.degrees(turn_angle):.1f}°)')
        else:
            all_points = waypoints + smooth_path
            if not all_points:
                all_points = [Point(0, 0)]
            
            x_coords = [p.x for p in waypoints]
            y_coords = [p.y for p in waypoints]
            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)
            x_padding = (x_max - x_min) * 0.1 + 5
            y_padding = (y_max - y_min) * 0.1 + 5
            self.ax.set_xlim(x_min - x_padding, x_max + x_padding)
            self.ax.set_ylim(y_min - y_padding, y_max + y_padding)
            
        self.ax.set_title(title)
        self.ax.set_xlabel("X-coordinate (m)")
        self.ax.set_ylabel("Y-coordinate (m)")
        
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')

        # 웨이포인트 (파란선)
        wx = [p.x for p in waypoints]
        wy = [p.y for p in waypoints]
        self.ax.plot(wx, wy, 'bo-', label='Waypoints', markersize=10)

        # 생성된 부드러운 경로 (회색 점선)
        # 경로 포인트가 3개 초과일 때만(스무딩 됐을 때만) 그린다.
        sx = [p.x for p in smooth_path]
        sy = [p.y for p in smooth_path]
        self.ax.plot(sx, sy, 'k:', label='Smoothed Path')

        # 로봇 이동 경로 (붉은 실선)
        robot_path = robot.get_path()
        rx = [p.x for p in robot_path]
        ry = [p.y for p in robot_path]
        self.ax.plot(rx, ry, 'r-', label='Robot Path')

        # 로봇의 현재 heading 방향 (빨간 화살표)        
        # 현재 로봇 위치 (녹색 원)
        self.ax.plot(robot.position.x, robot.position.y, 'go', markersize=8, label='Robot')

        # 현재 속도 텍스트
        velocity_text = f"v: {robot.velocity:.2f} m/s"
        self.ax.text(robot.position.x + 0.3, robot.position.y + 0.3, velocity_text, fontsize=9, color='darkred')

        self.ax.legend(loc='upper right')
        plt.pause(VisConfig.PAUSE_TIME)

    def close(self):
        plt.ioff()
        print("Simulation finished. Close the plot window to exit.")
        plt.show()