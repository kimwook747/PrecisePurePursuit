# Visualizer.py

import matplotlib.pyplot as plt
from Point import Point
from Config import VisConfig

class Visualizer:
    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
    
    def update_plot_realtime(self, waypoints, smooth_path, robot, w3_angle_deg):
        self.ax.cla()
        
        self.ax.set_xlim(VisConfig.X_MIN, VisConfig.X_MAX)
        self.ax.set_ylim(VisConfig.Y_MIN, VisConfig.Y_MAX)
        self.ax.set_xlabel("X-coordinate (m)")
        self.ax.set_ylabel("Y-coordinate (m)")
        self.ax.set_title(f'Path Following Simulation (W3 Angle: {w3_angle_deg:.1f}°)')
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')

        # 원래 웨이포인트 (파란선)
        wx = [p.x for p in waypoints]
        wy = [p.y for p in waypoints]
        self.ax.plot(wx, wy, 'bo-', label='Waypoints', markersize=10)
        
        # 생성된 부드러운 경로 (회색 점선)
        # 경로 포인트가 3개 초과일 때만(스무딩 됐을 때만) 그린다.
        if len(smooth_path) > 3:
            sx = [p.x for p in smooth_path]
            sy = [p.y for p in smooth_path]
            self.ax.plot(sx, sy, 'k:', label='Smoothed Path')

        # 로봇 이동 경로 (붉은 실선)
        robot_path = robot.get_path()
        rx = [p.x for p in robot_path]
        ry = [p.y for p in robot_path]
        self.ax.plot(rx, ry, 'r-', label='Robot Path')
        
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