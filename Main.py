# main.py

import math
from Point import Point
from Simulator import Simulator
from Visualizer import Visualizer
from Config import SimConfig, RobotConfig
from PathGenerator import generate_smooth_path

def main():
    w1 = Point(*SimConfig.W1)
    w2 = Point(*SimConfig.W2)
    w3_radius = SimConfig.W3_RADIUS
    turn_radius = RobotConfig.MIN_TURN_RADIUS
    
    w3_angle_deg = 0.0
    visualizer = Visualizer()

    while w3_angle_deg <= SimConfig.MAX_ANGLE:
        angle_rad = math.radians(w3_angle_deg)
        w3_x = w2.x + w3_radius * math.cos(angle_rad)
        w3_y = w2.y + w3_radius * math.sin(angle_rad)
        w3 = Point(w3_x, w3_y)
        
        original_waypoints = [w1, w2, w3]
        
        # 1. 부드러운 경로 생성 시도
        #    직선/U턴인 경우 원본 웨이포인트([w1,w2,w3])가 반환된다.
        path_to_follow, turn_angle = generate_smooth_path(original_waypoints, turn_radius, SimConfig.PATH_RESOLUTION)

        # 2. 생성된 경로로 시뮬레이터 실행
        simulator = Simulator(path_to_follow, turn_angle)
        
        while simulator.is_running():
            simulator.run_step()
            visualizer.update_plot_realtime(
                original_waypoints,
                path_to_follow, # 시각화 시 스무딩 여부 판단을 위해 전달
                simulator.robot,
                turn_angle
            )
        
        w3_angle_deg += SimConfig.ANGLE_STEP
    
    visualizer.close()

if __name__ == "__main__":
    main()