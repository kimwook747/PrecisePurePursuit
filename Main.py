# main.py

import math
from Point import Point
from Simulator import Simulator
from Visualizer import Visualizer
from Config import SimConfig, RobotConfig
from PathGenerator import generate_random_waypoints, generate_smooth_path
from Robot import Robot
from PrecisePurePursuit import precise_pure_pursuit

def run_multi_angle_scenario():
    """ 3-Waypoint 시뮬레이션을 각도별로 반복 실행합니다. """
    w1 = Point(*SimConfig.W1)
    w2 = Point(*SimConfig.W2)
    dist_to_w3 = SimConfig.W3_DIST
    turn_radius = RobotConfig.MIN_TURN_RADIUS
    w3_angle_deg = 0.0
    visualizer = Visualizer()
    
    while w3_angle_deg <= SimConfig.MAX_ANGLE:
        angle_rad = math.radians(w3_angle_deg)
        w3 = Point(w2.x + dist_to_w3 * math.cos(angle_rad), w2.y + dist_to_w3 * math.sin(angle_rad))        
        waypoints = [w1, w2, w3]
        
        precise_pure_pursuit(waypoints, visualizer, w3_angle_deg)

        w3_angle_deg += SimConfig.ANGLE_STEP
    visualizer.close()

def run_random_path_scenario():
    """ 랜덤 경로 시뮬레이션을 실행합니다. """
    waypoints = generate_random_waypoints(num_waypoints=10)  
    visualizer = Visualizer()
    
    precise_pure_pursuit(waypoints, visualizer)

    visualizer.close()

def main():
    while True:
        print("\n--- 메뉴 선택 ---")
        print("1: 3-Waypoint 시뮬레이션 실행")
        print("2: 랜덤 경로 시뮬레이션 실행")
        print("q: 종료")
        choice = input("원하는 기능의 번호를 입력하세요: ")
        
        if choice == '1':
            run_multi_angle_scenario()
            break
        elif choice == '2':
            run_random_path_scenario()
            break
        elif choice.lower() == 'q':
            print("프로그램을 종료합니다.")
            break
        else:
            print("잘못된 입력입니다. 다시 시도해주세요.")

if __name__ == "__main__":
    main()