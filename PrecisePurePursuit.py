import math
from Point import Point
from Simulator import Simulator
from Visualizer import Visualizer
from Config import SimConfig, RobotConfig
from PathGenerator import generate_random_waypoints, generate_smooth_path
from Robot import Robot

def precise_pure_pursuit(waypoints, visualizer, w3_angle_deg=None):
    """ Precise Pure Pursuit 시뮬레이션을 실행합니다. """
    turn_radius = RobotConfig.MIN_TURN_RADIUS  # 로봇의 최소 회전반경
    resolution = SimConfig.PATH_RESOLUTION  # 경로 생성 시 포인트 간 간격

    path_to_follow, corner_points = generate_smooth_path(waypoints, turn_radius, resolution)
    if not path_to_follow: path_to_follow = waypoints

    simulator = Simulator(path_to_follow, original_waypoints=waypoints, corner_points=corner_points)
    
    if w3_angle_deg == None:
        auto_scale = True
        title = 'Path Following Simulation : Random Multi-path'
    else:
        auto_scale = False
        if w3_angle_deg > 0:
            turn_angle = 180 - w3_angle_deg
        else:
            turn_angle = 180 + w3_angle_deg
        title = f'Path Following Simulation : Multi-angle (Turn Angle: {turn_angle:.1f}°)'
    while simulator.is_running():
        simulator.run_step()
        visualizer.update_plot_realtime(waypoints, path_to_follow, simulator.robot, title, auto_scale)