# Config.py

import math

# -------------------
# 제어기 설정 (Controller)
# -------------------
class ControllerConfig:
    # 퓨어 퍼슛 파라미터
    LOOKAHEAD_GAIN = 0.4      # 속도 비례 탐색 거리 게인 (k)
    MIN_LOOKAHEAD_DIST = 0.1  # 최소 탐색 거리    
    KP_SPEED = 1.5            # 속도 P 제어기 게인

# -------------------
# 로봇 설정 (Robot)
# -------------------
class RobotConfig:
    BASE_SPEED = 3.0          # 기본 주행 속도
    DECELERATION_FACTOR = 2.5 # 감속 강도를 결정하는 계수
    MIN_SPEED_AT_CORNER = 0.5 # 코너에서 허용되는 최소 속도
    WHEELBASE = 0.35          # 로봇 축거 (L)
    MIN_TURN_RADIUS = 0.6     # 경로 생성 시 사용할 로봇의 최소 회전반경

# -------------------
# 시뮬레이션 설정 (Simulation)
# -------------------
class SimConfig:
    DT = 0.1                  # 시뮬레이션 시간 간격
    W1 = (0.0, 5.0)           # 웨이포인트 1 (고정)
    W2 = (10.0, 5.0)          # 웨이포인트 2 (고정)
    W3_DIST = 10.0            # W2로부터 W3까지의 거리
    ANGLE_STEP = 5            # 시뮬레이션 반복 시 각도 증가량
    MAX_ANGLE = 150           # 최대 각도
    PATH_RESOLUTION = 0.2     # 부드러운 경로 생성 시 포인트 간 간격
    ARRIVAL_RADIUS = 0.5      # 최종 목표 지점 도착을 인정하는 반경
    DECEL_ZONE = 7.0          # A지점으로 부터의 감속 구역의 반경

# -------------------
# 시각화 설정 (Visualization)
# -------------------
class VisConfig:
    X_MIN = -5
    X_MAX = 25
    Y_MIN = -5
    Y_MAX = 20
    PAUSE_TIME = 0.001