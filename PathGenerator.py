# PathGenerator.py

import math
from Point import Point

def normalize_angle(angle):
    """ 각도를 -pi ~ +pi 범위로 정규화하는 헬퍼 함수이다. """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def generate_smooth_path(waypoints, turn_radius, resolution=0.1):
    """
    3개의 웨이포인트를 입력받아, 부드러운 '직선-호-직선' 경로를 생성한다.
    직선/U턴과 같은 예외 상황에서는 원본 웨이포인트를 반환한다.
    """
    if len(waypoints) != 3:
        raise ValueError("This generator works for 3 waypoints only.")

    w1, w2, w3 = waypoints
    
    v_in = (w1.x - w2.x, w1.y - w2.y)
    v_out = (w3.x - w2.x, w3.y - w2.y)

    len_in = math.sqrt(v_in[0]**2 + v_in[1]**2)
    len_out = math.sqrt(v_out[0]**2 + v_out[1]**2)

    if len_in < 1e-6 or len_out < 1e-6:
        return waypoints, 1e-6

    v_in_unit = (v_in[0] / len_in, v_in[1] / len_in)
    v_out_unit = (v_out[0] / len_out, v_out[1] / len_out)

    dot_product = v_in_unit[0] * v_out_unit[0] + v_in_unit[1] * v_out_unit[1]
    dot_product = max(-1.0, min(1.0, dot_product))
    turn_angle = math.acos(dot_product)

    # 예외 상황(직선/U턴)일 때, 원본 웨이포인트를 그대로 반환한다.
    if abs(turn_angle) < math.radians(1.0) or abs(turn_angle - math.pi) < math.radians(1.0):
        return waypoints, 180
    
    dist_to_tangent = turn_radius / math.tan(turn_angle / 2.0)
    dist_to_tangent = min(dist_to_tangent, len_in, len_out)
    
    point_a = Point(w2.x + dist_to_tangent * v_in_unit[0], w2.y + dist_to_tangent * v_in_unit[1])
    point_a_prime = Point(w2.x + dist_to_tangent * v_out_unit[0], w2.y + dist_to_tangent * v_out_unit[1])

    bisector_vec = (v_in_unit[0] + v_out_unit[0], v_in_unit[1] + v_out_unit[1])
    len_b = math.sqrt(bisector_vec[0]**2 + bisector_vec[1]**2)
    bisector_unit = (bisector_vec[0]/len_b, bisector_vec[1]/len_b)

    dist_to_icr = math.sqrt(dist_to_tangent**2 + turn_radius**2)
    icr = Point(w2.x + dist_to_icr * bisector_unit[0], w2.y + dist_to_icr * bisector_unit[1])
    
    smooth_path = []
    
    # 1. 직선 W1 -> A
    num_points = int(math.sqrt((point_a.x - w1.x)**2 + (point_a.y - w1.y)**2) / resolution)
    if num_points > 1:
        for i in range(num_points):
            t = i / float(num_points)
            smooth_path.append(Point(w1.x + t * (point_a.x - w1.x), w1.y + t * (point_a.y - w1.y)))
    
    # 2. 원형 호 A -> A'
    start_angle = math.atan2(point_a.y - icr.y, point_a.x - icr.x)
    end_angle = math.atan2(point_a_prime.y - icr.y, point_a_prime.x - icr.x)
    angle_diff = normalize_angle(end_angle - start_angle)
        
    num_points = int(abs(angle_diff) * turn_radius / resolution)
    if num_points > 1:
        for i in range(num_points + 1):
            t = i / float(num_points)
            current_angle = start_angle + t * angle_diff
            smooth_path.append(Point(icr.x + turn_radius * math.cos(current_angle), icr.y + turn_radius * math.sin(current_angle)))

    # 3. 직선 A' -> W3
    num_points = int(math.sqrt((w3.x - point_a_prime.x)**2 + (w3.y - point_a_prime.y)**2) / resolution)
    if num_points > 1:
        for i in range(1, num_points + 1):
            t = i / float(num_points)
            smooth_path.append(Point(point_a_prime.x + t * (w3.x - point_a_prime.x), point_a_prime.y + t * (w3.y - point_a_prime.y)))
    
    return smooth_path, turn_angle