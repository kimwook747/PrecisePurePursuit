# PathGenerator.py

import math
import random
from Point import Point

def _check_intersection(seg1_start, seg1_end, seg2_start, seg2_end):
    p0_x, p0_y = seg1_start.x, seg1_start.y
    p1_x, p1_y = seg1_end.x, seg1_end.y
    p2_x, p2_y = seg2_start.x, seg2_start.y
    p3_x, p3_y = seg2_end.x, seg2_end.y

    s1_x = p1_x - p0_x
    s1_y = p1_y - p0_y
    s2_x = p3_x - p2_x
    s2_y = p3_y - p2_y

    denominator = (-s2_x * s1_y + s1_x * s2_y)
    if denominator == 0:
        return False

    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / denominator
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / denominator

    if s > 0 and s < 1 and t > 0 and t < 1:
        return True
    return False

def generate_random_waypoints(num_waypoints=10, min_len=5, max_len=10, min_angle_deg=-150, max_angle_deg=150):
    waypoints = [Point(0, 0)]
    current_angle = 0.0
    max_retries = 50

    for i in range(num_waypoints - 1):
        for _ in range(max_retries):
            length = random.uniform(min_len, max_len)
            turn_angle = math.radians(random.uniform(min_angle_deg, max_angle_deg))
            
            new_angle = current_angle + turn_angle
            prev_point = waypoints[-1]
            new_x = prev_point.x + length * math.cos(new_angle)
            new_y = prev_point.y + length * math.sin(new_angle)
            candidate_point = Point(new_x, new_y)
            
            new_segment_start = prev_point
            new_segment_end = candidate_point
            
            has_intersection = False
            if len(waypoints) > 2:
                for j in range(len(waypoints) - 2):
                    if _check_intersection(waypoints[j], waypoints[j+1], new_segment_start, new_segment_end):
                        has_intersection = True
                        break
            
            if not has_intersection:
                waypoints.append(candidate_point)
                current_angle = new_angle
                break
        else:
            print("Warning: Path generation stopped due to too many self-intersection retries.")
            break
            
    return waypoints

def normalize_angle(angle):
    """ 각도를 -pi ~ +pi 범위로 정규화하는 헬퍼 함수이다. """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def _generate_path_segment(w1, w2, w3, turn_radius):
 
    v_in = (w1.x - w2.x, w1.y - w2.y)
    v_out = (w3.x - w2.x, w3.y - w2.y)

    len_in = math.sqrt(v_in[0]**2 + v_in[1]**2)
    len_out = math.sqrt(v_out[0]**2 + v_out[1]**2)

    if len_in < 1e-6 or len_out < 1e-6:
        return None

    v_in_unit = (v_in[0] / len_in, v_in[1] / len_in)
    v_out_unit = (v_out[0] / len_out, v_out[1] / len_out)

    dot_product = v_in_unit[0] * v_out_unit[0] + v_in_unit[1] * v_out_unit[1]
    dot_product = max(-1.0, min(1.0, dot_product))
    turn_angle = math.acos(dot_product)

    # 예외 상황(직선/U턴)일 때, 원본 웨이포인트를 그대로 반환한다.
    if abs(turn_angle) < math.radians(1.0) or abs(turn_angle - math.pi) < math.radians(1.0):
        return None
    
    dist_to_point_a = turn_radius / math.tan(turn_angle / 2.0)
    if(dist_to_point_a > len_in or dist_to_point_a > len_out):
        dist_to_point_a = min(dist_to_point_a, len_in, len_out)
        turn_radius = dist_to_point_a / math.tan(turn_angle / 2.0) if abs(math.tan(turn_angle / 2.0)) > 1e-6 else 0
    
    point_a = Point(w2.x + dist_to_point_a * v_in_unit[0], w2.y + dist_to_point_a * v_in_unit[1])
    point_a_prime = Point(w2.x + dist_to_point_a * v_out_unit[0], w2.y + dist_to_point_a * v_out_unit[1])

    bisector_vec = (v_in_unit[0] + v_out_unit[0], v_in_unit[1] + v_out_unit[1])
    len_b = math.sqrt(bisector_vec[0]**2 + bisector_vec[1]**2)
    bisector_unit = (bisector_vec[0]/len_b, bisector_vec[1]/len_b)

    dist_to_icr = math.sqrt(dist_to_point_a**2 + turn_radius**2)
    icr = Point(w2.x + dist_to_icr * bisector_unit[0], w2.y + dist_to_icr * bisector_unit[1])
    
    return [point_a, point_a_prime, icr, turn_radius, turn_angle]

def generate_smooth_path(waypoints, turn_radius, resolution):
    if len(waypoints) < 3:
        path = []
        if len(waypoints) == 2:
            p1, p2 = waypoints[0], waypoints[1]
            length = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            num_points = int(length / resolution)
            if num_points > 0:
                for i in range(num_points + 1):
                    t = i / float(num_points)
                    path.append(Point(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y)))
        return path, []
    
    corner_points = [None]
    for i in range(len(waypoints) - 2):
        w1, w2, w3 = waypoints[i], waypoints[i+1], waypoints[i+2]
        geom = _generate_path_segment(w1, w2, w3, turn_radius)
        corner_points.append(geom)
    corner_points.append(None)
    
    #valid_angles = [a for a in turn_angles if a is not None]
    #avg_turn_angle = sum(valid_angles) / len(valid_angles) if valid_angles else 0.0

    smoothed_path = []
    for i in range(len(waypoints) - 1):
        line_start = corner_points[i][1] if corner_points[i] else waypoints[i]
        line_end = corner_points[i+1][0] if corner_points[i+1] else waypoints[i+1]

        length = math.sqrt((line_end.x - line_start.x)**2 + (line_end.y - line_start.y)**2)
        num_points = int(length / resolution)
        if num_points > 0:
            for j in range(num_points):
                t = j / float(num_points)
                smoothed_path.append(Point(line_start.x + t * (line_end.x - line_start.x), line_start.y + t * (line_end.y - line_start.y)))
        
        if corner_points[i+1]:
            point_a, point_a_prime, icr, turn_radius, _ = corner_points[i+1]
            start_angle = math.atan2(point_a.y - icr.y, point_a.x - icr.x)
            end_angle = math.atan2(point_a_prime.y - icr.y, point_a_prime.x - icr.x)
            angle_diff = normalize_angle(end_angle - start_angle)
            num_points_arc = int(abs(angle_diff) * turn_radius / resolution)
            if num_points_arc > 0:
                for j in range(num_points_arc + 1):
                    t = j / float(num_points_arc)
                    current_angle = start_angle + t * angle_diff
                    smoothed_path.append(Point(icr.x + turn_radius * math.cos(current_angle), icr.y + turn_radius * math.sin(current_angle)))

    return smoothed_path, corner_points