#!/usr/bin/python3
"""venue_params.py : the values to check at the venue (체크리스트.md), as they are set now.

Reads param/mission_real.yaml and param/perception_real.yaml and prints each
checklist item's current value, grouped as in the checklist. Use it before and
after editing to see what changed.

  tools/venue_params.py            all groups
  tools/venue_params.py 주차        only the groups whose name contains '주차'
"""
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PARAM = os.path.join(HERE, '..', 'src', 'autorace_bringup', 'param')
M, P = 'mission_real.yaml', 'perception_real.yaml'

GROUPS = [
    ('1 조명: 차선 색', [
        (P, 'detect_lane', 'detect.lane.white.lightness_l', '흰색 밝기 하한'),
        (P, 'detect_lane', 'detect.auto_threshold_min_lightness', '자동 조정 하한'),
        (P, 'detect_lane', 'detect.lane.white.saturation_h', '흰색 채도 상한'),
        (P, 'detect_lane', 'detect.lane.yellow.hue_l', '노란색 색상 하한'),
        (P, 'detect_lane', 'detect.lane.yellow.hue_h', '노란색 색상 상한'),
        (P, 'detect_lane', 'detect.lane.yellow.saturation_l', '노란색 채도 하한'),
        (P, 'detect_lane', 'detect.lane.yellow.lightness_l', '노란색 밝기 하한'),
        (P, 'detect_lane', 'detect.glare_fill_m', '반사광 메우기 (m)'),
    ]),
    ('1 조명: 신호등', [
        (M, 'detect_traffic_light', 'lamp.saturation_l', '켜진 램프 채도 하한'),
        (M, 'detect_traffic_light', 'lamp.value_l', '켜진 램프 밝기 하한'),
        (M, 'detect_traffic_light', 'red.hue_margin', '빨강 색상 폭'),
        (M, 'detect_traffic_light', 'yellow.hue_l', '노랑 색상 하한'),
        (M, 'detect_traffic_light', 'yellow.hue_h', '노랑 색상 상한'),
        (M, 'detect_traffic_light', 'green.hue_l', '초록 색상 하한'),
        (M, 'detect_traffic_light', 'green.hue_h', '초록 색상 상한'),
        (M, 'detect_traffic_light', 'min_area', '램프 최소 면적 (px)'),
        (M, 'detect_traffic_light', 'housing.value_h', '하우징 밝기 상한'),
        (M, 'mission_manager', 'green_streak', '출발에 필요한 연속 초록'),
    ]),
    ('1 조명: 표지판', [
        (M, 'detect_sign', 'blue.hue_l', '파랑 색상 하한'),
        (M, 'detect_sign', 'blue.hue_h', '파랑 색상 상한'),
        (M, 'detect_sign', 'blue.saturation_l', '파랑 채도 하한'),
        (M, 'detect_sign', 'blue.value_l', '파랑 밝기 하한'),
        (M, 'detect_sign', 'min_area', '표지판 최소 면적 (px)'),
        (M, 'detect_sign', 'pole_cut', '기둥 잘라내기 비율'),
        (M, 'mission_manager', 'sign_streak', '연속 인식 횟수'),
    ]),
    ('1 조명: 차단바, 정지선', [
        (M, 'detect_level_crossing', 'red.saturation_l', '바 빨강 채도 하한'),
        (M, 'detect_level_crossing', 'red.value_l', '바 빨강 밝기 하한'),
        (M, 'detect_level_crossing', 'fragments.saturation_l', '조각 빨강 채도 하한'),
        (M, 'detect_level_crossing', 'fragments.value_l', '조각 빨강 밝기 하한'),
        (M, 'detect_level_crossing', 'roi_top', 'ROI 위 (화면 비율)'),
        (M, 'detect_level_crossing', 'roi_left', 'ROI 왼쪽 (아래 변)'),
        (M, 'detect_level_crossing', 'roi_right', 'ROI 오른쪽 (아래 변)'),
        (M, 'detect_level_crossing', 'roi_top_left', 'ROI 위 왼쪽 (윗변)'),
        (M, 'detect_level_crossing', 'roi_top_right', 'ROI 위 오른쪽 (윗변)'),
        (M, 'detect_level_crossing', 'closed_angle', '닫힘: 수평에서 (rad)'),
        (M, 'detect_level_crossing', 'closed_span', '닫힘: ROI 폭 비율'),
        (M, 'detect_level_crossing', 'stripe.min_thickness', '가까움 기준 줄무늬 두께 (px)'),
        (P, 'detect_stop_line', 'red.saturation_l', '정지선 채도 하한'),
        (P, 'detect_stop_line', 'red.value_l', '정지선 밝기 하한'),
    ]),
    ('1 조명: 터널 어두움', [
        (M, 'tunnel_mission', 'dark_ratio', '어두움 비율'),
        (M, 'tunnel_mission', 'bright_ratio', '밝아짐 비율'),
    ]),
    ('2 치수: 차선, 카메라', [
        (P, 'detect_lane', 'lane.width_m', '차선 폭 (선 중심 사이, m)'),
        (P, 'bev_projector', 'camera.height', '카메라 높이 (m)'),
        (P, 'bev_projector', 'camera.pitch', '카메라 숙임 (rad)'),
        (P, 'bev_projector', 'camera.lateral_offset', '카메라 좌우 보정 (m)'),
    ]),
    ('2 치수: 주차', [
        (M, 'parking_mission', 'entrance_limit', '입구 찾을 거리 (m)'),
        (M, 'parking_mission', 'entrance_gap_min', '입구 틈 최소 길이 (m)'),
        (M, 'parking_mission', 'entrance_line_min', '입구 앞 노란선 최소 길이 (m)'),
        (M, 'parking_mission', 'entrance_empty_pictures', '입구로 볼 빈 사진 수'),
        (M, 'parking_mission', 'entrance_gap_half', '틈 지나 회전 지점 (m)'),
        (M, 'parking_mission', 'entrance_turn_angle', '입구 회전 (rad)'),
        (M, 'parking_mission', 'lane_length_min', '진입로 최소 (m)'),
        (M, 'parking_mission', 'lane_length_max', '진입로 최대 (m)'),
        (M, 'parking_mission', 'bay_station', '칸 위치 (m)'),
        (M, 'parking_mission', 'bay_depth', '칸 깊이 (m)'),
        (M, 'parking_mission', 'swing_lead', '칸 회전 미리 (m)'),
        (M, 'parking_mission', 'bay_turn_angle', '칸 회전 (rad)'),
        (M, 'parking_mission', 'turn_offset', '후진 후 회전 위치 (m)'),
        (M, 'parking_mission', 'back_arc', '칸에서 후진 호로 나가기'),
        (M, 'parking_mission', 'back_arc_radius', '후진 호 반경 (m)'),
        (M, 'parking_mission', 'turn_slow_gain', '회전 끝 감속 비율'),
        (M, 'parking_mission', 'turn_min_rate', '회전 끝 최저 (rad/s)'),
        (M, 'parking_mission', 'stop_gain', '직진·후진 끝 감속 비율'),
        (M, 'parking_mission', 'stop_min_speed', '직진·후진 끝 최저 (m/s)'),
        (M, 'mission_manager', 'mission_timeout.parking', '주차 제한 시간 (s)'),
    ]),
    ('2 치수: 교차로, 공사, 신호등', [
        (M, 'intersection_mission', 'branch_distance', '교차로 갈래 길이 (m)'),
        (M, 'construction_mission', 'obstacle_count', '장애물 수'),
        (M, 'construction_mission', 'next_obstacle_distance', '장애물 사이 최대 (m)'),
        (M, 'construction_mission', 'max_obstacle_across', '도로 옆 한계 (m)'),
        (M, 'construction_mission', 'robot_radius', '장애물과 최소 거리 (m)'),
        (M, 'construction_mission', 'comfort_clearance', '여유 거리 (m)'),
        (M, 'traffic_light_mission', 'stop_distance', '신호등 정지 거리 (m)'),
    ]),
    ('2 치수: 터널', [
        (M, 'tunnel_mission', 'inside', '내부 한 변 (m)'),
        (M, 'tunnel_mission', 'entrance_right', '입구 중심, 오른쪽 벽에서 (m)'),
        (M, 'tunnel_mission', 'exit_x', '출구 위치 (m)'),
        (M, 'tunnel_mission', 'opening_width', '입구 폭 (m)'),
        (M, 'tunnel_mission', 'front_trigger', '입구 벽 인식 거리 (m)'),
        (M, 'tunnel_mission', 'robot_radius', '벽과 최소 거리 (m)'),
    ]),
    ('3 주행: 차선 제어', [
        (P, 'lane_controller', 'max_speed', '직선 최고 속도 (m/s)'),
        (P, 'lane_controller', 'cornering_rate', '커브 회전 상한 (rad/s)'),
        (P, 'lane_controller', 'pursuit_gain', '조향 세기'),
        (P, 'detect_lane', 'lane.lookahead_base', '직선 보는 거리 기본 (m)'),
        (P, 'detect_lane', 'lane.lookahead_speed_gain', '직선 보는 거리 속도 계수'),
        (P, 'detect_lane', 'lane.path_lookahead_base', '커브 보는 거리 기본 (m)'),
        (P, 'detect_lane', 'lane.path_lookahead_gain', '커브 보는 거리 속도 계수'),
        (P, 'detect_lane', 'lane.curve_enter_angle', '커브 판정 각도 (rad)'),
        (P, 'lane_controller', 'recovery_timeout', '차선 잃고 찾는 시간 (s)'),
    ]),
    ('3 주행: 미션 속도', [
        (M, 'construction_mission', 'speed', '공사 구간 (m/s)'),
        (M, 'parking_mission', 'approach_speed', '주차 입구까지 (m/s)'),
        (M, 'parking_mission', 'entrance_speed', '입구 진입 (m/s)'),
        (M, 'parking_mission', 'up_lane_speed', '진입로 (m/s)'),
        (M, 'parking_mission', 'speed', '주차장 안 (m/s)'),
        (M, 'parking_mission', 'turn_rate', '제자리 회전 (rad/s)'),
        (M, 'tunnel_mission', 'speed', '터널 안 (m/s)'),
        (M, 'tunnel_mission', 'approach_speed', '터널 입구 접근 (m/s)'),
        (M, 'level_crossing_mission', 'open_time', '바 올라간 뒤 대기 (s)'),
    ]),
    ('3 주행: 안전장치', [
        (P, 'cmd_vel_mux', 'scan_change_threshold', '끼임 감지 라이다 변화 (m)'),
        (P, 'cmd_vel_mux', 'stall_timeout', '정지 판정 (s)'),
    ]),
]


def lookup(cfg, dotted):
    node = cfg
    for part in dotted.split('.'):
        if not isinstance(node, dict) or part not in node:
            return '(코드 기본값)'
        node = node[part]
    return node


def main():
    want = sys.argv[1] if len(sys.argv) > 1 else ''
    files = {name: yaml.safe_load(open(os.path.join(PARAM, name))) for name in (M, P)}
    for title, items in GROUPS:
        if want not in title:
            continue
        print(f'== {title}')
        for name, node, key, label in items:
            params = (files[name].get(node) or {}).get('ros__parameters', {})
            print(f'  {label:<28} {str(lookup(params, key)):<14} {node}: {key}  [{name}]')
        print()


if __name__ == '__main__':
    main()
