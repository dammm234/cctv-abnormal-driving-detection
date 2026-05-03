#!/usr/bin/env python3
"""
build_dilemma_road.py — v5 (하이브리드)
=========================================
generate_opendrive_world()로 xodr 기반 도로+신호등 자동 생성
+ 한국 도로 규격 차선/정지선/횡단보도를 debug.draw로 표시

결과:
  ✅ 신호등 완전 제어 (CARLA 자동 생성)
  ✅ world.get_map() 정상 작동
  ✅ waypoint 기반 스폰 가능
  ✅ 한국 규격 차선 표시 (발광이지만 정교함)
"""

import sys
import os
import glob
import math
import time

# ── CARLA .egg 자동 탐색 ──
def _find_carla_egg():
    sd = os.path.dirname(os.path.abspath(__file__))
    roots = [sd, os.path.join(sd, ".."),
             os.path.join(sd, "..", "carla", "dist"),
             os.path.join(sd, "..", "..", "PythonAPI", "carla", "dist")]
    cr = os.environ.get("CARLA_ROOT", "")
    if cr:
        roots.append(os.path.join(cr, "PythonAPI", "carla", "dist"))
    for r in roots:
        for pattern in [f"carla-*py{sys.version_info.major}.{sys.version_info.minor}*.egg",
                        "carla-*.egg"]:
            eggs = glob.glob(os.path.join(r, pattern))
            if eggs:
                if eggs[0] not in sys.path:
                    sys.path.insert(0, eggs[0])
                return True
    return False

_find_carla_egg()
import carla

# ══════════════════════════════════════════════════════
# 설정
# ══════════════════════════════════════════════════════

SPECS = {
    12: {"name": "12m_2lane", "n": 1, "lw": 3.25, "mw": 0.0,
         "med": False, "sb": 2.0, "cw": 4.0, "spd": 50,
         "green": 25, "yellow": 3},
    20: {"name": "20m_4lane", "n": 2, "lw": 3.25, "mw": 1.0,
         "med": True, "sb": 2.0, "cw": 4.0, "spd": 60,
         "green": 35, "yellow": 4},
    30: {"name": "30m_6lane", "n": 3, "lw": 3.25, "mw": 2.0,
         "med": True, "sb": 2.0, "cw": 5.0, "spd": 60,
         "green": 45, "yellow": 4},
    40: {"name": "40m_8lane", "n": 4, "lw": 3.5, "mw": 3.0,
         "med": True, "sb": 2.5, "cw": 5.0, "spd": 70,
         "green": 50, "yellow": 5},
}

APPROACH_LEN = 200.0
THICKNESS = 0.01

# 색상
C_YELLOW = carla.Color(255, 200, 0)
C_WHITE = carla.Color(240, 240, 240)
C_STOP = carla.Color(255, 255, 255)
C_CW = carla.Color(230, 230, 230)
Z = 0.02  # 차선 높이


class ChaseCamera:
    """차량 추적 카메라"""
    def __init__(self, world, vehicle=None, mode="third_person"):
        self.world = world
        self.vehicle = vehicle
        self.mode = mode

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def update(self):
        if self.vehicle is None or not self.vehicle.is_alive:
            return
        t = self.vehicle.get_transform()
        fwd = t.get_forward_vector()
        s = self.world.get_spectator()
        if self.mode == "third_person":
            s.set_transform(carla.Transform(
                carla.Location(
                    x=t.location.x - fwd.x * 15,
                    y=t.location.y - fwd.y * 15,
                    z=t.location.z + 8),
                carla.Rotation(pitch=-20, yaw=t.rotation.yaw)))
        else:
            s.set_transform(carla.Transform(
                carla.Location(x=t.location.x, y=t.location.y,
                               z=t.location.z + 50),
                carla.Rotation(pitch=-90)))


class DilemmaZoneRoadBuilder:
    """
    하이브리드 방식:
    1) generate_opendrive_world() → 신호등 + waypoint 자동 생성
    2) debug.draw → 한국 규격 도로 표시 (차선/정지선/횡단보도)
    """

    def __init__(self, host="localhost", port=2000,
                 road_width=30, approach="south",
                 xodr_dir="./custom_road"):
        self.host = host
        self.port = port
        self.road_width = road_width
        self.approach = approach
        self.xodr_dir = xodr_dir
        self.client = None
        self.world = None
        self.debug = None
        self.spawned_actors = []
        self.selected_intersection = {}

    # ==============================================================
    # 맵 로드 (generate_opendrive_world)
    # ==============================================================
    def load_custom_map(self):
        sp = SPECS[self.road_width]
        xodr_name = f"Korean_W{self.road_width}m.xodr"
        xodr_path = os.path.join(self.xodr_dir, xodr_name)

        if not os.path.exists(xodr_path):
            raise FileNotFoundError(f"❌ {xodr_path} 없음")

        print(f"[Builder] CARLA 연결: {self.host}:{self.port}")
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(20.0)

        print(f"[Builder] xodr 맵 로드: {self.road_width}m 교차로")
        with open(xodr_path, "r", encoding="utf-8") as f:
            xodr = f.read()

        params = carla.OpendriveGenerationParameters(
            vertex_distance=2.0, max_road_length=200.0,
            wall_height=0.0, additional_width=0.6,
            smooth_junctions=True, enable_mesh_visibility=True,
            enable_pedestrian_navigation=True)

        self.world = self.client.generate_opendrive_world(xodr, params)
        self.debug = self.world.debug

        # 동기 모드
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)
        self.world.tick()
        time.sleep(1.0)

        # 날씨
        self.setup_weather("dry")

        # 한국 규격 도로 표시 그리기
        try:
            self._draw_road_markings()
        except Exception as e:
            print(f"[Builder] ⚠️ 도로 표시 오류: {e}")
            import traceback
            traceback.print_exc()

        # 신호등 탐색
        self._find_traffic_light()

        print(f"[Builder] ✅ 완료")
        return self.world

    # ==============================================================
    # 한국 도로 표시 (debug.draw)
    # ==============================================================
    def _draw_road_markings(self):
        sp = SPECS[self.road_width]
        n, lw, mw = sp["n"], sp["lw"], sp["mw"]
        half_m = mw / 2 if sp["med"] else 0
        half_road = n * lw + half_m
        junc = half_road + 2
        T = THICKNESS

        print(f"[Builder] 🎨 도로 표시 생성 중...")

        segs = [(-APPROACH_LEN, -junc), (junc, APPROACH_LEN)]
        solid_zone = 30.0

        # ① 황색 중앙선 이중실선
        gap = 0.15
        tc = T + 0.008
        for off in [-gap/2, gap/2]:
            for s, e in segs:
                self._solid("y", off, s, e, tc, C_YELLOW)
                self._solid("x", off, s, e, tc, C_YELLOW)

        # ② 차선 (접근부 실선 + 원거리 점선)
        if n > 1:
            tl = T + 0.005
            for li in range(1, n):
                off = half_m + li * lw
                for sign in [-1, 1]:
                    actual = sign * off
                    for s, e in segs:
                        if s < 0:
                            ss, se = s, min(s + solid_zone, e)
                            ds, de = se, e
                        else:
                            ds, de = s, max(e - solid_zone, s)
                            ss, se = de, e
                        self._solid("y", actual, ss, se, tl, C_WHITE)
                        self._solid("x", actual, ss, se, tl, C_WHITE)
                        self._dashed("y", actual, ds, de, tl, C_WHITE)
                        self._dashed("x", actual, ds, de, tl, C_WHITE)

        # ③ 갓길선 (실선)
        te = T + 0.003
        for sign in [-1, 1]:
            edge = sign * half_road
            for s, e in segs:
                self._solid("y", edge, s, e, te, C_WHITE)
                self._solid("x", edge, s, e, te, C_WHITE)

        # ④ 횡단보도 (먼저 그림 - 교차로에 가까움)
        # 정지선은 횡단보도 + sb(정지선 거리) 더 바깥쪽
        cw = sp["cw"]
        sb = sp["sb"]

        # 횡단보도 중심 = 교차로 끝(junc)에서 cw/2 만큼 바깥
        cw_offset = junc + cw / 2
        for sign in [-1, 1]:
            center = sign * cw_offset
            self._crosswalk("y", center, -half_road, half_road, cw, C_CW)
            self._crosswalk("x", center, -half_road, half_road, cw, C_CW)

        # ⑤ 정지선 (횡단보도 바깥쪽 = 차량 진행방향 반대쪽)
        # 정지선 위치 = junc + cw + sb (교차로에서부터 횡단보도 너머)
        stopline_offset = junc + cw + sb
        for sign in [-1, 1]:
            pos = sign * stopline_offset
            self._stopline("y", pos, -half_road, half_road, C_STOP)
            self._stopline("x", pos, -half_road, half_road, C_STOP)

        print(f"[Builder] 🎨 정지선 위치: ±{stopline_offset:.1f}m (교차로끝에서 {cw+sb:.1f}m 바깥)")

        print(f"[Builder] 🎨 도로 표시 완료 ✅")

    def _solid(self, axis, off, start, end, thickness, color):
        """실선 그리기"""
        step = 1.0
        pos = start
        while pos < end:
            nxt = min(pos + step, end)
            if axis == "y":
                self.debug.draw_line(
                    carla.Location(x=off, y=pos, z=Z),
                    carla.Location(x=off, y=nxt, z=Z),
                    thickness, color, -1.0)
            else:
                self.debug.draw_line(
                    carla.Location(x=pos, y=off, z=Z),
                    carla.Location(x=nxt, y=off, z=Z),
                    thickness, color, -1.0)
            pos += step

    def _dashed(self, axis, off, start, end, thickness, color,
                dl=3.0, gl=5.0):
        """점선 그리기"""
        pos = start
        while pos < end:
            nxt = min(pos + dl, end)
            if axis == "y":
                self.debug.draw_line(
                    carla.Location(x=off, y=pos, z=Z),
                    carla.Location(x=off, y=nxt, z=Z),
                    thickness, color, -1.0)
            else:
                self.debug.draw_line(
                    carla.Location(x=pos, y=off, z=Z),
                    carla.Location(x=nxt, y=off, z=Z),
                    thickness, color, -1.0)
            pos += dl + gl

    def _stopline(self, axis, pos, lane_start, lane_end, color):
        """정지선 (넓은 선)"""
        width = 0.45
        steps = 10
        for i in range(steps + 1):
            t = lane_start + (lane_end - lane_start) * i / steps
            if axis == "y":
                self.debug.draw_line(
                    carla.Location(x=t, y=pos - width/2, z=Z + 0.005),
                    carla.Location(x=t, y=pos + width/2, z=Z + 0.005),
                    0.04, color, -1.0)
            else:
                self.debug.draw_line(
                    carla.Location(x=pos - width/2, y=t, z=Z + 0.005),
                    carla.Location(x=pos + width/2, y=t, z=Z + 0.005),
                    0.04, color, -1.0)

    def _crosswalk(self, axis, center, lane_start, lane_end, cw_width, color):
        """횡단보도 (지브라)"""
        stripe_w = 0.45
        gap_w = 0.45
        half_cw = cw_width / 2
        pos = lane_start
        while pos < lane_end:
            nxt = min(pos + stripe_w, lane_end)
            if axis == "y":
                self.debug.draw_line(
                    carla.Location(x=pos, y=center - half_cw, z=Z + 0.006),
                    carla.Location(x=pos, y=center + half_cw, z=Z + 0.006),
                    0.04, color, -1.0)
                self.debug.draw_line(
                    carla.Location(x=nxt, y=center - half_cw, z=Z + 0.006),
                    carla.Location(x=nxt, y=center + half_cw, z=Z + 0.006),
                    0.04, color, -1.0)
            else:
                self.debug.draw_line(
                    carla.Location(x=center - half_cw, y=pos, z=Z + 0.006),
                    carla.Location(x=center + half_cw, y=pos, z=Z + 0.006),
                    0.04, color, -1.0)
                self.debug.draw_line(
                    carla.Location(x=center - half_cw, y=nxt, z=Z + 0.006),
                    carla.Location(x=center + half_cw, y=nxt, z=Z + 0.006),
                    0.04, color, -1.0)
            pos += stripe_w + gap_w

    # ==============================================================
    # 신호등
    # ==============================================================
    def _find_traffic_light(self):
        tls = self.world.get_actors().filter("*traffic_light*")

        sp = SPECS[self.road_width]
        for tl in tls:
            try:
                tl.set_green_time(sp["green"])
                tl.set_yellow_time(sp["yellow"])
                tl.set_red_time(sp["green"] + sp["yellow"])
            except Exception:
                pass

        stopline = self._get_stopline_location()
        nearest, min_d = None, float("inf")
        for tl in tls:
            d = tl.get_location().distance(stopline)
            if d < min_d:
                min_d = d
                nearest = tl

        if nearest:
            self.selected_intersection["traffic_light_id"] = nearest.id
            print(f"[Builder] 🚦 신호등: id={nearest.id} (거리 {min_d:.1f}m)")
        else:
            self.selected_intersection["traffic_light_id"] = -1
            print(f"[Builder] 🚦 신호등 {len(list(tls))}개 발견 (가까운 것 없음)")

    # ==============================================================
    # 좌표
    # ==============================================================
    def _get_stopline_location(self):
        """
        정지선 실제 위치 반환 (차량이 멈춰야 할 위치).

        도로 구조 (south 접근 기준):
          교차로 → 횡단보도 → 정지선 → 차량 (음의 y 방향)

          y = -half_road      : 차로 끝 (도로 표면)
          y = -junc           : 교차로 경계 (junc = half_road + 2)
          y = -(junc + cw)    : 횡단보도 너머
          y = -(junc + cw+sb) : 정지선 ← 여기서 차량 정차
        """
        sp = SPECS[self.road_width]
        n, lw, mw = sp["n"], sp["lw"], sp["mw"]
        half_m = mw / 2 if sp["med"] else 0
        half_road = n * lw + half_m
        junc = half_road + 2
        # 정지선 = 교차로끝(junc) + 횡단보도(cw) + 정지거리(sb) 만큼 바깥
        stopline_dist = junc + sp["cw"] + sp["sb"]
        lane_x = -(half_m + lw / 2)

        return {
            "south": carla.Location(x=lane_x, y=-stopline_dist, z=0.3),
            "north": carla.Location(x=-lane_x, y=stopline_dist, z=0.3),
            "west": carla.Location(x=-stopline_dist, y=-lane_x, z=0.3),
            "east": carla.Location(x=stopline_dist, y=lane_x, z=0.3),
        }[self.approach]

    def get_experiment_points(self):
        """
        실험에 필요한 모든 좌표/거리 정보 반환.

        - stopline_location: 정지선 위치 (차량이 멈춰야 할 곳)
        - approach_direction: 접근 방향 (yaw)
        - cross_distance: 정지선 → 교차로 통과 완료까지 거리
                         (= cw + 2 + W = 횡단보도+여유+교차로폭)
                         알고리즘에서 t_cross = cross_distance / v 계산 시 사용
        - intersection_width: 실제 교차로 폭 (교차로 양 끝 간 거리)
        """
        sp = SPECS[self.road_width]
        n, lw, mw = sp["n"], sp["lw"], sp["mw"]
        half_m = mw / 2 if sp["med"] else 0
        half_road = n * lw + half_m
        junc = half_road + 2  # 교차로 한쪽 끝 (정지선 방향 기준)

        # 교차로 통과에 필요한 실제 거리:
        # 정지선 → 횡단보도(cw) → 여유(2m) → 교차로 폭(2 * half_road) → 반대편 횡단보도까지
        # 단순화: 정지선부터 교차로 반대편 끝까지
        cross_distance = sp["cw"] + sp["sb"] + 2 * half_road + sp["sb"] + sp["cw"]

        yaw_map = {
            "south": 90.0, "north": -90.0,
            "west": 0.0, "east": 180.0,
        }
        return {
            "stopline_location": self._get_stopline_location(),
            "approach_direction": carla.Rotation(yaw=yaw_map[self.approach]),
            "cross_distance": cross_distance,
            "intersection_width": 2 * half_road,
            "junc_offset": junc,
            "cw_width": sp["cw"],
            "sb_distance": sp["sb"],
        }

    # ==============================================================
    # 정지선 기반 판정 + 안전 정차 제어
    # ==============================================================
    def distance_to_stopline(self, vehicle):
        """
        차량이 정지선까지 진행 방향으로 남은 거리 (m).

        ★ 진행 방향 결정 우선순위:
          1. 속도 벡터 (실제 움직이는 방향) — 가장 정확
          2. forward 벡터 (속도가 거의 0일 때)

        - 정지선 못 미침 → 양수 (멈춰야 함)
        - 정지선 통과    → 음수
        """
        stopline = self._get_stopline_location()
        v_tf = vehicle.get_transform()
        v_loc = v_tf.location

        # 속도 벡터 (실제 진행 방향)
        vel = vehicle.get_velocity()
        speed = math.sqrt(vel.x ** 2 + vel.y ** 2)

        if speed > 0.5:
            # 움직이고 있으면 속도 방향 사용 (가장 정확)
            fx = vel.x / speed
            fy = vel.y / speed
        else:
            # 거의 정지 상태면 forward 벡터 사용
            fwd = v_tf.get_forward_vector()
            fx, fy = fwd.x, fwd.y

        # 차량 → 정지선 벡터
        dx = stopline.x - v_loc.x
        dy = stopline.y - v_loc.y

        # 진행 방향과의 내적 = 부호 있는 거리
        signed_dist = dx * fx + dy * fy

        return signed_dist

    def can_pass_intersection(self, vehicle, yellow_time_s, vehicle_length=4.7):
        """
        ITE 모델 기반: 황색 시간 안에 교차로를 통과할 수 있는지 판정.

        통과 조건: t_cross = (d + W + L) / v  ≤  Y
          d: 정지선까지 거리
          W: 교차로 통과 거리 (정지선 → 반대편 끝)
          L: 차량 길이
          v: 현재 속도
          Y: 황색 시간

        Returns:
          dict {
            'can_pass': bool,
            'd_to_stopline': 정지선까지 거리,
            't_cross': 통과 소요시간,
            'margin': 시간 여유 (Y - t_cross, 양수면 통과 가능)
          }
        """
        d = self.distance_to_stopline(vehicle)
        W = self.get_experiment_points()["cross_distance"]

        v_vec = vehicle.get_velocity()
        v = math.sqrt(v_vec.x ** 2 + v_vec.y ** 2 + v_vec.z ** 2)

        if v < 0.1:
            return {"can_pass": False, "d_to_stopline": d,
                    "t_cross": float("inf"), "margin": -float("inf")}

        # 이미 정지선을 지났으면 무조건 통과 시도
        if d <= 0:
            t_cross = (W - abs(d) + vehicle_length) / v
        else:
            t_cross = (d + W + vehicle_length) / v

        margin = yellow_time_s - t_cross
        return {
            "can_pass": t_cross <= yellow_time_s,
            "d_to_stopline": d,
            "t_cross": t_cross,
            "margin": margin,
        }

    def safe_stop_control(self, vehicle, a_max=7.0, a_comfort=3.4,
                          stop_offset=1.0, target_speed_kmh=50):
        """
        정지선 코앞(stop_offset만큼 앞)에 정확히 정차시키는 제어.

        ★ 안전 마진 정밀 조정 (정지선 1m 앞 정차 목표):
          - 풀 브레이크 거리: 0.5m (이전 1.5m → 너무 일찍 멈춤)
          - 감속 안전 마진: 5% (이전 15% → 너무 강함)
          - 최소 브레이크: 0.05 (이전 0.2 → 너무 강함)
        """
        d = self.distance_to_stopline(vehicle) - stop_offset
        v_vec = vehicle.get_velocity()
        v = math.sqrt(v_vec.x ** 2 + v_vec.y ** 2)
        target_v = target_speed_kmh / 3.6

        # ─── 1) 정지선 통과 → 핸드브레이크 ───
        if d <= 0.0:
            return carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0,
                                        hand_brake=True)

        # ─── 2) 정지선 매우 가까이 → 풀 브레이크 ───
        # 0.5m 이내일 때만 풀브레이크 (이전 1.5m는 너무 일찍 멈춤)
        if d < 0.5:
            return carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0,
                                        hand_brake=(v < 0.5))

        # ─── 3) [예외] 멀리서 멈춰버린 경우 → 살짝 굴러감 ───
        if v < 0.3 and d > 3.0:
            return carla.VehicleControl(throttle=0.15, brake=0.0, steer=0.0)

        # ─── 4) 감속 시작 거리 (마진 1.2배 + 1m) ───
        d_brake_start = (v ** 2) / (2 * a_comfort) * 1.2 + 1.0

        # ─── 5) 충분히 멀면 등속 유지 ───
        if d > d_brake_start:
            if v < target_v - 1.0:
                return carla.VehicleControl(throttle=0.5, brake=0.0, steer=0.0)
            elif v > target_v + 1.0:
                return carla.VehicleControl(throttle=0.0, brake=0.2, steer=0.0)
            else:
                return carla.VehicleControl(throttle=0.3, brake=0.0, steer=0.0)

        # ─── 6) 감속 구간 - 물리식 기반 ───
        # 필요 감속도: a = v² / 2d, 안전 마진 5% (CARLA 응답 보정 수준)
        required_a = (v ** 2) / (2.0 * d) * 1.05

        # 감속도 → 브레이크 매핑 (CARLA: brake=1.0 ≈ a_max)
        brake = required_a / a_max

        # 클램핑 (최소값 매우 작게 → 멀면 거의 안 밟음)
        if brake >= 1.0:
            brake = 1.0
        elif brake < 0.05:
            brake = 0.05

        return carla.VehicleControl(throttle=0.0, brake=float(brake),
                                    steer=0.0, hand_brake=False)

    def setup_weather(self, condition="dry"):
        if condition == "dry":
            w = carla.WeatherParameters(
                cloudiness=15.0, precipitation=0.0,
                precipitation_deposits=0.0, wetness=0.0,
                sun_azimuth_angle=160.0, sun_altitude_angle=65.0,
                fog_density=0.0)
        else:
            w = carla.WeatherParameters(
                cloudiness=80.0, precipitation=30.0,
                precipitation_deposits=50.0, wetness=50.0,
                sun_altitude_angle=40.0)
        self.world.set_weather(w)

    def cleanup(self):
        for a in self.spawned_actors:
            if a.is_alive:
                a.destroy()
        self.spawned_actors.clear()

    def disconnect(self):
        self.cleanup()
        if self.world:
            s = self.world.get_settings()
            s.synchronous_mode = False
            self.world.apply_settings(s)
        print("[Builder] 정리 완료")


# ══════════════════════════════════════════════════════
# 독립 실행 테스트
# ══════════════════════════════════════════════════════
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=30,
                        choices=[12, 20, 30, 40])
    parser.add_argument("--approach", type=str, default="south",
                        choices=["south", "north", "west", "east"])
    parser.add_argument("--xodr-dir", type=str, default="./custom_road")
    args = parser.parse_args()

    builder = DilemmaZoneRoadBuilder(
        road_width=args.width, approach=args.approach,
        xodr_dir=args.xodr_dir)
    try:
        world = builder.load_custom_map()
        points = builder.get_experiment_points()
        print(f"\n정지선 위치: {points['stopline_location']}")
        print(f"접근 방향: yaw={points['approach_direction'].yaw}°")

        # 카메라를 정지선 상공으로
        sl = points["stopline_location"]
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(
            carla.Location(x=sl.x, y=sl.y, z=sl.z + 80),
            carla.Rotation(pitch=-90)))
        world.tick()

        # 신호등 상태 확인
        tls = list(world.get_actors().filter("traffic.traffic_light*"))
        print(f"\n🚦 신호등 {len(tls)}개:")
        for tl in tls:
            loc = tl.get_location()
            state = tl.get_state()
            print(f"   id={tl.id} 위치=({loc.x:.1f},{loc.y:.1f}) 상태={state}")

        print(f"\n[테스트] 맵 로드 성공! Ctrl+C 로 종료.")
        while True:
            world.tick()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n사용자 종료")
    finally:
        builder.disconnect()
