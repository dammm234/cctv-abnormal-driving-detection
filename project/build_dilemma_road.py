#!/usr/bin/env python3
# CARLA xodr 맵 로드하고 한국식 차선/정지선/횡단보도 debug.draw 로 그려줌.
# 신호등은 generate_opendrive_world 가 알아서 박아주니까 따로 안 만든다.

import sys
import os
import glob
import math
import time


def _find_carla_egg():
    sd = os.path.dirname(os.path.abspath(__file__))
    roots = [
        sd,
        os.path.join(sd, ".."),
        os.path.join(sd, "..", "carla", "dist"),
        os.path.join(sd, "..", "..", "PythonAPI", "carla", "dist"),
    ]
    cr = os.environ.get("CARLA_ROOT", "")
    if cr:
        roots.append(os.path.join(cr, "PythonAPI", "carla", "dist"))

    py_pattern = f"carla-*py{sys.version_info.major}.{sys.version_info.minor}*.egg"
    for r in roots:
        for pattern in (py_pattern, "carla-*.egg"):
            eggs = glob.glob(os.path.join(r, pattern))
            if eggs:
                if eggs[0] not in sys.path:
                    sys.path.insert(0, eggs[0])
                return True
    return False


_find_carla_egg()
import carla


# 도로 폭별 스펙 (n=편도 차로 수, lw=차로폭, mw=중앙분리대 폭, sb=정지거리, cw=횡단보도 폭)
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
Z = 0.02

C_YELLOW = carla.Color(255, 200, 0)
C_WHITE = carla.Color(240, 240, 240)
C_STOP = carla.Color(255, 255, 255)
C_CW = carla.Color(230, 230, 230)


class DilemmaZoneRoadBuilder:

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

    def load_custom_map(self):
        xodr_name = f"Korean_W{self.road_width}m.xodr"
        xodr_path = os.path.join(self.xodr_dir, xodr_name)
        if not os.path.exists(xodr_path):
            raise FileNotFoundError(xodr_path)

        print(f"[Builder] connecting {self.host}:{self.port}")
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(20.0)

        with open(xodr_path, "r", encoding="utf-8") as f:
            xodr = f.read()

        params = carla.OpendriveGenerationParameters(
            vertex_distance=2.0,
            max_road_length=200.0,
            wall_height=0.0,
            additional_width=0.6,
            smooth_junctions=True,
            enable_mesh_visibility=True,
            enable_pedestrian_navigation=True,
        )
        self.world = self.client.generate_opendrive_world(xodr, params)
        self.debug = self.world.debug

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)
        self.world.tick()
        time.sleep(1.0)

        self.setup_weather("dry")

        try:
            self._draw_road_markings()
        except Exception as e:
            print(f"[Builder] draw error: {e}")
            import traceback
            traceback.print_exc()

        self._find_traffic_light()
        print(f"[Builder] map loaded ({self.road_width}m)")
        return self.world

    # ----- 도로 표시 -----

    def _draw_road_markings(self):
        sp = SPECS[self.road_width]
        n, lw, mw = sp["n"], sp["lw"], sp["mw"]
        half_m = mw / 2 if sp["med"] else 0
        half_road = n * lw + half_m
        junc = half_road + 2
        T = THICKNESS

        segs = [(-APPROACH_LEN, -junc), (junc, APPROACH_LEN)]
        solid_zone = 30.0

        # 중앙선 이중실선 (황색)
        gap = 0.15
        tc = T + 0.008
        for off in (-gap / 2, gap / 2):
            for s, e in segs:
                self._solid("y", off, s, e, tc, C_YELLOW)
                self._solid("x", off, s, e, tc, C_YELLOW)

        # 차로 구분선: 교차로 근처는 실선, 멀어지면 점선
        if n > 1:
            tl = T + 0.005
            for li in range(1, n):
                off = half_m + li * lw
                for sign in (-1, 1):
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

        # 갓길선
        te = T + 0.003
        for sign in (-1, 1):
            edge = sign * half_road
            for s, e in segs:
                self._solid("y", edge, s, e, te, C_WHITE)
                self._solid("x", edge, s, e, te, C_WHITE)

        # 횡단보도 - 교차로 끝에서 cw/2 만큼 바깥
        cw = sp["cw"]
        sb = sp["sb"]
        cw_offset = junc + cw / 2
        for sign in (-1, 1):
            center = sign * cw_offset
            self._crosswalk("y", center, -half_road, half_road, cw, C_CW)
            self._crosswalk("x", center, -half_road, half_road, cw, C_CW)

        # 정지선 - 횡단보도 더 바깥쪽
        stopline_offset = junc + cw + sb
        for sign in (-1, 1):
            pos = sign * stopline_offset
            self._stopline("y", pos, -half_road, half_road, C_STOP)
            self._stopline("x", pos, -half_road, half_road, C_STOP)

        print(f"[Builder] stopline at +-{stopline_offset:.1f}m")

    def _solid(self, axis, off, start, end, thickness, color):
        step = 1.0
        pos = start
        while pos < end:
            nxt = min(pos + step, end)
            if axis == "y":
                a = carla.Location(x=off, y=pos, z=Z)
                b = carla.Location(x=off, y=nxt, z=Z)
            else:
                a = carla.Location(x=pos, y=off, z=Z)
                b = carla.Location(x=nxt, y=off, z=Z)
            self.debug.draw_line(a, b, thickness, color, -1.0)
            pos += step

    def _dashed(self, axis, off, start, end, thickness, color, dl=3.0, gl=5.0):
        pos = start
        while pos < end:
            nxt = min(pos + dl, end)
            if axis == "y":
                a = carla.Location(x=off, y=pos, z=Z)
                b = carla.Location(x=off, y=nxt, z=Z)
            else:
                a = carla.Location(x=pos, y=off, z=Z)
                b = carla.Location(x=nxt, y=off, z=Z)
            self.debug.draw_line(a, b, thickness, color, -1.0)
            pos += dl + gl

    def _stopline(self, axis, pos, lane_start, lane_end, color):
        width = 0.45
        steps = 10
        for i in range(steps + 1):
            t = lane_start + (lane_end - lane_start) * i / steps
            if axis == "y":
                a = carla.Location(x=t, y=pos - width / 2, z=Z + 0.005)
                b = carla.Location(x=t, y=pos + width / 2, z=Z + 0.005)
            else:
                a = carla.Location(x=pos - width / 2, y=t, z=Z + 0.005)
                b = carla.Location(x=pos + width / 2, y=t, z=Z + 0.005)
            self.debug.draw_line(a, b, 0.04, color, -1.0)

    def _crosswalk(self, axis, center, lane_start, lane_end, cw_width, color):
        # 지브라 패턴
        stripe_w = 0.45
        gap_w = 0.45
        half_cw = cw_width / 2
        pos = lane_start
        while pos < lane_end:
            nxt = min(pos + stripe_w, lane_end)
            for p in (pos, nxt):
                if axis == "y":
                    a = carla.Location(x=p, y=center - half_cw, z=Z + 0.006)
                    b = carla.Location(x=p, y=center + half_cw, z=Z + 0.006)
                else:
                    a = carla.Location(x=center - half_cw, y=p, z=Z + 0.006)
                    b = carla.Location(x=center + half_cw, y=p, z=Z + 0.006)
                self.debug.draw_line(a, b, 0.04, color, -1.0)
            pos += stripe_w + gap_w

    # ----- 신호등 -----

    def _find_traffic_light(self):
        tls = list(self.world.get_actors().filter("*traffic_light*"))
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
            print(f"[Builder] traffic light id={nearest.id} d={min_d:.1f}m")
        else:
            self.selected_intersection["traffic_light_id"] = -1
            print(f"[Builder] no traffic light (found {len(tls)} total)")

    # ----- 좌표/거리 -----

    def _get_stopline_location(self):
        # south 기준 좌표계: 차량은 -y 쪽에서 +y 방향으로 진입
        # y = -half_road       차로 끝
        # y = -junc            교차로 경계 (junc = half_road + 2)
        # y = -(junc + cw)     횡단보도 너머
        # y = -(junc + cw+sb)  정지선
        sp = SPECS[self.road_width]
        n, lw, mw = sp["n"], sp["lw"], sp["mw"]
        half_m = mw / 2 if sp["med"] else 0
        half_road = n * lw + half_m
        junc = half_road + 2
        stopline_dist = junc + sp["cw"] + sp["sb"]
        lane_x = -(half_m + lw / 2)

        return {
            "south": carla.Location(x=lane_x, y=-stopline_dist, z=0.3),
            "north": carla.Location(x=-lane_x, y=stopline_dist, z=0.3),
            "west": carla.Location(x=-stopline_dist, y=-lane_x, z=0.3),
            "east": carla.Location(x=stopline_dist, y=lane_x, z=0.3),
        }[self.approach]

    def get_experiment_points(self):
        sp = SPECS[self.road_width]
        n, lw, mw = sp["n"], sp["lw"], sp["mw"]
        half_m = mw / 2 if sp["med"] else 0
        half_road = n * lw + half_m
        junc = half_road + 2

        # 정지선 → 반대편 횡단보도까지 (교차로 통과 거리)
        cross_distance = sp["cw"] + sp["sb"] + 2 * half_road + sp["sb"] + sp["cw"]

        yaw_map = {"south": 90.0, "north": -90.0, "west": 0.0, "east": 180.0}

        return {
            "stopline_location": self._get_stopline_location(),
            "approach_direction": carla.Rotation(yaw=yaw_map[self.approach]),
            "cross_distance": cross_distance,
            "intersection_width": 2 * half_road,
            "junc_offset": junc,
            "cw_width": sp["cw"],
            "sb_distance": sp["sb"],
        }

    # ----- 판정/제어 -----

    def distance_to_stopline(self, vehicle):
        # 진행 방향 기준 부호 있는 거리.
        # 양수 = 정지선 못 미침, 음수 = 통과.
        # 멈춰있을 땐 forward 벡터, 움직이면 속도 벡터를 쓴다 (실제 진행방향이 더 정확).
        stopline = self._get_stopline_location()
        v_tf = vehicle.get_transform()
        v_loc = v_tf.location

        vel = vehicle.get_velocity()
        speed = math.sqrt(vel.x ** 2 + vel.y ** 2)

        if speed > 0.5:
            fx = vel.x / speed
            fy = vel.y / speed
        else:
            fwd = v_tf.get_forward_vector()
            fx, fy = fwd.x, fwd.y

        dx = stopline.x - v_loc.x
        dy = stopline.y - v_loc.y
        return dx * fx + dy * fy

    def safe_stop_control(self, vehicle, a_max=7.0, a_comfort=3.4,
                          stop_offset=1.0, target_speed_kmh=50):
        # 정지선 stop_offset m 앞에 정확히 멈추도록 throttle/brake 만든다.
        d = self.distance_to_stopline(vehicle) - stop_offset
        v_vec = vehicle.get_velocity()
        v = math.sqrt(v_vec.x ** 2 + v_vec.y ** 2)
        target_v = target_speed_kmh / 3.6

        # 정지선 이미 통과 → 핸드브레이크
        if d <= 0.0:
            return carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0,
                                        hand_brake=True)

        # 코앞이면 풀브레이크
        if d < 0.5:
            return carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0,
                                        hand_brake=(v < 0.5))

        # 멀리서 멈춰버린 경우 살짝 굴려준다
        if v < 0.3 and d > 3.0:
            return carla.VehicleControl(throttle=0.15, brake=0.0, steer=0.0)

        # 감속 시작점: 편안 감속도 기준 + 마진
        d_brake_start = (v ** 2) / (2 * a_comfort) * 1.2 + 1.0

        # 충분히 멀면 등속 유지
        if d > d_brake_start:
            if v < target_v - 1.0:
                return carla.VehicleControl(throttle=0.5, brake=0.0, steer=0.0)
            if v > target_v + 1.0:
                return carla.VehicleControl(throttle=0.0, brake=0.2, steer=0.0)
            return carla.VehicleControl(throttle=0.3, brake=0.0, steer=0.0)

        # 감속 구간: a = v^2 / 2d
        required_a = (v ** 2) / (2.0 * d) * 1.05
        brake = required_a / a_max
        brake = max(0.05, min(1.0, brake))

        return carla.VehicleControl(throttle=0.0, brake=float(brake),
                                    steer=0.0, hand_brake=False)

    # ----- 환경/정리 -----

    def setup_weather(self, condition="dry"):
        if condition == "dry":
            w = carla.WeatherParameters(
                cloudiness=15.0, precipitation=0.0,
                precipitation_deposits=0.0, wetness=0.0,
                sun_azimuth_angle=160.0, sun_altitude_angle=65.0,
                fog_density=0.0,
            )
        else:
            w = carla.WeatherParameters(
                cloudiness=80.0, precipitation=30.0,
                precipitation_deposits=50.0, wetness=50.0,
                sun_altitude_angle=40.0,
            )
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
        print("[Builder] done")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=30, choices=[12, 20, 30, 40])
    parser.add_argument("--approach", type=str, default="south",
                        choices=["south", "north", "west", "east"])
    parser.add_argument("--xodr-dir", type=str, default="./custom_road")
    args = parser.parse_args()

    builder = DilemmaZoneRoadBuilder(
        road_width=args.width,
        approach=args.approach,
        xodr_dir=args.xodr_dir,
    )
    try:
        world = builder.load_custom_map()
        points = builder.get_experiment_points()
        print(f"stopline: {points['stopline_location']}")
        print(f"approach yaw: {points['approach_direction'].yaw}")

        sl = points["stopline_location"]
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(
            carla.Location(x=sl.x, y=sl.y, z=sl.z + 80),
            carla.Rotation(pitch=-90),
        ))
        world.tick()

        tls = list(world.get_actors().filter("traffic.traffic_light*"))
        print(f"traffic lights: {len(tls)}")
        for tl in tls:
            loc = tl.get_location()
            print(f"  id={tl.id} ({loc.x:.1f},{loc.y:.1f}) state={tl.get_state()}")

        print("running. Ctrl+C to quit.")
        while True:
            world.tick()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        builder.disconnect()
