# CARLA 교차로 충돌위험 예측/대응 알고리즘 시뮬

import sys, os, glob, math, csv, json, time, itertools
from datetime import datetime
from dataclasses import dataclass, asdict
from enum import Enum

def _find_carla_egg():
    sd = os.path.dirname(os.path.abspath(__file__))
    roots = [sd, os.path.join(sd,".."), os.path.join(sd,"..","carla","dist"),
             os.path.join(sd,"..","..","PythonAPI","carla","dist")]
    cr = os.environ.get("CARLA_ROOT","")
    if cr: roots.append(os.path.join(cr,"PythonAPI","carla","dist"))
    for r in roots:
        for p in [f"carla-*py{sys.version_info.major}.{sys.version_info.minor}*.egg","carla-*.egg"]:
            eggs = glob.glob(os.path.join(r, p))
            if eggs:
                if eggs[0] not in sys.path: sys.path.insert(0, eggs[0])
                print(f"[INFO] CARLA egg: {eggs[0]}")
                return True
    return False

_find_carla_egg()
try:
    import carla
    if not hasattr(carla, "Client"):
        raise ImportError("egg 바인딩 필요")
    CARLA_AVAILABLE = True
    print("[INFO] carla OK")
except ImportError as e:
    CARLA_AVAILABLE = False
    print(f"[WARN] carla 없음 ({e})")

class RoadCondition(Enum):
    DRY = "dry"
    WET = "wet"

SPEEDS_KMH = [30, 35, 40, 45, 50, 55, 60]
DISTANCES_M = [10, 20, 30, 40, 50, 60, 70, 80]
YELLOW_TIMES_S = [3.0, 4.0, 5.0]
INTERSECTION_WIDTHS_M = [12, 20, 30, 40]
FIXED_DELTA_SECONDS = 0.05
MAX_SIM_STEPS = 600
OUTPUT_DIR = "results"

FF_BASE = 0.30          # throttle 베이스 (정지마찰 극복)
FF_SLOPE = 0.003        # km/h당 throttle 가산 (공기저항 보정)
P_KP = 0.3            
P_BRAKE_GAIN = 0.2      
P_BRAKE_MAX = 0.5       

SIG_MARGIN_S = 0.5      # 황색 안전 여유 시간
DIST_MARGIN_M = 1.5     # 정지선 앞 안전 여유 거리


COLLISION_GAP_M = 1.5           # 차간 < 이 값이면 추돌
SAFE_STOP_RADIUS_M = 5.0        # 정지선 반경 이내 정차면 안전
STATIONARY_KMH = 1.0            #이 속도 미만이면 정차로 간주

NO_REAR_GAP = 999.0             # 후방차 없음 표시값
NO_REAR_GAP_THRESHOLD = 900.0   # 이보다 작으면 후방차 있음
TRAFFIC_LIGHT_HOLD_S = 999.0    # 신호등 freeze 시간 (사실상 무한)


EXP2_MAX_STEPS = 200
EXP3_MAX_STEPS = 400
EXP3_HARD_BREAK_STEP = 350
WARMUP_STEPS = 20

VEHICLE_SPECS = {
    "sedan":{"name":"승용차","blueprint":"vehicle.tesla.model3","L":4.7,"a_max_dry":7.5,"a_max_wet":4.5,"a_comf_dry":3.4,"a_comf_wet":2.04,"t_r":1.0,"color":"255,255,255"},
    "suv":{"name":"SUV","blueprint":"vehicle.audi.etron","L":4.9,"a_max_dry":6.8,"a_max_wet":4.0,"a_comf_dry":3.0,"a_comf_wet":1.76,"t_r":1.0,"color":"0,100,200"},
    "truck":{"name":"트럭","blueprint":"vehicle.carlamotors.carlacola","L":7.5,"a_max_dry":5.0,"a_max_wet":3.0,"a_comf_dry":2.0,"a_comf_wet":1.2,"t_r":1.2,"color":"200,100,0"},
    "bus":{"name":"버스","blueprint":"vehicle.mitsubishi.fusorosa","L":9.0,"a_max_dry":4.5,"a_max_wet":2.5,"a_comf_dry":1.8,"a_comf_wet":1.0,"t_r":1.3,"color":"100,200,0"},
}

@dataclass
class ScenarioConfig:
    scenario_id: int
    exp: int
    speed_kmh: float
    distance_m: float
    yellow_time_s: float
    intersection_width_m: float
    road_condition: RoadCondition
    ego_type: str = "sedan"
    rear_speed_kmh: float = 0
    gap_m: float = NO_REAR_GAP
    rear_type: str = "sedan"

    @property
    def speed_ms(self) -> float:
        return self.speed_kmh / 3.6

    @property
    def has_rear(self) -> bool:
        return self.gap_m < NO_REAR_GAP_THRESHOLD

    @property
    def L(self) -> float:
        return VEHICLE_SPECS[self.ego_type]["L"]

@dataclass
class ScenarioResult:
    scenario_id: int
    exp: int
    speed_kmh: float
    distance_m: float
    yellow_time_s: float
    intersection_width_m: float
    road_condition: str
    ego_type: str = "sedan"
    rear_speed_kmh: float = 0
    gap_m: float = 0
    rear_type: str = ""

    #알고리즘 이론 판정
    theoretical_pass: bool = False
    can_stop_emg: bool = False
    rear_collision_theory: bool = False
    decision: str = ""
    zone: str = ""

    #실제 시뮬 결과
    pass_before_red: bool = False
    actual_travel_time_s: float = 0
    avg_speed_kmh: float = 0
    final_speed_kmh: float = 0
    time_margin_s: float = 0
    actual_collision: bool = False
    min_gap_m: float = NO_REAR_GAP
    safe_stop: bool = False
    red_entry: bool = False
    success: bool = False
    match_theory: bool = False
    timestamp: str = ""
    error: str = ""

    #딜레마존 분석용
    stop_overrun_m: float = 0       # 음수=정지선 전, 양수=침범
    dist_to_stopline_m: float = 0
    in_crosswalk: bool = False
    in_intersection: bool = False
    past_intersection: bool = False
    is_dilemma_zone: bool = False
    decision_feasible: bool = False

def algorithm_predict(cfg):
    s = VEHICLE_SPECS[cfg.ego_type]
    v = cfg.speed_ms
    d = cfg.distance_m
    W = cfg.intersection_width_m
    Y = cfg.yellow_time_s
    L = s["L"]
    road = cfg.road_condition.value
    a_max = s["a_max_dry"] if road == "dry" else s["a_max_wet"]
    a_comf = s["a_comf_dry"] if road == "dry" else s["a_comf_wet"]
    t_r = s["t_r"]

    # 통과/정지 판정
    t_cross = (d + W + L) / v if v > 0 else 999
    margin = Y - t_cross
    can_pass = (t_cross + SIG_MARGIN_S) <= Y
    d_stop = v * t_r + v ** 2 / (2 * a_max)
    can_stop = d >= (d_stop + DIST_MARGIN_M)

    # 후방
    rear_col = False
    closing = -999
    if cfg.has_rear:
        sr = VEHICLE_SPECS[cfg.rear_type]
        vr = cfg.rear_speed_kmh / 3.6
        ar = sr["a_max_dry"] if road == "dry" else sr["a_max_wet"]
        de = v * t_r + v ** 2 / (2 * a_max)
        dr = vr * (t_r + sr["t_r"]) + vr ** 2 / (2 * ar)
        closing = dr - de - cfg.gap_m
        rear_col = closing > 0
    rs = not rear_col

    # 쾌적감속으로 정지 가능여부
    d_stop_c = v * t_r + v ** 2 / (2 * a_comf)
    can_stop_c = d >= (d_stop_c + DIST_MARGIN_M)

    # 판정
    if can_pass and can_stop and rs:
        dec = "감속" if can_stop_c else "진행"
    elif can_pass and can_stop and not rs:
        dec = "진행"
    elif can_pass and not can_stop:
        dec = "진행"
    elif not can_pass and can_stop and rs:
        dec = "급정지"
    elif not can_pass and can_stop and not rs:
        # 정지가능 + 후방위험 can_pass=False면 진행은 신호위반이라 못함
        dec = "감속" if can_stop_c else "급정지"
    elif not can_pass and not can_stop and rs:
        dec = "급정지"
    else:
        # 딜레마존 + 후방위험
        dec = "감속" if can_stop_c else "급정지"

    zone = "진행/정지선택구간" if can_pass and can_stop else ("딜레마구간" if not can_pass and not can_stop else "일반구간")
    return {
        "can_pass": can_pass, "t_cross": t_cross, "margin": margin,
        "can_stop": can_stop, "d_stop": d_stop,
        "can_stop_comfort": can_stop_c,
        "rear_col": rear_col, "closing": closing,
        "decision": dec, "zone": zone,
    }


def compute_throttle_brake(target_ms, current_ms, target_kmh):
    ff = FF_BASE + FF_SLOPE * target_kmh
    err = target_ms - current_ms
    out = ff + P_KP * err
    if out > 0:
        return min(1, max(0, out)), 0.0
    else:
        return 0.0, min(P_BRAKE_MAX, abs(out) * P_BRAKE_GAIN)

def generate_scenarios(exp, quick=False):
    sc = []

    if quick:
        spds = [40, 50, 60]
        ds = [20, 40, 60]
        ys = [4.0]
        ws = [12, 20, 30, 40]
        ets = ["sedan"]
        rds = [RoadCondition.DRY]
    else:
        spds = SPEEDS_KMH
        ds = DISTANCES_M
        ys = YELLOW_TIMES_S
        ws = sorted(INTERSECTION_WIDTHS_M)
        ets = ["sedan", "suv", "truck", "bus"]
        rds = [RoadCondition.DRY, RoadCondition.WET]

    if exp == 1:
        combos = itertools.product(ws, ets, rds, spds, ds, ys)
        for idx, (w, et, rd, v, d, y) in enumerate(combos):
            sc.append(ScenarioConfig(idx, 1, v, d, y, w, rd, et))

    elif exp == 2:
        rss = [55, 60] if quick else [45, 50, 55, 60, 65]
        gs = [10, 20] if quick else [5, 10, 15, 20, 30]
        rts = ["sedan"] if quick else ["sedan", "truck"]
        combos = itertools.product(ets[:2], rds, gs, rss, rts)
        for idx, (et, rd, g, rs, rt) in enumerate(combos):
            sc.append(ScenarioConfig(idx, 2, 50, 35, 4.0, 20, rd, et, rs, g, rt))

    elif exp == 3:
        ws3 = ws if not quick else [12, 20, 30, 40]
        spds3 = spds if not quick else [40, 50, 60]
        ds3 = ds if not quick else [20, 40, 60]
        idx = 0
        for (w, et, rd, v, d, y) in itertools.product(ws3, ets, rds, spds3, ds3, ys):
            # 후방 없음
            sc.append(ScenarioConfig(idx, 3, v, d, y, w, rd, et))
            idx += 1
            # 후방 있음
            sc.append(ScenarioConfig(idx, 3, v, d, y, w, rd, et,
                                     v + 5, 15, "sedan"))
            idx += 1

    return sc


def load_scenarios_from_json(json_path):

    if not os.path.exists(json_path):
        print(f"[ERROR] 시나리오 파일이 없습니다: {json_path}")
        return []
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    sc = []
    data_sorted = sorted(data, key=lambda s: (s['intersection_width_m'], s['scenario_id']))

    for new_idx, s in enumerate(data_sorted):
        road_cond = RoadCondition.DRY if s['road_condition'] == 'dry' else RoadCondition.WET
        rear_speed = s.get('rear_speed_kmh', 0)
        gap = s.get('gap_m', 999)
        rear_type = s.get('rear_type', 'sedan') if rear_speed > 0 else 'sedan'

        sc.append(ScenarioConfig(
            new_idx,                            
            3,                                 
            float(s['speed_kmh']),
            float(s['distance_m']),
            float(s['yellow_time_s']),
            float(s['intersection_width_m']),
            road_cond,
            s.get('ego_type', 'sedan'),
            float(rear_speed),
            float(gap),
            rear_type
        ))

    print(f"[INFO] {len(sc)}건 로드")
    return sc


class CollisionRiskSimulator:
    def __init__(self):
        self.world = None
        self.client = None
        self.stopline_location = None
        self.move_dir = None
        self.approach_wps = []
        self.current_width = None
        self.spawned = []
        self.use_builder = False
        self.builder = None
        self.selected_tl_id = -1
        # hud용 시나리오 정보
        self._cur_scenario_idx = 0
        self._total_scenarios = 0

    def setup(self):
        try:
            from build_dilemma_road import DilemmaZoneRoadBuilder
            self.builder = DilemmaZoneRoadBuilder()
            self.use_builder = True
        except ImportError:
            self.use_builder = False
        if not self.use_builder:
            self.client = carla.Client("localhost", 2000)
            self.client.set_timeout(60.0)

    def load_intersection(self, width_m):
        if width_m == self.current_width:
            return
        if self.use_builder:
            if self.builder:
                self.builder.cleanup()
            self.builder.road_width = width_m
            self.world = self.builder.load_custom_map()
            pts = self.builder.get_experiment_points()
            self.stopline_location = pts["stopline_location"]
            self.selected_tl_id = self.builder.selected_intersection.get(
                "traffic_light_id", -1)
        else:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            xp = os.path.join(script_dir, "custom_road", f"Korean_W{width_m}m.xodr")

            if not os.path.exists(xp):
                print(f"[ERROR] 맵 파일이 없습니다: {xp}")
                print(f"        custom_road 폴더에 Korean_W{width_m}m.xodr 파일을 두세요.")
                return

            xodr = None
            for enc in ["utf-8", "cp949", "latin-1"]:
                try:
                    with open(xp, "r", encoding=enc) as f:
                        xodr = f.read()
                    break
                except (UnicodeDecodeError, OSError):
                    continue
            if xodr is None:
                with open(xp, "rb") as f:
                    xodr = f.read().decode("utf-8", errors="ignore")
            self.world = self.client.generate_opendrive_world(
                xodr,
                carla.OpendriveGenerationParameters(
                    vertex_distance=2.0, max_road_length=500.0,
                    wall_height=0.0, additional_width=0.8,
                    smooth_junctions=True, enable_mesh_visibility=True))
            time.sleep(3.0)
            s = self.world.get_settings()
            s.synchronous_mode = True
            s.fixed_delta_seconds = FIXED_DELTA_SECONDS
            self.world.apply_settings(s)
            self.world.tick()
            wps = self.world.get_map().generate_waypoints(2.0)
            self.approach_wps = sorted(
                [w for w in wps if not w.is_junction and w.lane_id < 0 and w.transform.location.y < 0],
                key=lambda w: w.transform.location.y)
            if self.approach_wps:
                self.stopline_location = self.approach_wps[-1].transform.location
        self.current_width = width_m

        if self.world:
            for _ in range(50):
                try:
                    self.world.tick()
                except Exception:
                    break

    def _spawn(self, dist_m, spd_kmh, vtype="sedan", role="ego"):
        spec = VEHICLE_SPECS[vtype]
        stop = self.stopline_location
        if self.use_builder:
            cm = self.world.get_map()
            wp = cm.get_waypoint(stop, project_to_road=True)
            rem = dist_m
            while rem > 0:
                prevs = wp.previous(min(5.0, rem))
                if not prevs:
                    break
                wp = prevs[0]
                rem -= 5.0
            tf = wp.transform
            tf.location.z += 0.5
        else:
            bw = min(self.approach_wps,
                     key=lambda w: abs(stop.distance(w.transform.location) - dist_m))
            tf = bw.transform
            tf.location.z += 1.0
        bp_lib = self.world.get_blueprint_library()
        bpl = bp_lib.filter(spec["blueprint"])
        if not bpl:
            bpl = bp_lib.filter("vehicle.tesla.model3")
        if not bpl:
            bpl = bp_lib.filter("vehicle.*")
        vbp = bpl[0]
        vbp.set_attribute("role_name", role)
        if vbp.has_attribute("color"):
            vbp.set_attribute("color", spec["color"])
        vehicle = None
        for _ in range(5):
            try:
                vehicle = self.world.spawn_actor(vbp, tf)
                break
            except RuntimeError:
                tf.location.z += 0.5
                if not self.use_builder and self.approach_wps:
                    dist_m += 3
                    bw = min(self.approach_wps,
                             key=lambda w: abs(stop.distance(w.transform.location) - dist_m))
                    tf = bw.transform
                    tf.location.z += 1.0
        if not vehicle:
            return None
        self.spawned.append(vehicle)
        if self.use_builder and self.builder:
            try:
                self.builder.spawned_actors.append(vehicle)
            except Exception:
                pass
        self.world.tick()

        if self.use_builder:
            fwd = vehicle.get_transform().get_forward_vector()
            self.move_dir = (fwd.x, fwd.y)
        else:
            loc = vehicle.get_location()
            dx = stop.x - loc.x
            dy = stop.y - loc.y
            d = max(0.1, math.hypot(dx, dy))
            self.move_dir = (dx / d, dy / d)

        ms = spd_kmh / 3.6
        for _ in range(5):
            vehicle.set_target_velocity(carla.Vector3D(x=0, y=0, z=0))
            self.world.tick()

        last_z = vehicle.get_location().z
        for _ in range(15):
            vehicle.set_target_velocity(carla.Vector3D(x=0, y=0, z=0))
            self.world.tick()
            cur_z = vehicle.get_location().z
            if abs(cur_z - last_z) < 0.003:
                break
            last_z = cur_z

        vehicle.set_target_velocity(carla.Vector3D(
            x=self.move_dir[0] * ms, y=self.move_dir[1] * ms, z=0))
        self.world.tick()

        return vehicle

    def _cleanup(self):
        if self.spawned:
            batch_ok = False
            if self.client is not None:
                try:
                    cmds = [carla.command.DestroyActor(a.id) for a in self.spawned]
                    self.client.apply_batch_sync(cmds, True)
                    batch_ok = True
                except Exception:
                    batch_ok = False
            if not batch_ok:
                for a in self.spawned:
                    try:
                        a.destroy()
                    except Exception:
                        pass
            self.spawned.clear()
        if self.use_builder and self.builder:
            try:
                self.builder.cleanup()
            except Exception:
                pass
        if self.world:
            try:
                for _ in range(5):
                    self.world.tick()
            except Exception:
                pass

    def _spd(self, v):
        vel = v.get_velocity()
        return math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)

    def _spd_kmh(self, v):
        return self._spd(v) * 3.6

    def _set_vel(self, v, kmh):
        if not self.move_dir:
            return
        ms = kmh / 3.6
        v.set_target_velocity(carla.Vector3D(
            x=self.move_dir[0] * ms, y=self.move_dir[1] * ms, z=0))

    def _brake(self, v):
        v.set_target_velocity(carla.Vector3D(x=0, y=0, z=0))
        v.apply_control(carla.VehicleControl(throttle=0, brake=1.0, steer=0))

    def _cdist(self, a, b):
        return math.hypot(a.x - b.x, a.y - b.y)

    def _draw_hud(self, ego, cfg, elapsed, decision, scenario_idx=0, total=0):
        # 정지선 위 공중에 hud 띄움
        if not self.world or not ego or not self.stopline_location:
            return
        try:
            v_kmh = self._spd_kmh(ego)
            if self.use_builder and self.builder:
                d = self.builder.distance_to_stopline(ego)
            else:
                d = self.stopline_location.distance(ego.get_location())

            yellow_left = max(0, cfg.yellow_time_s - elapsed)
            if elapsed < cfg.yellow_time_s:
                signal = f"YELLOW {yellow_left:.1f}s"
                signal_color = carla.Color(255, 200, 0)
            else:
                signal = "RED"
                signal_color = carla.Color(255, 50, 50)

            dec_map = {"진행": "GO", "급정지": "STOP", "감속": "SLOW"}
            dec_en = dec_map.get(decision, str(decision))
            road_str = cfg.road_condition.value if hasattr(cfg.road_condition, 'value') else str(cfg.road_condition)

            entries = [
                (f"#{scenario_idx+1}/{total}  {cfg.ego_type}  {road_str}",
                 carla.Color(150, 200, 255)),
                (f"Speed: {v_kmh:>4.0f} km/h", carla.Color(255, 255, 255)),
                (f"Dist:  {d:>5.1f} m", carla.Color(255, 255, 255)),
                (f"Signal: {signal}", signal_color),
                (f"Decision: {dec_en}", self._dec_color(dec_en)),
            ]

            sl = self.stopline_location
            base_z = sl.z + 8.0
            line_height = 0.7
            life = FIXED_DELTA_SECONDS * 1.2

            for i, (text, color) in enumerate(entries):
                text_loc = carla.Location(x=sl.x, y=sl.y, z=base_z - i * line_height)
                self.world.debug.draw_string(
                    text_loc, text, draw_shadow=True, color=color,
                    life_time=life, persistent_lines=False)
        except Exception:
            pass

    def _dec_color(self, dec_en):
        if dec_en == "GO": return carla.Color(0, 255, 0)
        if dec_en == "STOP": return carla.Color(255, 80, 80)
        if dec_en == "SLOW": return carla.Color(255, 200, 0)
        return carla.Color(255, 255, 255)

    def _set_static_camera(self, side_distance=25, height=12, pitch=-20):
        # 정지선 측면위쪽에 카메라 고정시킴
        if not self.world or not self.stopline_location:
            return
        if not self.move_dir:
            return
        try:
            sl = self.stopline_location
            perp_x = -self.move_dir[1]
            perp_y = self.move_dir[0]

            cam_loc = carla.Location(
                x=sl.x + perp_x * side_distance,
                y=sl.y + perp_y * side_distance,
                z=sl.z + height
            )

            target_yaw = math.degrees(math.atan2(-perp_y, -perp_x))

            spectator = self.world.get_spectator()
            spectator.set_transform(carla.Transform(
                cam_loc,
                carla.Rotation(pitch=pitch, yaw=target_yaw, roll=0)
            ))
        except Exception:
            pass

    def _set_yellow_signal(self, yellow_time_s):
        self._signal_changed = False

        if not self.world:
            return None

        all_tls = list(self.world.get_actors().filter("traffic.traffic_light*"))
        if not all_tls:
            return None

        for tl in all_tls:
            try:
                tl.freeze(False)
            except Exception:
                pass
        try:
            self.world.tick()
        except Exception:
            pass

        # 자동차 진행방향
        if self.move_dir:
            ego_yaw = math.degrees(math.atan2(self.move_dir[1], self.move_dir[0]))
        else:
            ego_yaw = 0.0

        target_tl = None
        if self.stopline_location:
            carla_map = self.world.get_map()
            ego_wp = carla_map.get_waypoint(
                self.stopline_location, project_to_road=True)

            if ego_wp:
                ego_road_id = ego_wp.road_id
                ego_lane_id = ego_wp.lane_id

                candidates = []
                for tl in all_tls:
                    try:
                        for wp in tl.get_stop_waypoints():
                            if wp.road_id == ego_road_id:
                                wp_yaw = wp.transform.rotation.yaw
                                candidates.append((tl, wp_yaw))
                                break
                    except Exception:
                        pass

                if not candidates:
                    for tl in all_tls:
                        try:
                            for wp in tl.get_affected_lane_waypoints():
                                if wp.road_id == ego_road_id:
                                    wp_yaw = wp.transform.rotation.yaw
                                    candidates.append((tl, wp_yaw))
                                    break
                        except Exception:
                            pass
                if candidates:
                    best = None
                    best_diff = 360
                    for tl, wp_yaw in candidates:
                        diff = abs(((wp_yaw - ego_yaw + 180) % 360) - 180)
                        if diff < best_diff:
                            best_diff = diff
                            best = tl
                    if best is not None and best_diff < 90:
                        target_tl = best

        if target_tl is None and self.selected_tl_id >= 0:
            for tl in all_tls:
                if tl.id == self.selected_tl_id:
                    target_tl = tl
                    break
        if target_tl:
            try:
                target_yaw = target_tl.get_transform().rotation.yaw
                yellow_group = [target_tl]
                red_group = []

                for tl in all_tls:
                    if tl.id == target_tl.id:
                        continue
                    tl_yaw = tl.get_transform().rotation.yaw
                    yaw_diff = abs(((tl_yaw - target_yaw + 180) % 360) - 180)
                    if yaw_diff > 135:
                        yellow_group.append(tl)
                    else:
                        red_group.append(tl)

                for tl in all_tls:
                    try:
                        tl.freeze(True)
                    except Exception:
                        pass

                target_tl.set_green_time(0.01)
                target_tl.set_yellow_time(yellow_time_s)
                target_tl.set_red_time(TRAFFIC_LIGHT_HOLD_S)

                for tl in yellow_group:
                    try:
                        tl.set_red_time(TRAFFIC_LIGHT_HOLD_S)
                        tl.set_yellow_time(yellow_time_s)
                        tl.set_state(carla.TrafficLightState.Yellow)
                    except: pass

                for tl in red_group:
                    try:
                        tl.set_red_time(TRAFFIC_LIGHT_HOLD_S)
                        tl.set_state(carla.TrafficLightState.Red)
                    except Exception:
                        pass

                self.world.tick()
            except Exception:
                target_tl = None

        return target_tl

    def _release_signal(self, traffic_light):
        if traffic_light:
            try:
                traffic_light.freeze(False)
            except Exception:
                pass

    def _update_signal(self, traffic_light, elapsed, yellow_time):
        if not traffic_light:
            return
        if getattr(self, '_signal_changed', False):
            return
        if elapsed < yellow_time:
            return
        try:
            traffic_light.freeze(False)
            traffic_light.set_state(carla.TrafficLightState.Red)
            traffic_light.freeze(True)
            self._signal_changed = True
        except Exception:
            pass

    def run_exp1(self, cfg, result):
        ego = self._spawn(cfg.distance_m, cfg.speed_kmh, cfg.ego_type, "ego")
        if not ego:
            result.error = "스폰실패"
            return
        self._set_weather(cfg.road_condition)


        tl = self._set_yellow_signal(cfg.yellow_time_s)
        self._set_static_camera()  # 정지선 측면 카메라 고정 

        sms = cfg.speed_ms
        skmh = cfg.speed_kmh
        total = cfg.distance_m + cfg.intersection_width_m + cfg.L
        prev = ego.get_location()
        cum = 0
        elapsed = 0
        passed = False
        pt = None
        ssum = 0
        scnt = 0
        spd_pass = 0
        extra_dist = sms * 2.0 
        finish_threshold = cfg.distance_m + max(cfg.intersection_width_m, 40) + cfg.L + 20 + extra_dist

        for _ in range(MAX_SIM_STEPS):
            cs = self._spd(ego)
            th, br = compute_throttle_brake(sms, cs, skmh)
            ego.apply_control(carla.VehicleControl(throttle=th, brake=br, steer=0))
            self.world.tick()
            elapsed += FIXED_DELTA_SECONDS

            self._draw_hud(ego, cfg, elapsed, "GO",
                           self._cur_scenario_idx, self._total_scenarios)

            self._update_signal(tl, elapsed, cfg.yellow_time_s)
            cl = ego.get_location()
            cum += self._cdist(prev, cl)
            prev = cl
            if not passed:
                ssum += cs
                scnt += 1
            if not passed and cum >= total:
                passed = True
                pt = elapsed
                spd_pass = self._spd(ego)
            if cum >= finish_threshold:
                break
        if passed and pt:
            result.pass_before_red = pt <= cfg.yellow_time_s
            result.actual_travel_time_s = round(pt, 3)
            result.time_margin_s = round(cfg.yellow_time_s - pt, 3)
        else:
            result.pass_before_red = False
            result.actual_travel_time_s = round(elapsed, 3)
            result.time_margin_s = round(cfg.yellow_time_s - elapsed, 3)
        if scnt > 0:
            result.avg_speed_kmh = round(ssum / scnt * 3.6, 1)
        result.final_speed_kmh = round((spd_pass if passed else self._spd(ego)) * 3.6, 1)
        result.match_theory = (result.pass_before_red == result.theoretical_pass)
        self._release_signal(tl)
        self._cleanup()

    def run_exp2(self, cfg, result):
        bstep = WARMUP_STEPS
        warmup_advance = cfg.speed_ms * (bstep * FIXED_DELTA_SECONDS)
        spawn_dist = cfg.distance_m + warmup_advance

        ego = self._spawn(spawn_dist, cfg.speed_kmh, cfg.ego_type, "ego")
        if not ego:
            result.error = "Ego스폰실패"
            return
        rear = self._spawn(spawn_dist + cfg.L + cfg.gap_m,
                           cfg.rear_speed_kmh, cfg.rear_type, "rear")
        if not rear:
            result.error = "후방스폰실패"
            self._cleanup()
            return
        self._set_weather(cfg.road_condition)

        tl = self._set_yellow_signal(cfg.yellow_time_s)
        self._set_static_camera()

        mg = cfg.gap_m
        col = False
        sr = VEHICLE_SPECS[cfg.rear_type]
        rear_t_r = sr["t_r"]
        for step in range(EXP2_MAX_STEPS):
            if step < bstep:
                th, br = compute_throttle_brake(cfg.speed_ms, self._spd(ego), cfg.speed_kmh)
                ego.apply_control(carla.VehicleControl(throttle=th, brake=br, steer=0))
                self._set_vel(rear, cfg.rear_speed_kmh)
            elif step == bstep:
                self._brake(ego)
                self._set_vel(rear, cfg.rear_speed_kmh)
            else:
                self._brake(ego)
                t = (step - bstep) * FIXED_DELTA_SECONDS
                if t > rear_t_r:
                    self._brake(rear)
                else:
                    self._set_vel(rear, cfg.rear_speed_kmh)
            self.world.tick()

            # HUD부분
            self._draw_hud(ego, cfg, step * FIXED_DELTA_SECONDS, "STOP",
                           self._cur_scenario_idx, self._total_scenarios)

            self._update_signal(tl, step * FIXED_DELTA_SECONDS, cfg.yellow_time_s)

            g = ego.get_location().distance(rear.get_location()) - cfg.L
            if g < mg:
                mg = g
            if g < COLLISION_GAP_M:
                col = True
            if step > bstep + 40 and self._spd_kmh(ego) < STATIONARY_KMH and self._spd_kmh(rear) < STATIONARY_KMH:
                break
        result.actual_collision = col
        result.min_gap_m = round(mg, 2)
        result.match_theory = (result.rear_collision_theory == col)
        self._release_signal(tl)
        self._cleanup()

    def run_exp3(self, cfg, result):
        # 알고리즘 판정대로 행동시킴 (진행=통과, 급정지/감속=정지선 정차)
        dec = result.decision

        astep = WARMUP_STEPS
        warmup_advance = cfg.speed_ms * (astep * FIXED_DELTA_SECONDS)
        spawn_dist = cfg.distance_m + warmup_advance

        ego = self._spawn(spawn_dist, cfg.speed_kmh, cfg.ego_type, "ego")
        if not ego:
            result.error = "Ego스폰실패"
            return

        rear = None
        if cfg.has_rear:
            rear_spawn_dist = spawn_dist + cfg.L + cfg.gap_m
            rear = self._spawn(rear_spawn_dist,
                               cfg.rear_speed_kmh, cfg.rear_type, "rear")

        self._set_weather(cfg.road_condition)
        tl = self._set_yellow_signal(cfg.yellow_time_s)
        self._set_static_camera()

        spec = VEHICLE_SPECS[cfg.ego_type]
        is_dry = (cfg.road_condition == RoadCondition.DRY)
        a_comf = spec["a_comf_dry"] if is_dry else spec["a_comf_wet"]
        a_max = spec["a_max_dry"] if is_dry else spec["a_max_wet"]
        # 후방차
        rear_spec = VEHICLE_SPECS[cfg.rear_type] if rear else None
        rear_t_r = rear_spec["t_r"] if rear_spec else 0
        if rear_spec:
            rear_a_comf = rear_spec.get("a_comf_dry", 3.4) if is_dry else rear_spec.get("a_comf_wet", 2.5)
            rear_a_max = rear_spec.get("a_max_dry", 7.0)
            rear_brake_intensity = float(min(1.0, rear_a_comf / rear_a_max))

        col = False
        red_entry = False
        safe_stop = False
        min_gap = cfg.gap_m if cfg.has_rear else NO_REAR_GAP
        cum = 0
        prev_loc = ego.get_location()
        # spawn~통과까지 누적거리
        total = warmup_advance + cfg.distance_m + cfg.intersection_width_m + cfg.L
        extra_distance = cfg.speed_ms * 2.0 
        pass_time = None  

        for step in range(EXP3_MAX_STEPS):
            current_v = self._spd(ego)

            if step < astep:
                th, br = compute_throttle_brake(cfg.speed_ms, current_v, cfg.speed_kmh)
                ego.apply_control(carla.VehicleControl(throttle=th, brake=br, steer=0))
                if rear:
                    self._set_vel(rear, cfg.rear_speed_kmh)
            else:
                if dec == "진행":
                    th, br = compute_throttle_brake(cfg.speed_ms, current_v, cfg.speed_kmh)
                    ego.apply_control(carla.VehicleControl(throttle=th, brake=br, steer=0))
                elif dec in ("급정지", "감속"):
                    if self.use_builder and self.builder:
                        if dec == "감속":
                            ctrl = self.builder.safe_stop_control(
                                ego, a_max=a_max, a_comfort=a_comf,
                                stop_offset=1.0,
                                target_speed_kmh=cfg.speed_kmh)
                        else:  # 급정지
                            ctrl = self.builder.safe_stop_control(
                                ego, a_max=a_max, a_comfort=a_comf * 0.7,
                                stop_offset=1.0,
                                target_speed_kmh=cfg.speed_kmh)
                        ego.apply_control(ctrl)
                    else:
                        dist_to_stopline = self.stopline_location.distance(ego.get_location())
                        target_dist = dist_to_stopline - DIST_MARGIN_M
                        if target_dist <= 0.3:
                            self._brake(ego)
                        else:
                            required_a = current_v ** 2 / (2 * max(0.3, target_dist))
                            brake_intensity = min(1.0, required_a / a_max * 1.05)
                            ego.apply_control(carla.VehicleControl(
                                throttle=0, brake=brake_intensity, steer=0))

                # 후방차 반응
                dt_action = (step - astep) * FIXED_DELTA_SECONDS
                if rear:
                    if dt_action < rear_t_r:
                        # 반응 전: 등속
                        self._set_vel(rear, cfg.rear_speed_kmh)
                    else:
                        # 반응 후: 앞차가 느려지면 후방도 브레이크
                        ego_kmh = self._spd_kmh(ego)
                        rear_kmh = self._spd_kmh(rear)
                        if dec in ("급정지", "감속") or rear_kmh > ego_kmh + 5:
                            rear.apply_control(carla.VehicleControl(
                                throttle=0, brake=rear_brake_intensity, steer=0))
                        else:
                            self._set_vel(rear, cfg.rear_speed_kmh)

            self.world.tick()

            elapsed_hud = max(0, (step - astep)) * FIXED_DELTA_SECONDS
            self._draw_hud(ego, cfg, elapsed_hud, dec,
                           self._cur_scenario_idx, self._total_scenarios)

            if step >= astep:
                self._update_signal(tl, (step - astep) * FIXED_DELTA_SECONDS, cfg.yellow_time_s)

            # 누적 이동거리 
            cur_loc = ego.get_location()
            cum += self._cdist(prev_loc, cur_loc)
            prev_loc = cur_loc

            # 후방 추돌 체크
            if rear:
                g = cur_loc.distance(rear.get_location()) - cfg.L
                if g < min_gap:
                    min_gap = g
                if g < COLLISION_GAP_M:
                    col = True

            # 통과 시점 기록
            if cum >= total and pass_time is None:
                pass_time = max(0, (step - astep) * FIXED_DELTA_SECONDS)
                if pass_time > cfg.yellow_time_s:
                    red_entry = True

            # 정지 판정: 1km/h 미만이면 정차로 보고 break
            if dec != "진행" and step > astep + 30:
                if self._spd_kmh(ego) < STATIONARY_KMH:
                    final_dist_to_stopline = self.stopline_location.distance(cur_loc)
                    # 정지선 5m 이내면 안전정차로 인정. 횡단보도/교차로 침범은 stop_overrun_m으로 봄
                    if final_dist_to_stopline < SAFE_STOP_RADIUS_M:
                        safe_stop = True
                    break

            if dec == "진행" and cum >= total + extra_distance and step > astep:
                break

            if step > EXP3_HARD_BREAK_STEP:
                break

        #결과!!!!!!!!!
        result.actual_collision = col
        result.red_entry = red_entry
        result.safe_stop = safe_stop
        result.min_gap_m = round(min_gap, 2)
        # actual_travel_time: 통과 시점, 없으면 break 시점
        if pass_time is not None:
            result.actual_travel_time_s = round(pass_time, 2)
        else:
            result.actual_travel_time_s = round((step - astep) * FIXED_DELTA_SECONDS, 2)

        result.final_speed_kmh = round(self._spd_kmh(ego), 1)

        if self.use_builder and self.builder:
            signed_dist = self.builder.distance_to_stopline(ego)
            result.dist_to_stopline_m = round(abs(signed_dist), 2)
            result.stop_overrun_m = round(-signed_dist, 2)
        else:
            d_abs = self.stopline_location.distance(ego.get_location())
            result.dist_to_stopline_m = round(d_abs, 2)
            result.stop_overrun_m = round(cum - cfg.distance_m, 2)

        if self.use_builder and self.builder:
            try:
                pts = self.builder.get_experiment_points()
                cw_w = pts.get("cw_width", 4.0)
                sb_d = pts.get("sb_distance", 2.0)
                int_w = pts.get("intersection_width", cfg.intersection_width_m)
                overrun = result.stop_overrun_m
                if overrun <= 0:
                    pass  # 정지선 전 정차
                elif overrun <= sb_d:
                    pass  # 횡단보도 진입 전
                elif overrun <= sb_d + cw_w:
                    result.in_crosswalk = True
                elif overrun <= sb_d + cw_w + sb_d + int_w:
                    result.in_intersection = True
                else:
                    result.past_intersection = True
            except Exception:
                pass
        else:
            overrun = result.stop_overrun_m
            if overrun > cfg.intersection_width_m:
                result.past_intersection = True
            elif overrun > 5:
                result.in_intersection = True
            elif overrun > 1.5:
                result.in_crosswalk = True

        # 알고리즘과 실제 비교
        result.is_dilemma_zone = (result.zone == "딜레마구간")
        if dec == "진행":
            result.decision_feasible = not red_entry
        elif dec in ("급정지", "감속"):
            result.decision_feasible = safe_stop

        # 성공 판정
        if dec == "진행":
            result.success = not red_entry
        else: 
            result.success = safe_stop and not col

        self._release_signal(tl)
        self._cleanup()

    def _set_weather(self, rc):
        if not self.world:
            return
        if self.use_builder and self.builder:
            try:
                self.builder.setup_weather(rc.value)
            except Exception:
                pass
        if rc == RoadCondition.WET:
            self.world.set_weather(carla.WeatherParameters(
                cloudiness=80, precipitation=60, precipitation_deposits=80,
                sun_altitude_angle=40, wetness=80, fog_density=5))
        else:
            self.world.set_weather(carla.WeatherParameters(
                cloudiness=10, precipitation=0,
                sun_altitude_angle=65, wetness=0, fog_density=0))

    def run_scenario(self, cfg):
        self.load_intersection(int(cfg.intersection_width_m))
        pred = algorithm_predict(cfg)
        result = ScenarioResult(
            scenario_id=cfg.scenario_id, exp=cfg.exp, speed_kmh=cfg.speed_kmh,
            distance_m=cfg.distance_m, yellow_time_s=cfg.yellow_time_s,
            intersection_width_m=cfg.intersection_width_m,
            road_condition=cfg.road_condition.value,
            ego_type=cfg.ego_type,
            rear_speed_kmh=cfg.rear_speed_kmh if cfg.has_rear else 0,
            gap_m=cfg.gap_m if cfg.has_rear else 0,
            rear_type=cfg.rear_type if cfg.has_rear else "",
            theoretical_pass=pred["can_pass"], can_stop_emg=pred["can_stop"],
            rear_collision_theory=pred["rear_col"],
            decision=pred["decision"], zone=pred["zone"],
            timestamp=datetime.now().isoformat())
        try:
            if cfg.exp == 1:
                self.run_exp1(cfg, result)
            elif cfg.exp == 2:
                self.run_exp2(cfg, result)
            elif cfg.exp == 3:
                self.run_exp3(cfg, result)
        except Exception as e:
            result.error = str(e)
            print(f"\n  [ERR] id={cfg.scenario_id}: {e}")
            self._cleanup()
        return result

    def disconnect(self):
        self._cleanup()
        if self.builder:
            self.builder.cleanup()
        print("[INFO] cleanup done")

class ResultManager:
    def __init__(self, od=OUTPUT_DIR):
        self.od = od
        self.results = []
        os.makedirs(od, exist_ok=True)

    def add(self, r):
        self.results.append(r)

    def load_intermediate(self, exp):
        # 중간 파일 읽어서 재개 자동 백업도 같이 함
        fp = os.path.join(self.od, f"exp{exp}_intermediate.csv")
        if not os.path.exists(fp):
            return -1

        # 자동 백업
        try:
            import shutil
            backup_name = f"exp{exp}_backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            backup_fp = os.path.join(self.od, backup_name)
            shutil.copy2(fp, backup_fp)
            print(f"[BACKUP] {fp} → {backup_name}")
        except Exception as e:
            print(f"[WARN] 백업 실패 (계속 진행): {e}")

        try:
            import csv as _csv
            with open(fp, "r", encoding="utf-8-sig") as f:
                reader = _csv.DictReader(f)
                rows = list(reader)
            if not rows:
                return -1

            from dataclasses import fields
            sr_fields = {f.name: f.type for f in fields(ScenarioResult)}

            last_id = -1
            for row in rows:
                kwargs = {}
                for k, v in row.items():
                    if k not in sr_fields:
                        continue
                    if v == "" or v is None:
                        continue
                    t = sr_fields[k]
                    try:
                        if t == int or "int" in str(t):
                            kwargs[k] = int(float(v))
                        elif t == float or "float" in str(t):
                            kwargs[k] = float(v)
                        elif t == bool or "bool" in str(t):
                            kwargs[k] = (v == "True" or v == "true" or v == "1")
                        else:
                            kwargs[k] = v
                    except (ValueError, TypeError):
                        kwargs[k] = v

                try:
                    r = ScenarioResult(**kwargs)
                    self.results.append(r)
                    if r.scenario_id > last_id:
                        last_id = r.scenario_id
                except Exception:
                    pass

            print(f"[RESUME] {fp}에서 {len(self.results)}건 복원 (마지막 id={last_id})")
            return last_id
        except Exception as e:
            print(f"[RESUME] 복원 실패: {e}")
            return -1

    def save_csv(self, fn=None):
        if not self.results:
            return
        if not fn:
            prefix = getattr(self, 'label', None) or f"exp{self.results[0].exp}"
            fn = f"{prefix}_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        fp = os.path.join(self.od, fn)
        fns = list(asdict(self.results[0]).keys())
        with open(fp, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=fns)
            w.writeheader()
            for r in self.results:
                w.writerow(asdict(r))
        print(f"[INFO] CSV → {fp} (n={len(self.results)})")

    def save_json(self, fn=None):
        if not self.results:
            return
        if not fn:
            prefix = getattr(self, 'label', None) or f"exp{self.results[0].exp}"
            fn = f"{prefix}_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        fp = os.path.join(self.od, fn)
        with open(fp, "w", encoding="utf-8") as f:
            json.dump([asdict(r) for r in self.results], f, ensure_ascii=False, indent=2)
        print(f"[INFO] JSON → {fp}")

    def save_intermediate(self):
        prefix = getattr(self, 'label', None) or f"exp{self.results[0].exp}"
        self.save_csv(f"{prefix}_intermediate.csv")
    def print_summary(self):
        if not self.results:
            print("[WARN] 결과 없음")
            return
        total = len(self.results)
        valid = [r for r in self.results if not r.error]
        errs = total - len(valid)
        ph = self.results[0].exp
        print(f"\n{'=' * 60}\n  결과 보고\n{'=' * 60}")
        print(f"  전체:{total} 정상:{len(valid)} 에러:{errs}")
        if not valid:
            return
        if ph == 1:
            m = sum(1 for r in valid if r.match_theory)
            print(f"  이론일치: {m}/{len(valid)} ({m / len(valid) * 100:.1f}%)")
            for vt in ["sedan", "suv", "truck", "bus"]:
                sub = [r for r in valid if r.ego_type == vt]
                if sub:
                    mm = sum(1 for r in sub if r.match_theory)
                    print(f"    {vt}: {mm}/{len(sub)} ({mm / len(sub) * 100:.1f}%)")
        elif ph == 2:
            m = sum(1 for r in valid if r.match_theory)
            c = sum(1 for r in valid if r.actual_collision)
            print(f"  추돌:{c}/{len(valid)} 이론일치:{m}/{len(valid)} ({m / len(valid) * 100:.1f}%)")
        elif ph == 3:
            s = sum(1 for r in valid if r.success)
            print(f"  성공률: {s}/{len(valid)} ({s / len(valid) * 100:.1f}%)")
            for dc in ["진행", "감속", "급정지"]:
                sub = [r for r in valid if r.decision == dc]
                if sub:
                    ss = sum(1 for r in sub if r.success)
                    print(f"    {dc}: {ss}/{len(sub)} ({ss / len(sub) * 100:.1f}%)")

            # 딜레마존 분석
            print(f"\n  [딜레마존 분석]")
            for zn in ["진행/정지선택구간", "일반구간", "딜레마구간"]:
                sub = [r for r in valid if r.zone == zn]
                if not sub:
                    continue
                ss = sum(1 for r in sub if r.success)
                print(f"    {zn}: {len(sub)}건 (성공률 {ss / len(sub) * 100:.1f}%)")

            # 정차 위치 통계 (정지/감속 판정만)
            stop_results = [r for r in valid if r.decision in ("급정지", "감속")]
            if stop_results:
                n_stop = len(stop_results)
                normal = sum(1 for r in stop_results if r.stop_overrun_m <= 0)
                cw = sum(1 for r in stop_results if r.in_crosswalk)
                inter = sum(1 for r in stop_results if r.in_intersection)
                past = sum(1 for r in stop_results if r.past_intersection)
                print(f"\n  [정차 위치] (정지/감속 판정 {n_stop}건)")
                print(f"    정상 (정지선 전):   {normal} ({normal / n_stop * 100:.1f}%)")
                print(f"    횡단보도 침범:      {cw} ({cw / n_stop * 100:.1f}%)")
                print(f"    교차로 내부:        {inter} ({inter / n_stop * 100:.1f}%)")
                print(f"    교차로 통과:        {past} ({past / n_stop * 100:.1f}%)")

            # 딜레마존만 따로
            dilemma = [r for r in valid if r.is_dilemma_zone]
            if dilemma:
                print(f"\n  [딜레마존 케이스 {len(dilemma)}건]")
                feasible = sum(1 for r in dilemma if r.decision_feasible)
                print(f"    판정대로 실행 가능: {feasible}/{len(dilemma)} ({feasible / len(dilemma) * 100:.1f}%)")
                avg_overrun = sum(r.stop_overrun_m for r in dilemma) / len(dilemma)
                print(f"    평균 정지선 침범: {avg_overrun:+.2f}m")
        print(f"{'=' * 60}\n")

def run_carla_simulation(exp, quick=False, pause_sec=1.5, resume=False, custom_scenarios=None, label=None):
    
    if not CARLA_AVAILABLE:
        print("[ERR] carla 없음")
        return
    print("=" * 60)
    if custom_scenarios is not None:
        print(f" 커스텀 시나리오 ({label or 'custom'}, 대기 {pause_sec}s, resume={resume})")
    else:
        print(f" 실험 {exp} 시뮬레이션 (대기 {pause_sec}s, resume={resume})")
    print("=" * 60)
    sim = CollisionRiskSimulator()
    sim.setup()
    if custom_scenarios is not None:
        scenarios = custom_scenarios
    else:
        scenarios = generate_scenarios(exp, quick)
    total = len(scenarios)
    sim._total_scenarios = total
    print(f"[INFO] 시나리오={total:,}{'  (quick)' if quick else ''}\n")

    mgr = ResultManager()
    if label:
        mgr.label = label 

    # 이전 중간 파일에서 복원
    last_done_id = -1
    if resume:
        last_done_id = mgr.load_intermediate(exp)
        if last_done_id >= 0:
            remaining = total - (last_done_id + 1)
            print(f"[RESUME] {last_done_id + 1}건 완료, {remaining}건 남음")
        else:
            print("[RESUME] 이전 데이터 없음, 처음부터 시작")

    try:
        cw = None
        wc = 0
        for i, cfg in enumerate(scenarios):
            # 이미 진행한 시나리오는 건너뜀
            if cfg.scenario_id <= last_done_id:
                continue

            sim._cur_scenario_idx = i
            if cw != cfg.intersection_width_m:
                if cw is not None:
                    print(f"\n  [done] {int(cw)}m (n={wc})")
                cw = cfg.intersection_width_m
                wc = 0
            wc += 1
            ri = f" r={cfg.rear_speed_kmh:.0f} g={cfg.gap_m:.0f}" if cfg.has_rear else ""
            print(f"\n  [{i + 1}/{total}] v={cfg.speed_kmh:.0f} d={cfg.distance_m:.0f} "
                  f"Y={cfg.yellow_time_s:.0f} W={int(cw)} "
                  f"{cfg.ego_type} {cfg.road_condition.value}{ri}")
            result = sim.run_scenario(cfg)
            mgr.add(result)

            # 결과 요약 출력
            if exp == 3:
                # 정차 위치 영역 표시
                if result.past_intersection:
                    pos_zone = "[교차로통과]"
                elif result.in_intersection:
                    pos_zone = "[교차로내]"
                elif result.in_crosswalk:
                    pos_zone = "[횡단보도]"
                elif result.stop_overrun_m > 0:
                    pos_zone = "[정지선침범]"
                else:
                    pos_zone = "[정상]"

                # 딜레마존 
                dz = "[딜레마]" if result.is_dilemma_zone else result.zone

                print(f"    → 판정={result.decision} 성공={'OK' if result.success else 'NG'} "
                      f"zone={dz} {pos_zone} "
                      f"overrun={result.stop_overrun_m:+.2f}m "
                      f"시간={result.actual_travel_time_s:.1f}s")
            elif exp == 1:
                print(f"    → 이론={result.theoretical_pass} 실제={result.pass_before_red} "
                      f"일치={'OK' if result.match_theory else 'NG'}")
            elif exp == 2:
                print(f"    → 이론충돌={result.rear_collision_theory} "
                      f"실제충돌={result.actual_collision} "
                      f"min_gap={result.min_gap_m:.1f}m")

            if (i + 1) % 50 == 0:
                mgr.save_intermediate()
                print(f"  [save] {i + 1}/{total}")

            # 시나리오 간 대기
            if pause_sec > 0 and i < total - 1:
                time.sleep(pause_sec)
        if cw:
            print(f"\n  [done] {int(cw)}m (n={wc})")
    except KeyboardInterrupt:
        print("\n\n[INFO] 사용자가 중단 (Ctrl+C)")
        try:
            if mgr.results:
                mgr.save_intermediate()
                print(f"[INFO] intermediate.csv 갱신 완료 ({len(mgr.results)}건)")
        except Exception as e:
            print(f"[WARN] intermediate 저장 실패: {e}")
    except RuntimeError as e:
        print(f"\n\n[ERR] CARLA 런타임 에러: {e}")
        print("[INFO] CARLA 서버 재시작 후 resume=y 로 이어서 진행하세요.")
        try:
            if mgr.results:
                mgr.save_intermediate()
                print(f"[INFO] intermediate.csv 갱신 완료 ({len(mgr.results)}건)")
        except Exception as e2:
            print(f"[WARN] intermediate 저장 실패: {e2}")
    except Exception as e:
        print(f"\n\n[ERR] 예외 발생: {e}")
        try:
            if mgr.results:
                mgr.save_intermediate()
                print(f"[INFO] intermediate.csv 갱신 완료 ({len(mgr.results)}건)")
        except Exception as e2:
            print(f"[WARN] intermediate 저장 실패: {e2}")
    finally:
        mgr.save_csv()
        mgr.save_json()
        mgr.print_summary()
        sim.disconnect()
    print("[INFO] 끝!")


if __name__ == "__main__":
    print("\n모드:")
    print("  1) 실험 1 — 전방 통과 (차종/노면)")
    print("  2) 실험 2 — 후방 추돌")
    print("  3) 실험 3 — 종합")
    print("  4) Quick 테스트")
    print("  5) 이론 분석만")
    print("  6) 딜레마구간만 재실험 (JSON 시나리오)")
    mode = input("번호? ").strip()

    pause = 1.5  # 기본값
    resume = False
    if mode in ("1", "2", "3", "4", "6"):
        try:
            inp = input("시나리오 간 대기 시간(초) [기본 1.5, 빠르게 0.3]: ").strip()
            if inp:
                pause = float(inp)
        except (ValueError, EOFError):
            pass
        try:
            r_inp = input("이전 결과부터 이어서 진행? (Y/N): ").strip().lower()
            resume = (r_inp == "y" or r_inp == "yes")
        except EOFError:
            pass
        print(f"[INFO] 대기 시간: {pause}s, resume={resume}")

    if mode == "1":
        run_carla_simulation(1, pause_sec=pause, resume=resume)
    elif mode == "2":
        run_carla_simulation(2, pause_sec=pause, resume=resume)
    elif mode == "3":
        run_carla_simulation(3, pause_sec=pause, resume=resume)
    elif mode == "4":
        run_carla_simulation(1, True, pause_sec=pause, resume=resume)
    elif mode == "5":
        for ph in [1, 2, 3]:
            scs = generate_scenarios(ph, True)
            mgr = ResultManager()
            for c in scs:
                p = algorithm_predict(c)
                r = ScenarioResult(
                    c.scenario_id, c.exp, c.speed_kmh, c.distance_m, c.yellow_time_s,
                    c.intersection_width_m, c.road_condition.value, c.ego_type,
                    c.rear_speed_kmh if c.has_rear else 0,
                    c.gap_m if c.has_rear else 0,
                    c.rear_type if c.has_rear else "",
                    p["can_pass"], p["can_stop"], p["rear_col"],
                    p["decision"], p["zone"],
                    match_theory=True, timestamp=datetime.now().isoformat())
                mgr.add(r)
            mgr.save_csv(f"theoretical_exp{ph}.csv")
            mgr.print_summary()
    elif mode == "6":
        # 딜레마구간만 재실험
        json_path = input("시나리오 JSON 경로 [기본: dilemma_scenarios.json]: ").strip()
        if not json_path:
            json_path = "dilemma_scenarios.json"
        custom = load_scenarios_from_json(json_path)
        if custom:
            run_carla_simulation(3, pause_sec=pause, resume=resume,
                                 custom_scenarios=custom, label="dilemma")
    else:
        run_carla_simulation(1, True)
