# -*- coding: utf-8 -*-
"""
auto_aim_bot_v5.py - 양측 기동 교전 봇

v3 대비 추가
  1) 기동 표적 추정      TargetTracker 로 속도 + 가속도 + 선회율 추정,
                         등가속 예측으로 요격 해를 푼다
  2) 명중 확률 게이팅    비행시간 동안 표적이 최대 성능으로 기동했을 때의
                         편차를 계산해 P(hit) 를 구하고, 임계값 미만이면
                         쏘지 않고 거리를 좁힌다 (재장전 6.5 s 라 헛방이 비쌈)
  3) 후방 기동           피탄 부위별 피해가 다르므로(후면 최대) 적의 후방 호로
                         파고드는 기동을 한다
  4) 피탄 부위 분석      착탄 좌표 + 적 차체 방위로 전/측/후면을 역산하고
                         HP 감소량과 대조해 부위별 피해 계수를 실측한다

같은 폴더에 fire_control.py 필요.
실행:  python auto_aim_bot_v5.py   ->  http://localhost:5000

--------------------------------------------------------------------
실측 근거 (measure_harness.py P0~P4)
    탄속 58.38 m/s, 발사고 2.601 m, g 9.81
    앙각 -5 ~ +10 deg  -> 교전 포락 21.8 ~ 132.1 m
    포탑 방위 40 deg/s, 앙각 5 deg/s, 재장전 6.5 s
    차량 가속 2.44 m/s^2, 선회 40 deg/s
    포탑 비안정화 (차체와 함께 회전)  -> 회전분 피드포워드 필요
    탄착 산포 0, 차량 속도는 탄에 실리지 않음 (회귀 a=+0.045, R^2=0.002)
--------------------------------------------------------------------
"""
import math
import logging
import threading
from flask import Flask, request, jsonify, Response

from enemy_bot import EnemyBot

from fire_control import (Ballistics, FireControl, TurretParams, TargetSize,
                          TargetTracker, MotionLimits, hit_probability,
                          impact_aspect, aspect_zone, bearing, ang_diff)

logging.getLogger("werkzeug").setLevel(logging.ERROR)
app = Flask(__name__)

# ── 설정 ────────────────────────────────────────────────
START_BLUE = (150.0, 10.0, 50.0)
START_RED = (150.0, 10.0, 250.0)

MAP_MARGIN = 35.0
BODY_DEADBAND = 3.0
YAW_RATE = 40.0            # 차체 선회 (P1 실측)
CTRL_DT = 0.41

# 교전 중에는 차체 선회를 제한한다.
# 포탑과 차체의 각속도가 둘 다 40 deg/s 라, 차체가 전속 선회하면
# 포탑은 그것을 상쇄하는 데 회전량을 전부 소모해 조준할 여유가 없다.
BODY_TURN_CAP = 0.45       # 교전 상태에서 차체 선회 weight 상한
ENGAGE_DRIVE = 0.5         # 교전 상태 주행 weight

BAND_NEAR = 45.0           # 교전 거리 하한
BAND_FAR = 70.0            # 교전 거리 상한  (55 m 부근이 명중률 최적)
P_HIT_MIN = 0.45           # 이 확률 미만이면 쏘지 않는다
FLANK_ON = True            # 적 후방으로 파고드는 기동
SETTLE_BEFORE_FIRE = False # True 면 조준 완료 후 한 주기 정지 뒤 발사

# ── 적 전차 조종 (내장 enemyTracking 이 제대로 기동하지 않아 직접 조종) ──
ENEMY_PORT = 5100
ENEMY_BEHAVIOR = "evade"   # evade / circle / charge / patrol / static
ENEMY_KEEP_R = 90.0        # 유지 거리 [m]
ENEMY_MASK = False         # True 면 적도 시야 제한을 받는다 (공정 대전용)
ENEMY_TRACKING = True      # /init 플래그. False 면 5100 이 호출되지 않는다
# ────────────────────────────────────────────────────────


def neutral():
    return {"moveWS": {"command": "STOP", "weight": 1.0},
            "moveAD": {"command": "", "weight": 0.0},
            "turretQE": {"command": "", "weight": 0.0},
            "turretRF": {"command": "", "weight": 0.0},
            "fire": False}


# ══════════════════════════════════════════════════════════
# 1. 텔레메트리
# ══════════════════════════════════════════════════════════
class Telemetry:
    def __init__(self):
        self.raw = {}
        self.t = None
        self.my = self.enemy = None
        self.body_x = self.turret_x = self.turret_y = 0.0
        self.enemy_body_x = 0.0
        self.my_hp = self.enemy_hp = None
        self.my_speed = self.enemy_speed = 0.0

    def update(self, d):
        if not d:
            return
        self.raw.update(d)
        r = self.raw
        self.t = r.get("time")
        p, e = r.get("playerPos"), r.get("enemyPos")
        if isinstance(p, dict):
            self.my = (float(p.get("x", 0)), float(p.get("y", 0)), float(p.get("z", 0)))
        if isinstance(e, dict):
            self.enemy = (float(e.get("x", 0)), float(e.get("y", 0)), float(e.get("z", 0)))
        self.body_x = float(r.get("playerBodyX", self.body_x) or 0.0)
        self.turret_x = float(r.get("playerTurretX", self.turret_x) or 0.0)
        self.turret_y = float(r.get("playerTurretY", self.turret_y) or 0.0)
        self.enemy_body_x = float(r.get("enemyBodyX", self.enemy_body_x) or 0.0)
        self.my_hp = r.get("playerHealth", self.my_hp)
        self.enemy_hp = r.get("enemyHealth", self.enemy_hp)
        self.my_speed = float(r.get("playerSpeed", 0.0) or 0.0)
        self.enemy_speed = float(r.get("enemySpeed", 0.0) or 0.0)

    @property
    def ready(self):
        return self.my is not None and self.enemy is not None

    @property
    def dist(self):
        if not self.ready:
            return None
        return math.hypot(self.enemy[0] - self.my[0], self.enemy[2] - self.my[2])

    @property
    def my_aspect(self):
        """적 기준 우리 방향의 상대각. |a|>120 이면 우리가 적의 후방에 있다."""
        if not self.ready:
            return None
        b = bearing(self.enemy, self.my)
        return ((b - self.enemy_body_x + 180.0) % 360.0) - 180.0


# ══════════════════════════════════════════════════════════
# 2. 기동 - 후방 침투 + 교전 거리 유지
# ══════════════════════════════════════════════════════════
class Maneuver:
    """
    FLANK    적 후방 호로 이동 (피해 2배 구역)
    APPROACH 사거리 밖 -> 접근
    WITHDRAW 너무 가까움 -> 후퇴
    ORBIT    후방 확보 후 거리 유지 선회
    RECENTER 맵 경계
    """

    # 상태 전환 히스테리시스 - 경계에서 왔다갔다 하며 제자리 회전만 하는 것을 막는다
    FLANK_ENTER = 110.0     # 상대각이 이보다 작으면 후방으로 파고든다
    FLANK_EXIT = 145.0      # 이보다 커져야 후방 확보로 인정

    def __init__(self, bal: Ballistics):
        self.bal = bal
        self.state = "IDLE"
        self.orbit_dir = 1
        self.body_rate = 0.0
        self.goal = None
        self.dt = CTRL_DT
        self._flanking = True       # 히스테리시스 래치
        self._stuck_t = 0.0         # 정체 누적 시간
        self._unstick_until = 0.0

    def compute(self, tm: Telemetry):
        stop = ({"command": "STOP", "weight": 1.0},
                {"command": "", "weight": 0.0}, 0.0, False)
        if not tm.ready:
            self.state = "IDLE"
            return stop

        mx, _, mz = tm.my
        d = tm.dist
        rmin, rmax = self.bal.min_range(), self.bal.max_range()[0]
        near = max(BAND_NEAR, rmin * 1.25)
        far = min(BAND_FAR, rmax * 0.92)
        to_enemy = bearing(tm.my, tm.enemy)
        aspect = tm.my_aspect or 0.0

        # 후방 확보 여부 래치 (경계 진동 방지)
        if self._flanking and abs(aspect) >= self.FLANK_EXIT:
            self._flanking = False
        elif not self._flanking and abs(aspect) <= self.FLANK_ENTER:
            self._flanking = True

        # 교착 해소: 교전 밴드 안인데 오래 정지해 있으면 강제로 선회 이탈
        if tm.t is not None:
            if self.state in ("FLANK", "ORBIT") and tm.my_speed < 0.4:
                self._stuck_t += self.dt
            else:
                self._stuck_t = 0.0
            if self._stuck_t > 2.5 and tm.t > self._unstick_until:
                self._unstick_until = tm.t + 3.0
                self.orbit_dir *= -1
                self._stuck_t = 0.0
        unstick = tm.t is not None and tm.t < self._unstick_until

        out = not (MAP_MARGIN < mx < 300 - MAP_MARGIN and
                   MAP_MARGIN < mz < 300 - MAP_MARGIN)

        if unstick:
            # 강제 이탈 - 적과 수직 방향으로 무조건 전진
            self.state = "UNSTICK"
            tgt_yaw = to_enemy + 90.0 * self.orbit_dir
            drive, w = "W", 1.0
            self.goal = None
        elif out:
            self.state = "RECENTER"
            self.orbit_dir *= -1
            tgt_yaw = bearing(tm.my, (150.0, 0.0, 150.0))
            drive, w = "W", 1.0
            self.goal = (150.0, 150.0)
        elif d > far:
            self.state = "APPROACH"
            tgt_yaw = to_enemy
            drive, w = "W", 1.0
            self.goal = (tm.enemy[0], tm.enemy[2])
        elif d < near:
            self.state = "WITHDRAW"
            tgt_yaw = to_enemy
            drive, w = "S", 0.7
            self.goal = None
        elif FLANK_ON and self._flanking:
            # 후방으로 파고든다.
            # 후방 지점을 직접 겨누면 경로가 적을 관통해 접근/후퇴를 반복한다.
            # 적을 중심으로 한 원 위를 따라 단계적으로 돌아간다.
            self.state = "FLANK"
            r = (near + far) * 0.5
            cur_ang = bearing(tm.enemy, tm.my)          # 적 기준 우리 각위치
            rear_ang = tm.enemy_body_x + 180.0
            delta = ang_diff(rear_ang, cur_ang)
            self.orbit_dir = 1 if delta > 0 else -1
            step = max(-55.0, min(55.0, delta))         # 한 번에 55도씩 원호 이동
            ga = math.radians(cur_ang + step)
            gx = tm.enemy[0] + math.sin(ga) * r
            gz = tm.enemy[2] + math.cos(ga) * r
            self.goal = (gx, gz)
            tgt_yaw = bearing(tm.my, (gx, 0.0, gz))
            drive, w = "W", ENGAGE_DRIVE
        else:
            self.state = "ORBIT"
            # 후방 확보 후에도 같은 방식으로 원 위를 이동해 거리를 유지한다
            r = (near + far) * 0.5
            cur_ang = bearing(tm.enemy, tm.my)
            ga = math.radians(cur_ang + 45.0 * self.orbit_dir)
            gx = tm.enemy[0] + math.sin(ga) * r
            gz = tm.enemy[2] + math.cos(ga) * r
            self.goal = (gx, gz)
            tgt_yaw = bearing(tm.my, (gx, 0.0, gz))
            drive, w = "W", ENGAGE_DRIVE

        err = ang_diff(tgt_yaw, tm.body_x)
        ad = {"command": "", "weight": 0.0}
        rate = 0.0
        # 교전 상태에서는 포탑이 추적할 여유를 남기도록 선회를 제한한다
        cap = (BODY_TURN_CAP if self.state in ("FLANK", "ORBIT") else 1.0)
        if abs(err) > BODY_DEADBAND:
            wt = min(cap, max(0.1, abs(err) / (YAW_RATE * self.dt)))
            ad = {"command": "D" if err > 0 else "A", "weight": round(wt, 2)}
            rate = YAW_RATE * wt * (1.0 if err > 0 else -1.0)

        # 크게 틀어져도 절반 속도로 전진하며 선회한다 (제자리 정지 방지)
        if abs(err) < 45.0:
            ws = {"command": drive, "weight": w}
        elif abs(err) < 110.0:
            ws = {"command": drive, "weight": round(w * 0.45, 2)}
        else:
            ws = {"command": "STOP", "weight": 1.0}
        self.body_rate = rate
        return ws, ad, rate, abs(rate) > 1e-6


# ══════════════════════════════════════════════════════════
# 3. 사격 기록 + 피탄 부위 분석
# ══════════════════════════════════════════════════════════
class ShotLog:
    MAX = 80
    MATCH_R = 20.0
    T_MIN, T_MAX = 0.2, 3.0

    def __init__(self):
        self.records = []
        self.fired = 0
        self.tank_hits = 0
        self.obstacle_hits = 0
        self.incoming = 0
        self.unmatched = 0
        self.pending = None
        self.zone_stat = {"front": [0, 0.0], "side": [0, 0.0], "rear": [0, 0.0]}

    def on_fire(self, tm: Telemetry, sol):
        self.fired += 1
        self.pending = {
            "id": self.fired,
            "time": round(tm.t, 2) if tm.t is not None else 0.0,
            "t_raw": tm.t,
            "fire_pos": tuple(round(v, 1) for v in tm.my),
            "target_pos": tuple(round(v, 1) for v in tm.enemy),
            "enemy_body": round(tm.enemy_body_x, 1),
            "expect": (sol.aim_point[0], sol.aim_point[2]),
            "dist": round(sol.distance, 1),
            "aim": (round(sol.bearing, 2), round(sol.elevation, 2)),
            "tof": round(sol.flight, 2), "tof_raw": sol.flight,
            "lead": round(sol.lead, 1),
            "p_hit": round(sol.p_hit, 3),
            "own_speed": round(tm.my_speed, 1),
            "enemy_speed": round(tm.enemy_speed, 1),
            "impact": None, "miss": None, "range_err": None,
            "aspect": None, "zone": None, "damage": None,
            "result": "비행중", "kind": "pending",
        }

    def _is_ours(self, ix, iz, now):
        p = self.pending
        if p is None or ix is None:
            return False
        ex, ez = p["expect"]
        if math.hypot(ix - ex, iz - ez) > self.MATCH_R:
            return False
        if now is not None and p["t_raw"] is not None and p["tof_raw"]:
            dt = now - p["t_raw"]
            if not (self.T_MIN * p["tof_raw"] <= dt <= self.T_MAX * p["tof_raw"] + 1.0):
                return False
        return True

    def on_impact(self, d, now=None, enemy_body=0.0):
        ix, iy, iz = d.get("x"), d.get("y"), d.get("z")
        raw = str(d.get("hit", "")).lower()
        pos = (round(ix, 1), round(iy, 1), round(iz, 1)) if ix is not None else None

        if raw == "player":
            self.incoming += 1
            self.records.insert(0, {
                "id": "-", "time": round(now, 2) if now else "-",
                "fire_pos": None, "target_pos": None, "dist": "-",
                "aim": None, "tof": "-", "p_hit": "-", "own_speed": "-",
                "enemy_speed": "-", "impact": pos, "miss": None,
                "range_err": None, "aspect": None, "zone": None, "damage": None,
                "result": "피격 (적탄)", "kind": "incoming"})
            del self.records[self.MAX:]
            return

        if not self._is_ours(ix, iz, now):
            self.unmatched += 1
            return

        rec = self.pending
        if "enemy" in raw or "tank" in raw:
            kind, label = "tank", "명중"
            self.tank_hits += 1
            # 피탄 부위 역산
            if pos and rec.get("fire_pos"):
                asp = impact_aspect(rec["fire_pos"], pos, enemy_body)
                rec["aspect"] = round(asp, 1)
                rec["zone"] = aspect_zone(asp)
        elif "terrain" in raw or "ground" in raw:
            kind, label = "terrain", "지면"
        else:
            kind, label = "obstacle", "장애물"
            self.obstacle_hits += 1

        rec["impact"] = pos
        tp = rec.get("target_pos")
        if pos and tp:
            rec["miss"] = round(math.hypot(ix - tp[0], iz - tp[2]), 2)
        fp = rec.get("fire_pos")
        if pos and fp and isinstance(rec.get("dist"), (int, float)):
            rec["range_err"] = round(math.hypot(ix - fp[0], iz - fp[2]) - rec["dist"], 2)
        rec["result"] = f"{label} ({raw})"
        rec["kind"] = kind
        self.records.insert(0, rec)
        del self.records[self.MAX:]
        self.pending = None

    def attach_damage(self, dmg: float):
        """적 HP 감소를 직전 명중 기록에 붙이고 부위별로 집계"""
        for r in self.records:
            if r["kind"] == "tank" and r.get("damage") is None:
                r["damage"] = round(dmg, 1)
                z = r.get("zone")
                if z in self.zone_stat:
                    self.zone_stat[z][0] += 1
                    self.zone_stat[z][1] += dmg
                return

    @property
    def hit_rate(self):
        return (self.tank_hits / self.fired * 100.0) if self.fired else 0.0

    @property
    def mean_miss(self):
        v = [r["miss"] for r in self.records
             if isinstance(r.get("miss"), (int, float)) and r["kind"] != "incoming"]
        return sum(v) / len(v) if v else None

    def zone_summary(self):
        out = []
        for z, ko in (("front", "전면"), ("side", "측면"), ("rear", "후면")):
            n, s = self.zone_stat[z]
            out.append((ko, n, s / n if n else None))
        return out


# ══════════════════════════════════════════════════════════
# 4. 봇
# ══════════════════════════════════════════════════════════
class Bot:
    def __init__(self):
        self.tm = Telemetry()
        self.bal = Ballistics()
        self.fc = FireControl(self.bal, TurretParams(), reload_s=6.5,
                              target=TargetSize())
        self.mv = Maneuver(self.bal)
        self.trk = TargetTracker(limits=MotionLimits())
        self.log = ShotLog()
        self._prev_enemy_hp = None
        self._settle = 0
        self.enemy = EnemyBot(behavior=ENEMY_BEHAVIOR, keep_r=ENEMY_KEEP_R,
                              mask=ENEMY_MASK)
        self._last_act_t = None
        self.ctrl_dt = CTRL_DT          # 실측으로 계속 보정된다

    def _measure_dt(self, t):
        """
        /get_action 실제 호출 주기를 측정해 제어 상수에 반영한다.
        시뮬레이터 Interval 설정이나 PC 성능에 따라 달라지므로
        고정값(0.41 s)을 쓰면 포탑 weight 가 어긋난다.
        """
        if t is None:
            return
        if self._last_act_t is not None:
            dt = t - self._last_act_t
            if 0.05 < dt < 3.0:
                self.ctrl_dt = 0.2 * dt + 0.8 * self.ctrl_dt
                self.fc.t.dt = self.ctrl_dt
        self._last_act_t = t

    def reset(self):
        self.tm = Telemetry()
        self.trk = TargetTracker(limits=MotionLimits())
        self.fc.last_fire_t = None
        self.fc.state = "IDLE"
        self._prev_enemy_hp = None
        self._settle = 0
        self.enemy = EnemyBot(behavior=ENEMY_BEHAVIOR, keep_r=ENEMY_KEEP_R,
                              mask=ENEMY_MASK)
        self._last_act_t = None
        self.ctrl_dt = CTRL_DT

    def on_info(self, d):
        self.tm.update(d)
        tm = self.tm
        if tm.ready and tm.t is not None:
            self.trk.update(tm.t, tm.enemy)
        # Enemy EndPoint 는 자기 위치만 받으므로 전장 정보를 공유해 준다
        if tm.ready:
            self.enemy.share_world(tm.my)
            self.enemy.set_body(tm.enemy_body_x)
        # 적 HP 감소 -> 직전 명중에 피해량 부여
        hp = tm.enemy_hp
        if hp is not None and self._prev_enemy_hp is not None and hp < self._prev_enemy_hp:
            self.log.attach_damage(self._prev_enemy_hp - hp)
        if hp is not None:
            self._prev_enemy_hp = hp

    def act(self, payload):
        self.tm.update(payload)
        tm = self.tm
        self._measure_dt(tm.t)
        if not tm.ready:
            return neutral()

        self.mv.dt = self.ctrl_dt
        ws, ad, body_rate, turning = self.mv.compute(tm)

        sgn = -1.0 if ws["command"] == "S" else (1.0 if ws["command"] == "W" else 0.0)
        rad = math.radians(tm.body_x)
        my_vel = (math.sin(rad) * tm.my_speed * sgn, 0.0,
                  math.cos(rad) * tm.my_speed * sgn)

        prev_fire_t = self.fc.last_fire_t
        turret = self.fc.update(
            my_pos=tm.my, turret_x=tm.turret_x, turret_y=tm.turret_y,
            target_pos=tm.enemy, target_vel=self.trk.vel,
            sim_time=tm.t or 0.0, body_moving=turning,
            body_rate_dps=body_rate, my_vel=my_vel,
            allow_moving_fire=True, tracker=self.trk, p_hit_min=P_HIT_MIN)

        # 조준 완료 후 한 주기 포탑 정지 (선택)
        if SETTLE_BEFORE_FIRE and turret["fire"] and self._settle == 0:
            self._settle = 1
            self.fc.last_fire_t = prev_fire_t
            return {"moveWS": ws, "moveAD": ad,
                    "turretQE": {"command": "", "weight": 0.0},
                    "turretRF": {"command": "", "weight": 0.0}, "fire": False}
        self._settle = 0

        if turret.get("fire") and self.fc.last_fire_t != prev_fire_t:
            self.log.on_fire(tm, self.fc.last_solution)

        return {"moveWS": ws, "moveAD": ad,
                "turretQE": turret["turretQE"], "turretRF": turret["turretRF"],
                "fire": turret["fire"]}


bot = Bot()


# ══════════════════════════════════════════════════════════
# 5. 엔드포인트
# ══════════════════════════════════════════════════════════
@app.route("/init", methods=["GET", "POST"])
def init():
    bot.reset()
    print("=== 에피소드 초기화 ===")
    return jsonify({
        "startMode": "start",
        "blStartX": START_BLUE[0], "blStartY": START_BLUE[1], "blStartZ": START_BLUE[2],
        "rdStartX": START_RED[0], "rdStartY": START_RED[1], "rdStartZ": START_RED[2],
        "trackingMode": True, "detectMode": False, "logMode": False,
        "stereoCameraMode": False,
        # enemyTracking 은 "적 전차를 제어 대상으로 활성화" 하는 플래그로 보인다.
        # False 로 두면 Enemy EndPoint(5100) 자체가 호출되지 않으므로 True 유지.
        # 실제 조종은 Use Enemy Server 체크 + 5100 서버가 담당한다.
        "enemyTracking": ENEMY_TRACKING,
        "saveSnapshot": False, "saveLog": False, "saveLidarData": False,
        "lux": 30000, "destoryObstaclesOnHit": False,
    })


@app.route("/start", methods=["GET", "POST"])
def start():
    return jsonify({"control": ""})


@app.route("/info", methods=["POST"])
def info():
    bot.on_info(request.get_json(force=True, silent=True) or {})
    return jsonify({"status": "success", "control": ""})


@app.route("/get_action", methods=["POST"])
def get_action():
    return jsonify(bot.act(request.get_json(force=True, silent=True) or {}))


@app.route("/update_bullet", methods=["POST"])
def update_bullet():
    bot.log.on_impact(request.get_json(force=True, silent=True) or {},
                      now=bot.tm.t, enemy_body=bot.tm.enemy_body_x)
    return jsonify({"status": "OK"})


@app.route("/collision", methods=["POST"])
def collision():
    return jsonify({"status": "success"})


@app.route("/detect", methods=["POST"])
def detect():
    return jsonify([])


@app.route("/stereo_image", methods=["POST"])
def stereo():
    return jsonify({"result": "success"})


@app.route("/set_destination", methods=["POST"])
def set_dest():
    return jsonify({"status": "OK"})


@app.route("/update_obstacle", methods=["POST"])
def upd_obs():
    return jsonify({"status": "success"})


@app.route("/export")
def export():
    import csv, io
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(["id", "time", "dist", "miss", "range_err", "p_hit",
                "aim_bearing", "aim_elev", "tof", "lead",
                "own_speed", "enemy_speed", "aspect", "zone", "damage",
                "result", "kind"])
    for r in reversed(bot.log.records):
        am = r.get("aim") or (None, None)
        w.writerow([r.get("id"), r.get("time"), r.get("dist"), r.get("miss"),
                    r.get("range_err"), r.get("p_hit"), am[0], am[1],
                    r.get("tof"), r.get("lead"), r.get("own_speed"),
                    r.get("enemy_speed"), r.get("aspect"), r.get("zone"),
                    r.get("damage"), r.get("result"), r.get("kind")])
    return Response(buf.getvalue(), mimetype="text/csv",
                    headers={"Content-Disposition": "attachment; filename=shots_v5.csv"})


@app.route("/reset")
def reset_stats():
    bot.log.__init__()
    return '<meta charset="utf-8">통계 초기화 <a href="/">돌아가기</a>'


# ══════════════════════════════════════════════════════════
# 6. 대시보드
# ══════════════════════════════════════════════════════════
@app.route("/")
def index():
    tm, fc, lg, trk = bot.tm, bot.fc, bot.log, bot.trk
    sol = fc.last_solution
    rmin, rmax = bot.bal.min_range(), bot.bal.max_range()[0]

    def fmt(v, u="", n=1):
        return f"{v:.{n}f}{u}" if isinstance(v, (int, float)) else "-"

    rows = ""
    for r in lg.records:
        c = {"tank": "#3fb950", "obstacle": "#e3b341",
             "incoming": "#f85149"}.get(r["kind"], "#8b949e")
        zc = {"rear": "#f85149", "side": "#e3b341", "front": "#58a6ff"}.get(r.get("zone"), "#8b949e")
        zt = {"rear": "후면", "side": "측면", "front": "전면"}.get(r.get("zone"), "-")
        am = r.get("aim")
        rows += (f"<tr><td>#{r['id']}</td><td>{r['time']}</td>"
                 f"<td>{r['dist']}</td>"
                 f"<td style='color:#e3b341'>{fmt(r.get('miss'),' m',2)}</td>"
                 f"<td style='color:#a5a5ff'>{fmt(r.get('range_err'),' m',2)}</td>"
                 f"<td>{fmt(r.get('p_hit'),'',2)}</td>"
                 f"<td>{fmt(r.get('own_speed'),'',1)}/{fmt(r.get('enemy_speed'),'',1)}</td>"
                 f"<td style='color:{zc}'>{zt} {fmt(r.get('aspect'),'°',0)}</td>"
                 f"<td>{fmt(r.get('damage'),'',1)}</td>"
                 f"<td style='color:{c};font-weight:600'>{r['result']}</td></tr>")
    if not rows:
        rows = "<tr><td colspan='10' style='text-align:center;color:#8b949e'>사격 기록 없음</td></tr>"

    zrows = "".join(
        f"<tr><td>{ko}</td><td>{n}</td><td>{fmt(avg,'',2)}</td></tr>"
        for ko, n, avg in lg.zone_summary())

    d = tm.dist
    asp = tm.my_aspect
    asp_txt = "-" if asp is None else f"{asp:+.0f}° ({'후방' if abs(asp)>120 else '측면' if abs(asp)>60 else '전방'})"
    sol_txt = "-"
    if sol and sol.valid:
        sol_txt = (f"방위 {sol.bearing:.2f} / 앙각 {sol.elevation:+.2f} / "
                   f"비행 {sol.flight:.2f}s / 리드 {sol.lead:.1f}m / "
                   f"P(hit) {sol.p_hit*100:.0f}%")
    elif sol:
        sol_txt = sol.reason
    sl = sn = 0.0
    if sol and sol.valid:
        sl, sn = trk.sigma(sol.flight)

    ecolor = "#3fb950" if bot.enemy.calls > 0 else "#f85149"
    ewarn = ("" if bot.enemy.calls > 0 else
             '<div class="card" style="border-color:#f85149"><div class="k" '
             'style="color:#f85149">Enemy EndPoint 미호출</div>'
             '<div style="font-size:13px;line-height:1.7">'
             f'포트 {ENEMY_PORT} 로 요청이 오지 않고 있습니다. 확인 순서:<br>'
             f'1. 브라우저에서 <b>http://localhost:{ENEMY_PORT}/ping</b> 열어 서버 생존 확인<br>'
             '2. 시뮬레이터 Setting → <b>Use Enemy Server 체크</b>, '
             f'Enemy Request Port = <b>{ENEMY_PORT}</b><br>'
             '3. Save 누른 뒤 Run 으로 에피소드 <b>재시작</b><br>'
             '4. 그래도 안 되면 Versus 모드로 전환해 Red Url 에 '
             f'<b>http://127.0.0.1:{ENEMY_PORT}</b> 지정</div></div>')

    return f"""<!DOCTYPE html><html><head><meta charset="utf-8">
<title>Tank FCS v5</title><meta http-equiv="refresh" content="1">
<style>
body{{background:#0d1117;color:#c9d1d9;font-family:"Segoe UI",monospace;margin:0;padding:22px}}
h1{{color:#58a6ff;font-size:21px;margin:0 0 4px}}
.sub{{color:#8b949e;font-size:12px;margin-bottom:16px}}
.row{{display:flex;gap:12px;margin-bottom:12px;flex-wrap:wrap}}
.card{{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:12px 18px;flex:1;min-width:130px}}
.k{{font-size:11px;color:#8b949e;margin-bottom:3px}}
.v{{font-size:22px;font-weight:700;color:#f0f6fc}}
table{{width:100%;border-collapse:collapse;background:#161b22;border:1px solid #30363d;border-radius:8px;overflow:hidden}}
th,td{{padding:8px 10px;text-align:left;border-bottom:1px solid #21262d;font-size:12px}}
th{{background:#21262d;color:#8b949e}}
h2{{font-size:15px;color:#8b949e;margin:20px 0 8px}}
</style></head><body>
<h1>Tank FCS v5 — 양측 기동 교전</h1>
<div class="sub">포락 {rmin:.0f}~{rmax:.0f} m · 앙각 {bot.bal.p.theta_min:+.0f}~{bot.bal.p.theta_max:+.0f}° ·
재장전 {fc.reload_s}s · 교전밴드 {BAND_NEAR:.0f}~{BAND_FAR:.0f} m ·
사격 임계 P(hit)≥{P_HIT_MIN:.2f} · 후방기동 {'ON' if FLANK_ON else 'OFF'} ·
제어주기 실측 <b style="color:#79c0ff">{bot.ctrl_dt:.3f}s</b></div>

<div class="row">
  <div class="card"><div class="k">사격</div><div class="v">{lg.fired}</div></div>
  <div class="card"><div class="k">명중</div><div class="v" style="color:#3fb950">{lg.tank_hits}</div></div>
  <div class="card"><div class="k">명중률</div><div class="v" style="color:#3fb950">{lg.hit_rate:.1f}%</div></div>
  <div class="card"><div class="k">평균 착탄오차</div><div class="v">{fmt(lg.mean_miss,' m',2)}</div></div>
  <div class="card"><div class="k">피격</div><div class="v" style="color:#f85149">{lg.incoming}</div></div>
  <div class="card"><div class="k">적탄(미분류)</div><div class="v" style="color:#8b949e">{lg.unmatched}</div></div>
</div>

<div class="row">
  <div class="card"><div class="k">사격 상태</div><div class="v" style="font-size:17px">{fc.state}</div></div>
  <div class="card"><div class="k">기동 상태</div><div class="v" style="font-size:17px">{bot.mv.state}</div></div>
  <div class="card"><div class="k">거리</div><div class="v" style="font-size:17px">{fmt(d,' m')}</div></div>
  <div class="card"><div class="k">적 기준 우리 위치</div><div class="v" style="font-size:17px">{asp_txt}</div></div>
  <div class="card"><div class="k">아군 HP</div><div class="v" style="font-size:17px">{tm.my_hp}</div></div>
  <div class="card"><div class="k">적 HP</div><div class="v" style="font-size:17px">{tm.enemy_hp}</div></div>
</div>

<div class="row">
  <div class="card"><div class="k">적 속도 / 선회</div><div class="v" style="font-size:17px">{trk.speed:.1f} m/s · {trk.turn_rate:+.0f}°/s</div></div>
  <div class="card"><div class="k">적 가속</div><div class="v" style="font-size:17px">{math.hypot(trk.acc[0],trk.acc[2]):.2f} m/s²</div></div>
  <div class="card"><div class="k">아군 속도</div><div class="v" style="font-size:17px">{tm.my_speed:.1f} m/s</div></div>
  <div class="card"><div class="k">예측 불확실성</div><div class="v" style="font-size:17px">횡 {sl:.1f} / 종 {sn:.1f} m</div></div>
  <div class="card"><div class="k">적 봇 ({ENEMY_BEHAVIOR})</div>
      <div class="v" style="font-size:17px;color:{ecolor}">{bot.enemy.state}
      <span style="font-size:11px;color:#8b949e"> {bot.enemy.calls}회</span></div></div>
</div>
{ewarn}

<div class="card"><div class="k">사격 해</div>
  <div style="font-size:14px;color:#79c0ff">{sol_txt}</div>
  <div style="font-size:11px;color:#8b949e;margin-top:4px">
    <a href="/export" style="color:#58a6ff">CSV 내려받기</a> ·
    <a href="/reset" style="color:#58a6ff">통계 초기화</a></div></div>

<h2>피탄 부위별 피해 (실측)</h2>
<table><thead><tr><th>부위</th><th>명중 수</th><th>평균 피해</th></tr></thead>
<tbody>{zrows}</tbody></table>

<h2>사격 기록</h2>
<table><thead><tr>
<th>#</th><th>시각</th><th>거리</th><th>착탄오차</th><th>사거리오차</th>
<th>P(hit)</th><th>속도 아/적</th><th>피탄부위</th><th>피해</th><th>결과</th>
</tr></thead><tbody>{rows}</tbody></table>
</body></html>"""


@app.route("/<path:unknown>", methods=["GET", "POST", "PUT", "PATCH", "DELETE"])
def catch_all(unknown):
    return jsonify({})


# ══════════════════════════════════════════════════════════
# 7. 적 전차 서버 (포트 5100)
# ══════════════════════════════════════════════════════════
enemy_app = Flask("enemy")
logging.getLogger("werkzeug").setLevel(logging.ERROR)


@enemy_app.route("/get_action", methods=["POST"])
def enemy_action():
    return jsonify(bot.enemy.act(request.get_json(force=True, silent=True) or {}))


@enemy_app.route("/update_bullet", methods=["POST"])
def enemy_bullet():
    return jsonify({"status": "OK"})


@enemy_app.route("/<path:unknown>", methods=["GET", "POST", "PUT", "PATCH", "DELETE"])
def enemy_any(unknown):
    return jsonify({})


def run_enemy():
    enemy_app.run(host="0.0.0.0", port=ENEMY_PORT, debug=False,
                  use_reloader=False)


if __name__ == "__main__":
    b = Ballistics()
    print("=" * 56)
    print("  Tank FCS v5 (양측 기동)   http://localhost:5000")
    print(f"  교전 포락 {b.min_range():.1f} ~ {b.max_range()[0]:.1f} m")
    print(f"  교전 밴드 {BAND_NEAR:.0f} ~ {BAND_FAR:.0f} m,  사격 임계 P(hit) >= {P_HIT_MIN}")
    print(f"  후방 기동 {'ON' if FLANK_ON else 'OFF'}")
    print(f"  적 전차 조종  포트 {ENEMY_PORT},  행동 '{ENEMY_BEHAVIOR}'")
    print("  시뮬레이터 설정:")
    print("    Mode=Simulation, Request Port=5000")
    print(f"    Enemy Request Port={ENEMY_PORT},  Use Enemy Server 체크")
    print("=" * 56)
    threading.Thread(target=run_enemy, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, debug=False)
