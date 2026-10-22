"""
sensor_fusion.py  —  Telemetry & Sensor Fusion
=========================================================

  * one detector PER CAR (dict keyed by car_id), not one global detector
  * compute(raw, now) -> {"kinematic_state", "accel_long_ms2", "gyro_ok"}  (the contract)
  * state names imported from contract.py (no hand-typed strings)
  * uses the `now` passed in (sim time or real time), not time.monotonic()
  * side-road zone comes from Member 1 (lane_role == SIDE_ROAD), not a local flag
  * TRAPPED clears when the car moves off
  * YIELDING / ABORT_HAZARD reset to CRUISING once the car has passed the junction
  * ABORT_HAZARD only while still approaching the junction
  * "speed is falling" compares with the speed ~1.5 s ago (GPS only updates ~1 Hz,
    so comparing with the previous sample was almost always "not falling")
  * backup path: GPS speed drop >= 8 km/h within 3 s also counts as YIELDING
    (gentle lift-off rarely reaches -1.2 m/s^2)

Live phone test (Phyphox, with GPS checks + real lane detection):   python sensor_fusion.py
Find the phone's forward axis from a recorded drive:  python sensor_fusion.py --calibrate sensor_log.csv
"""
import csv
import json
import math
import time
import urllib.request
from collections import deque
from datetime import datetime

from contract import CRUISING, TRAPPED, YIELDING, ABORT_HAZARD, SIDE_ROAD

# ==========================================
# Tuning (Member 2 owns these numbers)
# ==========================================
FILTER_WINDOW = 5            # moving-average samples (5 samples @ 5 Hz = 1 s)
TRAPPED_SPEED_KMH = 2.0      # below this = "stopped"
TRAPPED_TIME_S = 15.0        # stopped this long in the side-road zone -> TRAPPED
YIELD_ACCEL_MS2 = -1.2       # longitudinal decel threshold (tune with real drives)
YIELD_HOLD_S = 1.5           # decel must last this long
SPEED_FALL_KMH = 1.0         # speed must have fallen at least this much over YIELD_HOLD_S
GPS_DROP_KMH = 8.0           # backup: speed dropped this much ...
GPS_DROP_WINDOW_S = 3.0      # ... within this window -> YIELDING even without accel
ABORT_ACCEL_MS2 = 2.0        # floored it while yielding
GYRO_MAX_DPS = 45.0          # above this the phone is being handled/shaken -> ignore accel

# Phone mount calibration: which phone axis points to the car's FRONT, and its sign.
# Do one test brake with the phone mounted: the axis that goes most NEGATIVE while
# braking is the long axis with sign +1 (if it goes most POSITIVE, use sign -1).
LONG_AXIS = "y"              # "x", "y" or "z"
LONG_SIGN = 1


class MovingAverage:
    def __init__(self, window_size=FILTER_WINDOW):
        self.values = deque(maxlen=window_size)

    def update(self, value):
        self.values.append(value)
        return sum(self.values) / len(self.values)


class VehicleDetector:
    """Turns one car's speed + IMU stream into CRUISING / TRAPPED / YIELDING / ABORT_HAZARD."""

    def __init__(self):
        self.state = CRUISING
        self.stopped_since = None
        self.braking_since = None
        self.speed_hist = deque()                     # (t, speed_kmh), last few seconds
        self.filters = {a: MovingAverage() for a in "xyz"}

    def _speed_change(self, now, window_s):
        """speed now minus speed `window_s` ago (negative = slowing). None if not enough history."""
        old = [v for t, v in self.speed_hist if now - t >= window_s]
        if not old or not self.speed_hist:
            return None
        return self.speed_hist[-1][1] - old[-1]

    def update(self, speed_kmh, accel_long, gyro_dps, in_minor_zone, approaching, now):
        gyro_ok = gyro_dps is None or gyro_dps <= GYRO_MAX_DPS

        if speed_kmh is not None:
            self.speed_hist.append((now, speed_kmh))
            while self.speed_hist and now - self.speed_hist[0][0] > GPS_DROP_WINDOW_S + 1:
                self.speed_hist.popleft()

        # ---- 1. TRAPPED: stopped in the side-road zone for 15 s ----
        if in_minor_zone and speed_kmh is not None and speed_kmh < TRAPPED_SPEED_KMH:
            if self.stopped_since is None:
                self.stopped_since = now
            if now - self.stopped_since >= TRAPPED_TIME_S:
                self.state = TRAPPED
            return self.state, gyro_ok
        self.stopped_since = None
        if self.state == TRAPPED:
            self.state = CRUISING                     # moved off -> no longer trapped

        # ---- 2. Passed the junction (or not heading to it): reset the latch ----
        if not approaching:
            self.state, self.braking_since = CRUISING, None
            return self.state, gyro_ok

        # ---- 3. ABORT_HAZARD: floored it while yielding (only while approaching) ----
        if self.state == YIELDING:
            if accel_long is not None and accel_long > ABORT_ACCEL_MS2 and gyro_ok:
                self.state = ABORT_HAZARD
            return self.state, gyro_ok                # YIELDING stays latched otherwise

        # ---- 4. YIELDING: sustained decel AND speed really falling ----
        if self.state == CRUISING:
            change_hold = self._speed_change(now, YIELD_HOLD_S)
            speed_falling = change_hold is None or change_hold <= -SPEED_FALL_KMH
            braking = accel_long is not None and accel_long < YIELD_ACCEL_MS2 and gyro_ok
            if braking:
                if self.braking_since is None:
                    self.braking_since = now
                if now - self.braking_since >= YIELD_HOLD_S and speed_falling:
                    self.state = YIELDING
            else:
                self.braking_since = None
            # backup path: GPS speed alone shows a clear slow-down
            change_window = self._speed_change(now, GPS_DROP_WINDOW_S)
            if change_window is not None and change_window <= -GPS_DROP_KMH:
                self.state = YIELDING

        return self.state, gyro_ok

    def long_accel_from_imu(self, lin_x, lin_y, lin_z):
        f = {"x": self.filters["x"].update(lin_x),
             "y": self.filters["y"].update(lin_y),
             "z": self.filters["z"].update(lin_z)}
        return LONG_SIGN * f[LONG_AXIS]


# ==========================================
# Contract interface — called by main.py every 200 ms
# ==========================================
_detectors = {}


def compute(raw, now):
    """
    raw (contract v1.1): car_id, speed_kmh, lin_x/lin_y/lin_z (m/s^2, gravity removed),
                         gyro_x/gyro_y/gyro_z (rad/s), + Member 1's lane_role, approaching
    returns: {"kinematic_state", "accel_long_ms2", "gyro_ok"}
    """
    det = _detectors.setdefault(raw["car_id"], VehicleDetector())

    if all(raw.get(k) is not None for k in ("lin_x", "lin_y", "lin_z")):
        accel_long = det.long_accel_from_imu(raw["lin_x"], raw["lin_y"], raw["lin_z"])
    else:
        accel_long = None

    if all(raw.get(k) is not None for k in ("gyro_x", "gyro_y", "gyro_z")):
        gyro_dps = math.degrees(math.sqrt(raw["gyro_x"] ** 2 + raw["gyro_y"] ** 2 + raw["gyro_z"] ** 2))
    else:
        gyro_dps = None

    state, gyro_ok = det.update(
        speed_kmh=raw.get("speed_kmh"),
        accel_long=accel_long,
        gyro_dps=gyro_dps,
        in_minor_zone=raw.get("lane_role") == SIDE_ROAD,
        approaching=raw.get("approaching", True),
        now=now,
    )
    return {"kinematic_state": state,
            "accel_long_ms2": round(accel_long, 3) if accel_long is not None else 0.0,
            "gyro_ok": gyro_ok}


def reset():
    """Forget all cars (used when the demo controller restarts a scenario)."""
    _detectors.clear()


# ==========================================
# Live phone mode (Member 2's Phyphox loop, now with his GPS checks + the real
# location engine, so a real phone runs the same pipeline as the demo)
# ==========================================
PHYPHOX_URL = ("http://192.168.0.191:8080/get"          # CHANGE to your phone's IP
               "?linX&linY&linZ&lin_time&gyrX&gyrY&gyrZ&gyr_time"
               "&locLat&locLon&locV&loc_time&locDir&locAccuracy")
CAR_ID = "car_blue_1"        # the phone plays this car in the demo
LOG_FILE = "sensor_log.csv"
GPS_MAX_AGE_S = 5.0          # reject a GPS fix not refreshed within 5 s (from ai_traffic_cop.py)
GPS_MAX_ACCURACY_M = 25.0    # reject a GPS fix worse than 25 m (from ai_traffic_cop.py)
_gps = {"last_fix_time": None, "last_update_wall": None}


def read_phyphox():
    req = urllib.request.Request(PHYPHOX_URL, headers={"User-Agent": "TJunction-Yield"})
    with urllib.request.urlopen(req, timeout=3) as resp:
        buf = json.loads(resp.read().decode("utf-8"))["buffer"]

    def latest(key, required=True):
        item = buf.get(key, {})
        vals = item.get("buffer", []) if isinstance(item, dict) else []
        for v in reversed(vals):
            try:
                x = float(v)
                if math.isfinite(x):
                    return x
            except (TypeError, ValueError):
                continue
        if required:
            raise ValueError(f"No data received for sensor: {key}")
        return None

    lat, lon = latest("locLat", False), latest("locLon", False)
    speed_ms, fix_t = latest("locV", False), latest("loc_time", False)
    heading, acc = latest("locDir", False), latest("locAccuracy", False)

    wall = time.monotonic()
    present = None not in (lat, lon, speed_ms, fix_t) and speed_ms >= 0
    if present and (_gps["last_fix_time"] is None or fix_t > _gps["last_fix_time"]):
        _gps["last_fix_time"], _gps["last_update_wall"] = fix_t, wall
    fresh = _gps["last_update_wall"] is not None and wall - _gps["last_update_wall"] <= GPS_MAX_AGE_S
    accurate = acc is None or acc <= GPS_MAX_ACCURACY_M
    gps_ok = present and fresh and accurate

    return {"car_id": CAR_ID, "user_id": f"user_{CAR_ID}",
            "lin_x": latest("linX"), "lin_y": latest("linY"), "lin_z": latest("linZ"),
            "lin_time": latest("lin_time"),
            "gyro_x": latest("gyrX"), "gyro_y": latest("gyrY"), "gyro_z": latest("gyrZ"),
            "lat": lat if gps_ok else None, "lon": lon if gps_ok else None,
            "speed_kmh": speed_ms * 3.6 if gps_ok else None,
            "heading_deg": heading if gps_ok else None,
            "gps_ok": gps_ok, "gps_accuracy_m": acc}


def run_live_phone():
    try:
        import spatial_engine            # Member 1: real lane / distance from the phone's GPS
    except ImportError:
        spatial_engine = None
    start, last_t = time.monotonic(), None
    with open(LOG_FILE, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "car_id", "lin_time", "lin_x", "lin_y", "lin_z", "speed_kmh",
                    "gps_ok", "gps_accuracy_m", "lane_role", "distance_to_junction_m",
                    "accel_long_ms2", "gyro_ok", "kinematic_state"])
        print("Live Phyphox mode. Ctrl+C to stop.")
        try:
            while True:
                try:
                    raw = read_phyphox()
                    if last_t is not None and raw["lin_time"] <= last_t:
                        time.sleep(0.05)
                        continue
                    last_t = raw["lin_time"]
                    if spatial_engine:
                        raw.update(spatial_engine.compute(raw))
                    else:                                          # no location engine: assume main road
                        raw.update({"lane_role": "MAIN_NEAR", "approaching": True,
                                    "distance_to_junction_m": None})
                    out = compute(raw, time.monotonic() - start)
                    w.writerow([datetime.now().isoformat(timespec="milliseconds"), CAR_ID,
                                raw["lin_time"], raw["lin_x"], raw["lin_y"], raw["lin_z"],
                                raw["speed_kmh"], raw["gps_ok"], raw["gps_accuracy_m"],
                                raw.get("lane_role"), raw.get("distance_to_junction_m"),
                                out["accel_long_ms2"], out["gyro_ok"], out["kinematic_state"]])
                    f.flush()
                    spd = f"{raw['speed_kmh']:.1f} km/h" if raw["speed_kmh"] is not None else "GPS --"
                    print(f"{raw['lin_time']:7.2f}s | {spd:>10} | {raw.get('lane_role')} | "
                          f"a_long {out['accel_long_ms2']:+.2f} m/s^2 | gyro_ok {out['gyro_ok']} | "
                          f"{out['kinematic_state']}")
                    time.sleep(0.2)                                # 5 Hz, same as the backend tick
                except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as e:
                    print("Sensor read error:", e)
                    time.sleep(1)
        except KeyboardInterrupt:
            print(f"\nStopped. Log saved to {LOG_FILE}")


# ==========================================
# Phone-mount calibration:  python sensor_fusion.py --calibrate sensor_log.csv
# Drive with the phone in its real mount: cruise, brake firmly, accelerate. The axis
# whose acceleration best follows the GPS speed change is the car's forward axis.
# ==========================================
def calibrate_axis(csv_path):
    def col(row, *names):
        for n in names:
            v = row.get(n)
            if v not in (None, "", "None"):
                try:
                    return float(v)
                except ValueError:
                    pass
        return None

    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    pts = []
    for row in rows:
        t = col(row, "elapsed_seconds", "lin_time")
        v = col(row, "gps_speed_kmh", "speed_kmh")
        ax = {a: col(row, f"lin_{a}_filtered", f"lin_{a}") for a in "xyz"}
        if t is not None and v is not None and None not in ax.values():
            pts.append((t, v / 3.6, ax))
    # speed change per second (m/s^2) between GPS samples at least 0.8 s apart
    samples, j = [], 0
    for i in range(len(pts)):
        while j < i and pts[i][0] - pts[j][0] > 2.0:
            j += 1
        if pts[i][0] - pts[j][0] >= 0.8 and pts[i][1] != pts[j][1]:
            dvdt = (pts[i][1] - pts[j][1]) / (pts[i][0] - pts[j][0])
            samples.append((dvdt, pts[i][2]))
    speeds = [p[1] * 3.6 for p in pts]
    if len(samples) < 15 or not speeds or max(speeds) - min(speeds) < 10:
        print(f"Not enough driving data in {csv_path}: {len(samples)} usable samples, speed range "
              f"{(max(speeds) - min(speeds)) if speeds else 0:.1f} km/h. Record a real drive with the "
              f"phone mounted (cruise, brake firmly, accelerate), then try again.")
        return None

    def corr(xs, ys):
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        sxx = sum((x - mx) ** 2 for x in xs)
        syy = sum((y - my) ** 2 for y in ys)
        return sxy / math.sqrt(sxx * syy) if sxx and syy else 0.0

    dv = [s[0] for s in samples]
    scores = {a: corr([s[1][a] for s in samples], dv) for a in "xyz"}
    best = max(scores, key=lambda a: abs(scores[a]))
    sign = 1 if scores[best] > 0 else -1
    for a in "xyz":
        print(f"  axis {a}: correlation with speed change = {scores[a]:+.2f}")
    print(f"\nSet in sensor_fusion.py:   LONG_AXIS = \"{best}\"   LONG_SIGN = {sign}")
    if abs(scores[best]) < 0.5:
        print("Warning: weak match (< 0.5). Mount the phone firmly and record a longer drive.")
    return best, sign


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 3 and sys.argv[1] == "--calibrate":
        calibrate_axis(sys.argv[2])
    else:
        run_live_phone()
