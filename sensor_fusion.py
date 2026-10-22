"""
sensor_fusion.py  —  Member 2 (Telemetry & Sensor Fusion)
=========================================================
Aligned to contract.py v1.1. Same logic as Member 2's original Phyphox script,
plus the fixes listed in the "M2 alignment" notes:

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

Live phone test (Member 2's original Phyphox mode):   python sensor_fusion.py
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
# Live phone mode (Member 2's original Phyphox loop)
# ==========================================
PHYPHOX_URL = ("http://192.168.0.191:8080/get"
               "?linX&linY&linZ&lin_time&gyrX&gyrY&gyrZ&gyr_time&locV")
# locV = GPS speed in m/s, if you add the Location (GPS) sensor to the same Phyphox
# experiment under that buffer name. Missing -> speed stays None.
CAR_ID = "car_blue_1"        # the phone plays this car in the demo
LOG_FILE = "sensor_log.csv"


def read_phyphox():
    req = urllib.request.Request(PHYPHOX_URL, headers={"User-Agent": "TJunction-Yield"})
    with urllib.request.urlopen(req, timeout=3) as resp:
        buf = json.loads(resp.read().decode("utf-8"))["buffer"]

    def latest(key, required=True):
        vals = buf.get(key, {}).get("buffer") or []
        vals = [v for v in vals if v is not None]
        if not vals:
            if required:
                raise ValueError(f"No data received for sensor: {key}")
            return None
        return float(vals[-1])

    loc_v = latest("locV", required=False)
    return {"car_id": CAR_ID,
            "lin_x": latest("linX"), "lin_y": latest("linY"), "lin_z": latest("linZ"),
            "lin_time": latest("lin_time"),
            "gyro_x": latest("gyrX"), "gyro_y": latest("gyrY"), "gyro_z": latest("gyrZ"),
            "speed_kmh": loc_v * 3.6 if loc_v is not None else None,
            "lane_role": "MAIN_NEAR", "approaching": True}   # placeholders until Member 1 is live


def run_live_phone():
    start, last_t = time.monotonic(), None
    with open(LOG_FILE, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "car_id", "lin_time", "speed_kmh",
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
                    out = compute(raw, time.monotonic() - start)
                    w.writerow([datetime.now().isoformat(timespec="milliseconds"), CAR_ID,
                                raw["lin_time"], raw["speed_kmh"], out["accel_long_ms2"],
                                out["gyro_ok"], out["kinematic_state"]])
                    f.flush()
                    print(f"{raw['lin_time']:.2f}s | a_long {out['accel_long_ms2']:+.2f} m/s^2 | "
                          f"gyro_ok {out['gyro_ok']} | {out['kinematic_state']}")
                    time.sleep(0.2)                    # 5 Hz, same as the backend tick
                except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as e:
                    print("Sensor read error:", e)
                    time.sleep(1)
        except KeyboardInterrupt:
            print(f"\nStopped. Log saved to {LOG_FILE}")


if __name__ == "__main__":
    run_live_phone()
