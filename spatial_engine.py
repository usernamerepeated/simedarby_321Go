"""
spatial_engine.py  —   WHERE is each car?
===================================================
Contract v1.3. main.py calls compute(raw) every 200 ms for every car and gets back
exactly the 7 Member-1 fields (contract.FIELD_OWNER["member1"]):

    junction_id, in_geofence, lane_role, distance_to_junction_m,
    approaching, ble_rssi, link_mode

How it works (simple version)
-----------------------------
1. Turn the car's GPS (lat, lon) into metres around the junction centre.
   We use two axes:   a = along the main road  (+a = side the NEAR-lane traffic comes from)
                      b = across the main road (-b = towards the side road)
2. Lane:
     * SIDE_ROAD  -> car is below the main road (b < -3.5 m) inside the side-road corridor
     * MAIN_NEAR / MAIN_FAR -> decided by the direction the car drives (GPS heading),
       but only above 10 km/h because GPS heading is junk when slow. Below that we keep
       the car's last known lane, or fall back to which side of the road centre it is on.
     * OUTSIDE    -> outside the 250 m geofence, or not on the main road / side road
3. Approaching: a NEAR-lane car drives towards -a, so it is approaching while a > 0.
                A FAR-lane car drives towards +a, so it is approaching while a < 0.
4. BLE RSSI is SIMULATED from distance (log-distance path-loss model). Real phone-to-phone
   BLE is not implemented in the prototype - say so if judges ask.

Run `python spatial_engine.py` to check your junctions.json and run a self-test.
"""
import json
import math
import os

from contract import SIDE_ROAD, MAIN_NEAR, MAIN_FAR, OUTSIDE, GEOFENCE_RADIUS_M

HERE = os.path.dirname(os.path.abspath(__file__))
JUNCTION_FILE = os.path.join(HERE, "junctions.json")

# ---------------- tuning (Member 1 owns these) ----------------
ROAD_HALF_WIDTH_M = 3.5        # 2 lanes x 3.5 m -> road edge is 3.5 m from the centre line
MAIN_ROAD_CORRIDOR_M = 15.0    # further than this sideways from the main road = not on it
SIDE_ZONE_HALF_WIDTH_M = 8.0   # side-road corridor: +-8 m around its centre line (GPS error margin)
SIDE_ZONE_DEPTH_M = 40.0       # up to 40 m back into the side road counts as "waiting"
HEADING_MIN_SPEED_KMH = 10.0   # trust GPS heading only above this speed
HEADING_TOLERANCE_DEG = 45.0   # heading within +-45 deg of a lane's direction -> that lane
BLE_TX_POWER_DBM = -59         # RSSI at 1 m (typical phone)
BLE_PATH_LOSS_N = 2.2          # 2 = free space, 2-3 = outdoors with cars around
BLE_MIN_DBM = -100             # below this the phone hears nothing
BLE_ALERT_DBM = -85            # offline mode: stronger than this = car is close (~15 m with this model)
FAR_AWAY_M = 99999.0           # distance reported when there is no GPS fix
M_PER_DEG = 111_320.0          # metres per degree of latitude


def _bearing(east, north):
    """Compass bearing (0 = north, 90 = east) of a vector."""
    return math.degrees(math.atan2(east, north)) % 360


def _angle_diff(x, y):
    d = abs(x - y) % 360
    return min(d, 360 - d)


class Junction:
    """One T-junction, built from 3 GPS points in junctions.json."""

    def __init__(self, cfg):
        self.id = cfg["junction_id"]
        self.name = cfg.get("name", self.id)
        self.lat0, self.lon0 = cfg["centre"]
        self._m_per_deg_lon = M_PER_DEG * math.cos(math.radians(self.lat0))

        rx, ry = self._east_north(*cfg["near_traffic_from"])
        sx, sy = self._east_north(*cfg["side_wait_point"])
        length = math.hypot(rx, ry)
        if length < 20:
            raise ValueError(f"{self.id}: near_traffic_from must be ~200 m from the centre")
        self.r = (rx / length, ry / length)          # unit vector along the main road
        n = (-self.r[1], self.r[0])                  # perpendicular...
        if n[0] * sx + n[1] * sy > 0:                # ...pointing AWAY from the side road
            n = (-n[0], -n[1])
        self.n = n

        self.far_bearing = _bearing(*self.r)          # far-lane cars drive towards +a
        self.near_bearing = (self.far_bearing + 180) % 360
        self.side_wait = self.to_local(*cfg["side_wait_point"])
        self.warnings = self._check()

    # ---- coordinate helpers ----
    def _east_north(self, lat, lon):
        return (lon - self.lon0) * self._m_per_deg_lon, (lat - self.lat0) * M_PER_DEG

    def to_local(self, lat, lon):
        """GPS -> (a, b) metres. a: along main road, b: across (negative = side-road side)."""
        e, nth = self._east_north(lat, lon)
        return (e * self.r[0] + nth * self.r[1], e * self.n[0] + nth * self.n[1])

    def from_local(self, a, b):
        """(a, b) metres -> GPS. Used by the simulator."""
        e = a * self.r[0] + b * self.n[0]
        nth = a * self.r[1] + b * self.n[1]
        return self.lat0 + nth / M_PER_DEG, self.lon0 + e / self._m_per_deg_lon

    def _check(self):
        w = []
        # Left-hand traffic: the near lane (driving towards -a) must be on its driver's LEFT,
        # i.e. on the side-road side. Left of direction (dx, dy) in east/north = (-dy, dx).
        dx, dy = -self.r[0], -self.r[1]
        left = (-dy, dx)
        if left[0] * self.n[0] + left[1] * self.n[1] > 0:
            w.append("near_traffic_from looks like it is on your LEFT. In Malaysia the near "
                     "lane's traffic comes from your RIGHT - swap it to the other side of the centre.")
        a, b = self.side_wait
        if not (-SIDE_ZONE_DEPTH_M <= b < -ROAD_HALF_WIDTH_M and abs(a) <= SIDE_ZONE_HALF_WIDTH_M):
            w.append(f"side_wait_point is at a={a:.1f} m, b={b:.1f} m - it should be 4-40 m straight "
                     "back from the centre into the side road.")
        return w


def _load(path=JUNCTION_FILE):
    with open(path, encoding="utf-8") as f:
        return [Junction(c) for c in json.load(f)["junctions"]]


JUNCTIONS = _load()
_last_lane = {}          # car_id -> last main-road lane (used when the car is slow)


def default_junction():
    return JUNCTIONS[0]


def reset():
    """Forget lane memory (call when the demo restarts a scenario)."""
    _last_lane.clear()


def ble_proximity_alert(fields):
    """For Member 2: True when offline and another car is close enough to act on BLE."""
    return fields.get("link_mode") == "OFFLINE_BLE" and fields.get("ble_rssi", BLE_MIN_DBM) > BLE_ALERT_DBM


def simulated_rssi(distance_m):
    d = max(distance_m, 1.0)
    return int(max(BLE_MIN_DBM, round(BLE_TX_POWER_DBM - 10 * BLE_PATH_LOSS_N * math.log10(d))))


def _main_lane(car_id, J, b, heading, speed):
    if heading is not None and speed is not None and speed >= HEADING_MIN_SPEED_KMH:
        if _angle_diff(heading, J.near_bearing) <= HEADING_TOLERANCE_DEG:
            return MAIN_NEAR
        if _angle_diff(heading, J.far_bearing) <= HEADING_TOLERANCE_DEG:
            return MAIN_FAR
    if car_id in _last_lane:
        return _last_lane[car_id]
    return MAIN_NEAR if b < 0 else MAIN_FAR       # near lane is on the side-road side


def compute(raw):
    """raw needs: car_id, lat, lon, speed_kmh, heading_deg (optional: online=False for BLE mode)."""
    car_id = raw.get("car_id")
    link_mode = "OFFLINE_BLE" if raw.get("online") is False else "ONLINE_GPS"
    lat, lon = raw.get("lat"), raw.get("lon")
    if lat is None or lon is None:                # no GPS fix yet
        return {"junction_id": None, "in_geofence": False, "lane_role": OUTSIDE,
                "distance_to_junction_m": FAR_AWAY_M, "approaching": False,
                "ble_rssi": BLE_MIN_DBM, "link_mode": link_mode}

    # nearest junction (supports several in junctions.json)
    best = None
    for J in JUNCTIONS:
        a, b = J.to_local(lat, lon)
        d = math.hypot(a, b)
        if best is None or d < best[3]:
            best = (J, a, b, d)
    J, a, b, d = best

    if d > GEOFENCE_RADIUS_M:
        lane = OUTSIDE
    elif -SIDE_ZONE_DEPTH_M <= b < -ROAD_HALF_WIDTH_M and abs(a - J.side_wait[0]) <= SIDE_ZONE_HALF_WIDTH_M:
        lane = SIDE_ROAD
    elif abs(b) > MAIN_ROAD_CORRIDOR_M:
        lane = OUTSIDE
    else:
        lane = _main_lane(car_id, J, b, raw.get("heading_deg"), raw.get("speed_kmh"))
        _last_lane[car_id] = lane

    if lane == MAIN_NEAR:
        approaching = a > 0
    elif lane == MAIN_FAR:
        approaching = a < 0
    else:
        approaching = False

    return {"junction_id": J.id, "in_geofence": d <= GEOFENCE_RADIUS_M, "lane_role": lane,
            "distance_to_junction_m": round(d, 1), "approaching": approaching,
            "ble_rssi": simulated_rssi(d), "link_mode": link_mode}


# =============================================================================
# Self-test:  python spatial_engine.py
# =============================================================================
if __name__ == "__main__":
    J = default_junction()
    print(f"Junction {J.id} - {J.name}")
    print(f"  near lane traffic drives towards {J.near_bearing:5.1f} deg")
    print(f"  far  lane traffic drives towards {J.far_bearing:5.1f} deg")
    print(f"  side wait point at a={J.side_wait[0]:.1f} m, b={J.side_wait[1]:.1f} m")
    for w in J.warnings:
        print("  WARNING:", w)
    tests = [  # (description, a, b, heading, speed, expected lane, expected approaching)
        ("red car waiting on side road", 0, -8, None, 0, SIDE_ROAD, False),
        ("near-lane car 150 m away, coming", 150, -1.75, J.near_bearing, 50, MAIN_NEAR, True),
        ("near-lane car passed the junction", -40, -1.75, J.near_bearing, 50, MAIN_NEAR, False),
        ("far-lane car 120 m away, coming", -120, 1.75, J.far_bearing, 50, MAIN_FAR, True),
        ("far-lane car crawling (heading ignored)", -60, 1.75, J.near_bearing, 3, MAIN_FAR, True),
        ("car 300 m away", 300, -1.75, J.near_bearing, 50, OUTSIDE, False),
        ("car on another street", 50, 60, 0, 40, OUTSIDE, False),
    ]
    ok = True
    for i, (desc, a, b, hdg, spd, lane, appr) in enumerate(tests):
        lat, lon = J.from_local(a, b)
        out = compute({"car_id": f"t{i}", "lat": lat, "lon": lon, "heading_deg": hdg, "speed_kmh": spd})
        good = out["lane_role"] == lane and out["approaching"] == appr
        ok &= good
        print(f"  {'PASS' if good else 'FAIL'}  {desc:42s} -> {out['lane_role']:9s} "
              f"approaching={out['approaching']!s:5s} d={out['distance_to_junction_m']:6.1f} m "
              f"rssi={out['ble_rssi']} dBm")
    print("All checks passed." if ok and not J.warnings else "Fix the items above.")
