"""
run_scenarios.py  —  test the state machine WITHOUT waiting for Member 1 & 2
=============================================================================
This file pretends to be Member 1 (positions/lanes) and Member 2 (TRAPPED /
YIELDING / ABORT_HAZARD) with a very simple 1-D traffic simulation, so you can
prove your state machine works on day 1. Later, replace the two mock_* functions
with the real spatial_engine / sensor_fusion functions from your teammates.

Run:   python run_scenarios.py            (all scenarios)
       python run_scenarios.py rogue      (one scenario)
"""
import sys
import time as _time

from contract import (SIDE_ROAD, MAIN_NEAR, MAIN_FAR, CRUISING, TRAPPED, YIELDING,
                      ABORT_HAZARD, TICK_S, GEOFENCE_RADIUS_M, UI_PROMPT_YIELD,
                      UI_GO_GREEN, UI_ABORT_RED)
from store import Store
import state_machine
import ledger

JUNCTION = "elmina_J1"


class SimCar:
    """A car on a straight line. pos = metres BEFORE the junction (negative = already passed)."""

    def __init__(self, car_id, lane, pos, speed_kmh, behaviour="yield", user_id=None):
        self.car_id, self.lane, self.pos = car_id, lane, pos
        self.speed_kmh, self.cruise_kmh = speed_kmh, speed_kmh
        self.behaviour = behaviour          # "yield" | "ignore" | "rogue" | "red"
        self.user_id = user_id or f"user_{car_id}"
        self.accel = 0.0                    # m/s^2 (what the phone accelerometer would feel)
        self.offer_seen_at = None
        self.kin = CRUISING                 # mock Member 2 output
        self.stopped_since = 0.0
        self.go_at = None                   # red car: when it starts moving
        self.exited = False
        self.intended_turn = "RIGHT"


def raw_from_car(car, now):
    """What a phone would send (contract RAW_FIELDS). The simulated phone is mounted so
    its y-axis points forward, i.e. lin_y = longitudinal acceleration."""
    return {"car_id": car.car_id, "user_id": car.user_id, "intended_turn": car.intended_turn,
            "speed_kmh": round(car.speed_kmh, 1),
            "lin_x": 0.0, "lin_y": car.accel, "lin_z": 0.0,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0, "updated_at": now}


# ---------------------------------------------------------------------------
# MOCK Member 1: spatial fields
# ---------------------------------------------------------------------------
def mock_spatial(car):
    if car.behaviour == "red":
        lane = SIDE_ROAD if not car.exited else (MAIN_FAR if car.intended_turn == "RIGHT" else MAIN_NEAR)
        return {"junction_id": JUNCTION, "in_geofence": True, "lane_role": lane,
                "distance_to_junction_m": getattr(car, "queue_dist", 5.0), "approaching": False,
                "ble_rssi": -45, "link_mode": "ONLINE_GPS"}
    d = abs(car.pos)
    rssi = round(-45 - 50 * min(d, 200) / 200)          # -45 dBm at 0 m ... -95 dBm at 200 m
    return {"junction_id": JUNCTION, "in_geofence": d <= GEOFENCE_RADIUS_M,
            "lane_role": car.lane, "distance_to_junction_m": round(d, 1),
            "approaching": car.pos > 0, "ble_rssi": rssi, "link_mode": "ONLINE_GPS"}


# ---------------------------------------------------------------------------
# MOCK Member 2: kinematic state
# ---------------------------------------------------------------------------
def mock_sensor(car, now):
    if car.behaviour == "red":
        if car.speed_kmh < 2.0:
            if now - car.stopped_since >= 15:
                car.kin = TRAPPED
        else:
            car.stopped_since, car.kin = now, CRUISING
    else:
        if car.pos <= 0:
            car.kin = CRUISING                         # passed the junction -> reset latch
        elif car.kin in (YIELDING,) and car.accel > 2.0:
            car.kin = ABORT_HAZARD                     # floored it while yielding
        elif car.kin == CRUISING and car.accel < -1.2:
            car.kin = YIELDING                         # latched until passed / abort
    return {"kinematic_state": car.kin, "accel_long_ms2": car.accel, "gyro_ok": True}


# ---------------------------------------------------------------------------
# Very simple driver behaviour + physics
# ---------------------------------------------------------------------------
def drive(cars, store, now, dt):
    red_ui = next((store.vehicles[c.car_id].get("ui_command")
                   for c in cars if c.behaviour == "red"), None)
    for car in cars:
        ui = store.vehicles.get(car.car_id, {}).get("ui_command")
        if car.behaviour == "red":
            if ui == UI_GO_GREEN and car.go_at is None:
                car.go_at = now + 0.6                  # driver reaction time
            impatient_at = getattr(car, "impatient_at", None)
            if impatient_at is not None and now >= impatient_at and car.go_at is None:
                car.go_at = now                        # forces out WITHOUT waiting for green
            if ui == UI_ABORT_RED:
                car.go_at, car.speed_kmh = None, 0.0   # slam brakes, back to waiting
            if car.go_at is not None and now >= car.go_at:
                car.speed_kmh = min(car.speed_kmh + 3, 20)
                if now - car.go_at > 2.0:
                    car.exited = True                  # now on the main road
            continue

        target = car.cruise_kmh
        if ui == UI_PROMPT_YIELD and car.offer_seen_at is None:
            car.offer_seen_at = now
        if car.behaviour in ("yield", "rogue") and car.offer_seen_at is not None \
                and car.pos > 0 and now - car.offer_seen_at > 1.5:
            target = 15                                # lift off + brake gently
        if car.behaviour == "rogue" and red_ui == UI_GO_GREEN and car.kin == YIELDING:
            car.rogue_at = getattr(car, "rogue_at", None) or now
            if now - car.rogue_at > 1.0:
                target = 90                            # driver changes mind and floors it
        if car.pos <= 0:
            target = car.cruise_kmh                    # passed junction -> speed back up

        # car-following: don't drive through the car in front
        ahead = [o for o in cars if o.lane == car.lane and o.behaviour != "red" and o.pos < car.pos]
        if ahead:
            front = max(ahead, key=lambda o: o.pos)
            if car.pos - front.pos < 20:
                target = min(target, front.speed_kmh)

        old = car.speed_kmh
        if target < car.speed_kmh:
            car.speed_kmh = max(target, car.speed_kmh - 1.8 * 3.6 * dt)   # -1.8 m/s^2
        elif target > car.speed_kmh:
            rate = 3.0 if target == 90 else 1.0
            car.speed_kmh = min(target, car.speed_kmh + rate * 3.6 * dt)
        car.accel = (car.speed_kmh - old) / 3.6 / dt
        car.pos -= car.speed_kmh / 3.6 * dt


def traffic(lane, n, spacing_m, speed, behaviours, start=260):
    return [SimCar(f"{'near' if lane == MAIN_NEAR else 'far'}_{i+1}", lane,
                   start + i * spacing_m, speed, behaviours(i)) for i in range(n)]


# ---------------------------------------------------------------------------
# Scenario runner
# ---------------------------------------------------------------------------
def run(name, cars, duration=70, verbose=True, dump_at=None):
    print(f"\n{'=' * 70}\nSCENARIO: {name}   [Member 2: {'REAL' if REAL_M2 else 'mock'}]\n{'=' * 70}")
    if REAL_M2:
        REAL_M2.reset()
    store = Store()
    now = 0.0
    while now <= duration:
        for car in cars:                               # 1) phones send data
            fields = raw_from_car(car, now)
            fields.update(mock_spatial(car))           # 2) Member 1 adds spatial fields
            if REAL_M2:                                # 3) Member 2 adds kinematic state
                fields.update(REAL_M2.compute(fields, now))
                car.kin = fields["kinematic_state"]    # drivers in the sim react to it
            else:
                fields.update(mock_sensor(car, now))
            store.update_vehicle(car.car_id, fields)
        for e in state_machine.tick(store, now):       # 4) YOUR state machine
            if verbose:
                print(e)
        if dump_at and abs(now - dump_at) < TICK_S / 2:
            store.save_json("mock_snapshot.json")
            print(f"        (saved mock_snapshot.json at t={now:.1f}s for the frontend team)")
        drive(cars, store, now, TICK_S)                # 5) drivers react to their screens
        now = round(now + TICK_S, 3)
    print("Wallets:", {u: w["balance"] for u, w in store.wallets.items()} or "none")
    return store


def scenario_normal():
    red = SimCar("car_red_1", SIDE_ROAD, 0, 0, "red")
    cars = [red] + traffic(MAIN_NEAR, 8, 45, 50, lambda i: "yield") \
                 + traffic(MAIN_FAR, 8, 45, 50, lambda i: "yield", start=240)
    return run("1. Normal yield, RIGHT turn (needs both lanes)", cars, dump_at=21.0)


def scenario_rogue():
    red = SimCar("car_red_1", SIDE_ROAD, 0, 0, "red")
    cars = [red] + traffic(MAIN_NEAR, 8, 45, 50, lambda i: "yield") \
                 + traffic(MAIN_FAR, 8, 45, 50, lambda i: "rogue" if i == 2 else "yield", start=240)
    return run("2. Rogue driver accelerates during green -> ABORT_RED", cars, duration=80)


def scenario_left():
    red = SimCar("car_red_1", SIDE_ROAD, 0, 0, "red")
    red.intended_turn = "LEFT"
    cars = [red] + traffic(MAIN_NEAR, 8, 45, 50, lambda i: "yield") \
                 + traffic(MAIN_FAR, 8, 45, 50, lambda i: "ignore", start=240)
    return run("3. LEFT turn (only near lane needs to yield)", cars)


def scenario_ignore():
    red = SimCar("car_red_1", SIDE_ROAD, 0, 0, "red")
    red.intended_turn = "LEFT"
    cars = [red] + traffic(MAIN_NEAR, 10, 45, 50, lambda i: "ignore" if i == 1 else "yield")
    return run("4. First car ignores the offer -> next car is asked", cars, duration=80)


def scenario_offline():
    print(f"\n{'=' * 70}\nSCENARIO: 5. Offline BLE receipts synced later\n{'=' * 70}")
    store = Store()
    now = _time.time()
    receipts = [
        {"receipt_id": "r1", "encounter_id": "enc-1", "role": "EXITER", "car_id": "car_red_1",
         "user_id": "user_red", "junction_id": JUNCTION, "timestamp": now - 600},
        {"receipt_id": "r2", "encounter_id": "enc-1", "role": "YIELDER", "car_id": "car_blue_1",
         "user_id": "user_blue1", "junction_id": JUNCTION, "timestamp": now - 600},
        {"receipt_id": "r2", "encounter_id": "enc-1", "role": "YIELDER", "car_id": "car_blue_1",
         "user_id": "user_blue1", "junction_id": JUNCTION, "timestamp": now - 600},   # duplicate
        {"receipt_id": "r3", "encounter_id": "enc-9", "role": "YIELDER", "car_id": "car_cheat",
         "user_id": "user_cheat", "junction_id": JUNCTION, "timestamp": now - 300},   # no red car
    ]
    for rid, status in ledger.sync_offline_receipts(store, receipts, now):
        print(f"   receipt {rid}: {status}")
    print("Wallets:", {u: w["balance"] for u, w in store.wallets.items()})


def scenario_queue():
    red1 = SimCar("car_red_1", SIDE_ROAD, 0, 0, "red")
    red2 = SimCar("car_red_2", SIDE_ROAD, 0, 0, "red", user_id="user_car_red_2")
    red1.intended_turn = red2.intended_turn = "LEFT"
    red2.queue_dist = 12.0                             # second in the queue, further from the junction
    cars = [red1, red2] + traffic(MAIN_NEAR, 14, 45, 50, lambda i: "yield")
    return run("6. Two red cars queued -> one ticket at a time, front car first", cars, duration=90)


def scenario_no_green():
    red = SimCar("car_red_1", SIDE_ROAD, 0, 0, "red")
    red.impatient_at = 17.0                            # driver gives up waiting and forces out
    cars = [red] + traffic(MAIN_NEAR, 8, 45, 50, lambda i: "yield") \
                 + traffic(MAIN_FAR, 8, 45, 50, lambda i: "yield", start=240)
    return run("7. Red car forces out without green -> CANCELLED, nobody paid", cars, duration=40)


def scenario_empty():
    red = SimCar("car_red_1", SIDE_ROAD, 0, 0, "red")
    return run("8. No traffic at all -> instant green, nobody to pay", [red], duration=30)


SCENARIOS = {"normal": scenario_normal, "rogue": scenario_rogue, "left": scenario_left,
             "ignore": scenario_ignore, "offline": scenario_offline, "queue": scenario_queue,
             "no_green": scenario_no_green, "empty": scenario_empty}

REAL_M2 = None

if __name__ == "__main__":
    args = sys.argv[1:]
    if "--real-m2" in args:                            # test with Member 2's real sensor_fusion.py
        args.remove("--real-m2")
        import sensor_fusion as REAL_M2
    picks = args or list(SCENARIOS)
    for p in picks:
        SCENARIOS[p]()
