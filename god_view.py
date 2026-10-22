"""
god_view.py  —   the live demo screen ("God View")
============================================================
Runs the whole system (simulated traffic -> Member 1 spatial_engine -> Member 2
sensor_fusion -> Member 3 state machine + ledger) and shows it live in your web browser:
map with cars coloured by state, offer pulses, gap bands, the two drivers' phone
screens, lane status, speed graph, event log, tokens - with sound and voice.

No extra install needed (Python standard library only). Run from the backend folder:

    python god_view.py              -> opens http://localhost:8000 in your browser
    python god_view.py --port 8080  -> use another port
    python god_view.py --no-browser -> don't open the browser automatically

Keys (in the browser window; click the page once first to enable sound):
    1 normal right turn   2 rogue driver (ABORT)   3 left turn   4 offline BLE sync
    5 driver ignores offer   6 two waiting cars   7 driver forces out without green
    R restart   Space pause   Left/Right speed   M sound on/off   F fullscreen

Until a ticket opens, the sim fast-forwards x4 through the 15 s "is the car really
trapped?" wait, then drops back to the speed you chose.
"""
import argparse
import json
import math
import os
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import contract
from contract import (SIDE_ROAD, MAIN_NEAR, MAIN_FAR, TICK_S, ACTIVE_TICKET_STATES,
                      UI_PROMPT_YIELD, UI_YIELD_CONFIRMED, UI_PAYOUT, REWARD_TOKENS,
                      ALERT_SPEC, CRITICAL_GAP_S, YIELD_GAP_S, MATCH_MIN_M, MATCH_MAX_M,
                      GEOFENCE_RADIUS_M)
from store import Store
import state_machine
import ledger
import run_scenarios as sim
import sim_geo
import spatial_engine
import offline_receipts

try:
    import sensor_fusion            # Member 2 (real); falls back to the mock if missing
except ImportError:
    sensor_fusion = None

HERE = os.path.dirname(os.path.abspath(__file__))
FAST_FORWARD_X = 4.0
SPEEDS = [0.25, 0.5, 1.0, 2.0, 4.0]
MAX_SIM_S = 150.0                   # traffic has long gone by then; sim pauses itself


# =============================================================================
# Scenarios (same traffic as run_scenarios.py)
# =============================================================================
def _red(turn="RIGHT", car_id="car_red_1", **extra):
    car = sim.SimCar(car_id, SIDE_ROAD, 0, 0, "red", user_id=f"user_{car_id}")
    car.intended_turn = turn
    for k, v in extra.items():
        setattr(car, k, v)
    return car


def _near(n=10, rule=lambda i: "yield"):
    return sim.traffic(MAIN_NEAR, n, 45, 50, rule)


def _far(n=10, rule=lambda i: "yield"):
    return sim.traffic(MAIN_FAR, n, 45, 50, rule, start=240)


SCENARIOS = {
    "1": ("Normal yield - right turn (both lanes)", lambda: [_red()] + _near() + _far()),
    "2": ("Rogue driver floors it during green -> STOP",
          lambda: [_red()] + _near() + _far(rule=lambda i: "rogue" if i == 2 else "yield")),
    "3": ("Left turn - only the near lane is asked",
          lambda: [_red("LEFT")] + _near() + _far(rule=lambda i: "ignore")),
    "5": ("Driver ignores the offer -> next car asked",
          lambda: [_red("LEFT")] + _near(12, lambda i: "ignore" if i == 1 else "yield")),
    "6": ("Two waiting cars - one ticket at a time",
          lambda: [_red("LEFT"), _red("LEFT", "car_red_2", queue_dist=12.0)] + _near(14)),
    "7": ("Driver forces out without green -> cancelled, no tokens",
          lambda: [_red(impatient_at=17.0)] + _near() + _far()),
    "8": ("No internet: phones save BLE receipts, press 4 to sync",
          lambda: [_offline(c) for c in [_red()] + _near() + _far()]),
}


def _offline(car):
    car.online = False
    return car


def demo_receipts(now):
    """Key 4: receipts two phones saved while offline (BLE), uploaded when back online."""
    return [
        {"receipt_id": f"r-exit-{int(now)}", "encounter_id": f"enc-{int(now)}", "role": "EXITER",
         "car_id": "car_red_9", "user_id": "user_red_9", "junction_id": "elmina_J1", "timestamp": now - 300},
        {"receipt_id": f"r-yield-{int(now)}", "encounter_id": f"enc-{int(now)}", "role": "YIELDER",
         "car_id": "car_blue_9", "user_id": "user_blue_9", "junction_id": "elmina_J1", "timestamp": now - 300},
        {"receipt_id": f"r-yield-{int(now)}", "encounter_id": f"enc-{int(now)}", "role": "YIELDER",
         "car_id": "car_blue_9", "user_id": "user_blue_9", "junction_id": "elmina_J1", "timestamp": now - 300},
        {"receipt_id": f"r-fake-{int(now)}", "encounter_id": "enc-nobody", "role": "YIELDER",
         "car_id": "car_cheat", "user_id": "user_cheat", "junction_id": "elmina_J1", "timestamp": now - 120},
    ]


RECEIPT_LABEL = {"r-exit": "red car's exit receipt", "r-yield": "blue car's yield receipt",
                 "r-fake": "receipt with no matching exit"}


# =============================================================================
# The demo engine (runs in a background thread)
# =============================================================================
class Demo:
    def __init__(self):
        self.lock = threading.Lock()
        self.speed_i = SPEEDS.index(1.0)
        self.paused = False
        self.J = spatial_engine.default_junction()
        self.load("1")

    # ---------- control ----------
    def load(self, key):
        title, build = SCENARIOS[key]
        self.key, self.title = key, title
        self.cars = build()
        self.red_ids = {c.car_id for c in self.cars if c.behaviour == "red"}
        self.store = Store()
        self.t = 0.0
        self.acc = 0.0
        self.events = deque(maxlen=40)
        self.history = {}                        # car_id -> deque of (t, speed, accel)
        self.red_wait = {}                       # car_id -> seconds stopped on the side road
        self.featured = None
        self.last_ticket = None
        self.ticket_seen = False
        self.offline = None
        self.done = False
        self.recorder = offline_receipts.OfflineRecorder() if key == "8" else None
        spatial_engine.reset()
        if sensor_fusion:
            sensor_fusion.reset()
        self.events.append(f"{0:6.1f}s  START   {title}")

    def command(self, k):
        with self.lock:
            if k in SCENARIOS:
                self.load(k)
            elif k == "r":
                self.load(self.key)
            elif k == "4":
                if self.recorder:                         # scenario 8: receipts the phones really saved
                    receipts, now = self.recorder.take_all(), self.t
                else:                                     # any other scenario: a canned example
                    now = time.time()
                    receipts = demo_receipts(now)
                by_id = {r["receipt_id"]: r for r in receipts}
                rows = []
                for rid, status in ledger.sync_offline_receipts(self.store, receipts, now):
                    label = next((v for p, v in RECEIPT_LABEL.items() if rid.startswith(p)),
                                 f"{by_id[rid]['car_id']} yield receipt")
                    rows.append({"label": label, "status": status})
                    self.events.append(f"{self.t:6.1f}s  OFFLINE {label}: {status}")
                if not rows:
                    rows.append({"label": "No yield receipts on the phones yet", "status": "NOTHING_TO_SYNC"})
                self.offline = {"rows": rows, "at": time.time()}
            elif k == "space":
                self.paused = not self.paused
            elif k == "faster":
                self.speed_i = min(self.speed_i + 1, len(SPEEDS) - 1)
            elif k == "slower":
                self.speed_i = max(self.speed_i - 1, 0)
            elif k.startswith("seek:"):             # used for testing / rehearsal: jump to time t
                target = float(k.split(":", 1)[1])
                while self.t < target and not self.done:
                    self._step()

    def fast_forwarding(self):
        return bool(self.red_ids) and not self.ticket_seen and self.t < 16

    # ---------- one 200 ms tick of the whole system ----------
    def _step(self):
        now = self.t
        for car in self.cars:
            raw = {**sim.raw_from_car(car, now), **sim_geo.gps_fields(car, now)}
            spatial = spatial_engine.compute(raw)                       # Member 1
            if sensor_fusion:                                           # Member 2
                motion = sensor_fusion.compute({**raw, **spatial}, now)
                car.kin = motion["kinematic_state"]                     # simulated drivers react to it
            else:
                motion = sim.mock_sensor(car, now)
            self.store.update_vehicle(car.car_id, {**raw, **spatial, **motion})
            h = self.history.setdefault(car.car_id, deque(maxlen=60))
            h.append((round(now, 1), round(car.speed_kmh, 1), round(motion.get("accel_long_ms2", 0.0), 2)))
            if car.car_id in self.red_ids:
                stopped = car.speed_kmh < 2 and spatial["lane_role"] == SIDE_ROAD
                self.red_wait[car.car_id] = self.red_wait.get(car.car_id, 0.0) + TICK_S if stopped else 0.0
        events = state_machine.tick(self.store, now)                    # Member 3
        if self.recorder:                                               # offline phones save receipts
            events += self.recorder.observe(self.store, now)
        for e in events:
            self.events.append(e)
            if " OPEN " in e:
                self.ticket_seen = True
        sim.drive(self.cars, self.store, now, TICK_S)                   # drivers react to their phones
        self.t = round(now + TICK_S, 3)
        if self.t >= MAX_SIM_S:
            self.done = True

    def run_forever(self):
        last = time.perf_counter()
        while True:
            time.sleep(0.01)
            now = time.perf_counter()
            dt, last = now - last, now
            with self.lock:
                if self.paused or self.done:
                    continue
                mult = FAST_FORWARD_X if self.fast_forwarding() else SPEEDS[self.speed_i]
                self.acc += dt * mult
                steps = 0
                while self.acc >= TICK_S and steps < 40:
                    self._step()
                    self.acc -= TICK_S
                    steps += 1

    # ---------- what the browser draws ----------
    def _local_dir(self, heading):
        if heading is None:
            return 0.0, 0.0
        e, n = math.sin(math.radians(heading)), math.cos(math.radians(heading))
        r, nn = self.J.r, self.J.n
        return e * r[0] + n * r[1], e * nn[0] + n * nn[1]

    def _current_ticket(self):
        tickets = list(self.store.tickets.values())
        active = [t for t in tickets if t["state"] in ACTIVE_TICKET_STATES]
        if active:
            return active[0]
        return tickets[-1] if tickets else None

    def state(self):
        with self.lock:
            V = self.store.vehicles
            vehicles = []
            for cid, v in V.items():
                if v.get("lat") is None:
                    continue
                a, b = self.J.to_local(v["lat"], v["lon"])
                if abs(a) > 330:
                    continue
                da, db = self._local_dir(v.get("heading_deg"))
                vehicles.append({
                    "id": cid, "a": round(a, 2), "b": round(b, 2), "da": round(da, 3), "db": round(db, 3),
                    "lane": v.get("lane_role"), "kin": v.get("kinematic_state"), "ui": v.get("ui_command"),
                    "speed": v.get("speed_kmh"), "dist": v.get("distance_to_junction_m"),
                    "approaching": v.get("approaching"), "red": cid in self.red_ids,
                    "rssi": v.get("ble_rssi")})

            t = self._current_ticket()
            ticket = None
            if t:
                lanes = []
                for ln in t["required_lanes"]:
                    info = t["lanes"][ln]
                    lead = V.get(info["lead_car"]) if info["lead_car"] else None
                    tta = None
                    if lead:
                        tta = round(lead["distance_to_junction_m"] / max(lead.get("speed_kmh", 0) / 3.6, 0.1), 1)
                    lanes.append({"lane": ln, "clear": info["clear"], "lead": info["lead_car"],
                                  "lead_dist": lead["distance_to_junction_m"] if lead else None,
                                  "tta": tta, "offered": info["offered_car"],
                                  "needed": YIELD_GAP_S if info["lead_car"] in t["yielders"] else CRITICAL_GAP_S})
                ticket = {"id": t["ticket_id"], "state": t["state"], "turn": t["turn"],
                          "red": t["red_car_id"], "yielders": list(t["yielders"]),
                          "declined": list(t["declined"]), "lanes": lanes,
                          "active": t["state"] in ACTIVE_TICKET_STATES,
                          "age": round(self.t - t["created_at"], 1)}

            # which main-road driver's phone to show
            pri = {UI_PROMPT_YIELD: 3, UI_YIELD_CONFIRMED: 2, UI_PAYOUT: 1}
            best = max((v for v in V.values() if v["car_id"] not in self.red_ids and v.get("ui_command") in pri),
                       key=lambda v: pri[v["ui_command"]], default=None)
            if best:
                self.featured = best["car_id"]
            fv = V.get(self.featured) if self.featured else None
            blue = None
            if fv:
                blue = {"id": fv["car_id"], "ui": fv.get("ui_command"), "speed": fv.get("speed_kmh"),
                        "dist": fv.get("distance_to_junction_m"), "lane": fv.get("lane_role"),
                        "history": list(self.history.get(fv["car_id"], []))}

            red_id = ticket["red"] if ticket and ticket["active"] else next(
                (c for c in sorted(self.red_ids) if c in V and V[c].get("lane_role") == SIDE_ROAD), None) \
                or (ticket["red"] if ticket else next(iter(sorted(self.red_ids)), None))
            rv = V.get(red_id) if red_id else None
            red = None
            if rv:
                red = {"id": red_id, "ui": rv.get("ui_command"), "kin": rv.get("kinematic_state"),
                       "speed": rv.get("speed_kmh"), "lane": rv.get("lane_role"),
                       "wait": round(self.red_wait.get(red_id, 0.0), 1)}

            wallets = sorted(({"user": u, "balance": w["balance"]} for u, w in self.store.wallets.items()),
                             key=lambda w: -w["balance"])
            offline = self.offline if self.offline and time.time() - self.offline["at"] < 9 else None
            return {
                "t": self.t, "title": self.title, "key": self.key, "paused": self.paused, "done": self.done,
                "speed": SPEEDS[self.speed_i], "ff": self.fast_forwarding(),
                "contract": contract.CONTRACT_VERSION, "m2": sensor_fusion is not None,
                "reward": REWARD_TOKENS, "geofence": GEOFENCE_RADIUS_M,
                "match": [MATCH_MIN_M, MATCH_MAX_M], "junction": self.J.name,
                "vehicles": vehicles, "ticket": ticket, "blue": blue, "red": red,
                "events": list(self.events)[-12:], "wallets": wallets[:5],
                "paid_total": sum(x["amount"] for x in self.store.ledger),
                "offline": offline,
            }


# =============================================================================
# Tiny web server: serves the page and the live state
# =============================================================================
DEMO = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):          # keep the terminal clean
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            with open(os.path.join(HERE, "god_view.html"), "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        elif path == "/state":
            self._send(200, json.dumps(DEMO.state()).encode(), "application/json")
        elif path == "/alerts":
            self._send(200, json.dumps(ALERT_SPEC).encode(), "application/json")
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        q = parse_qs(urlparse(self.path).query)
        DEMO.command(q.get("k", [""])[0])
        self._send(200, b"ok", "text/plain")


def main():
    global DEMO
    ap = argparse.ArgumentParser(description="T-junction yield - live God View")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    DEMO = Demo()
    threading.Thread(target=DEMO.run_forever, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    url = f"http://localhost:{args.port}"
    print(f"God View running at {url}   (contract v{contract.CONTRACT_VERSION}, "
          f"Member 1: REAL, Member 2: {'REAL' if sensor_fusion else 'mock'})")
    print("Click the page once to enable sound. Ctrl+C here to stop.")
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
