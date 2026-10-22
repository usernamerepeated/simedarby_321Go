"""
main.py  —  the real-time backend loop (used for the live demo)
==================================================================================
Every 200 ms:
   1. get raw data for each car         (simulator now, real phones later)
   2. Member 1  -> spatial fields       (spatial_engine.compute(raw))
   3. Member 2  -> kinematic state      (sensor_fusion.compute(raw))
   4. Member 3  -> state machine tick   (state_machine.tick(store, now))
   5. publish snapshot                  (god-view reads store.snapshot(); optional Firebase)

Until your teammates' modules exist, the mock versions from run_scenarios.py are used.
When they are ready, they only need to provide one function each
(NOTE: real spatial_engine needs lat/lon in raw -> Member 1 also upgrades the
 simulator so each SimCar outputs lat/lon along the real lane coordinates):

    # spatial_engine.py (Member 1)
    def compute(raw: dict) -> dict:   # returns Member 1 fields from contract.FIELD_OWNER
    # sensor_fusion.py  (Member 2)
    def compute(raw_plus_m1: dict, now: float) -> dict:   # raw + Member 1 fields in, Member 2 fields out

Run:  python main.py              (simulated cars, prints events live)
      python main.py --firebase https://<your-db>.firebasedatabase.app
"""
import sys
import time

import contract

from contract import TICK_S, SIDE_ROAD, MAIN_NEAR, MAIN_FAR
from store import Store
import state_machine
import run_scenarios as sim   # mock Member 1 + Member 2 + fake traffic
import sim_geo                # simulated phone GPS (lat/lon/heading) for the real spatial_engine

# ---- try to load teammates' real modules; fall back to mocks ----
try:
    import spatial_engine          # Member 1
except ImportError:
    spatial_engine = None
try:
    import sensor_fusion           # Member 2
except ImportError:
    sensor_fusion = None


def build_demo_cars(turn="RIGHT", rogue=False):
    red = sim.SimCar("car_red_1", SIDE_ROAD, 0, 0, "red")
    red.intended_turn = turn
    near = sim.traffic(MAIN_NEAR, 10, 45, 50, lambda i: "yield")
    far = sim.traffic(MAIN_FAR, 10, 45, 50,
                      lambda i: "rogue" if (rogue and i == 2) else "yield", start=240)
    return [red] + near + far


def main():
    store = Store()
    if "--firebase" in sys.argv:
        store.enable_firebase(sys.argv[sys.argv.index("--firebase") + 1])
        print("Mirroring to Firebase /live")
    cars = build_demo_cars(rogue="--rogue" in sys.argv)
    start = time.time()
    print(f"Backend running, contract v{contract.CONTRACT_VERSION}. "
          f"Member 1: {'REAL' if spatial_engine else 'mock'}, "
          f"Member 2: {'REAL' if sensor_fusion else 'mock'}. Ctrl+C to stop.")
    try:
        while True:
            now = time.time() - start
            for car in cars:
                raw = {**sim.raw_from_car(car, now), **sim_geo.gps_fields(car, now)}
                spatial = spatial_engine.compute(raw) if spatial_engine else sim.mock_spatial(car)
                # Member 2 needs Member 1's lane_role + approaching, so pass both
                motion = (sensor_fusion.compute({**raw, **spatial}, now) if sensor_fusion
                          else sim.mock_sensor(car, now))
                car.kin = motion["kinematic_state"]   # simulated drivers react to Member 2's state
                store.update_vehicle(car.car_id, {**raw, **spatial, **motion})
            for e in state_machine.tick(store, now):
                print(e)
            store.push_to_firebase(now)
            sim.drive(cars, store, now, TICK_S)
            time.sleep(max(0, TICK_S - ((time.time() - start) - now)))
    except KeyboardInterrupt:
        store.save_json("final_snapshot.json")
        print("\nSaved final_snapshot.json")


if __name__ == "__main__":
    main()
