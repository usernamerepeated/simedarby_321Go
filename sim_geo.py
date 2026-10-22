"""
sim_geo.py  —  give the simulated cars real GPS coordinates
=====================================================================
The traffic simulator (run_scenarios.py) moves cars along a straight line:
car.pos = metres before the junction. Real phones send lat/lon/heading instead.
gps_fields(car, now) converts the simulated position into what a phone would send,
placed on the real junction from junctions.json, so spatial_engine.compute() can be
tested exactly as it will run with real phones.

    near-lane car : a = +pos,  b = -1.75 m (lane centre on the side-road side), heading = near bearing
    far-lane car  : a = -pos,  b = +1.75 m,                                   heading = far bearing
    red car       : waits on the side road, then drives up to its target lane and turns
"""
from contract import MAIN_NEAR
import spatial_engine as se

LANE_OFFSET_M = 1.75          # half of a 3.5 m lane
QUEUE_GAP_M = 3.0             # front red car's GPS sits ~3 m behind the stop line distance


def _red_pose(car, now, J):
    """Waiting red car: on the side road. Moving red car: drives forward, then turns."""
    last_t = getattr(car, "_geo_t", None)
    car._geo_t = now
    progress = getattr(car, "_geo_progress", 0.0)
    if car.speed_kmh <= 0 and car.go_at is None:
        progress = 0.0                                  # waiting, or stopped by ABORT_RED at the line
    elif last_t is not None:
        progress += car.speed_kmh / 3.6 * (now - last_t)
    car._geo_progress = progress

    b0 = -(getattr(car, "queue_dist", 5.0) + QUEUE_GAP_M)
    right_turn = car.intended_turn != "LEFT"
    lane_b = LANE_OFFSET_M if right_turn else -LANE_OFFSET_M
    leg1 = lane_b - b0                                  # straight out of the side road
    if progress <= leg1:
        return 0.0, b0 + progress, se._bearing(*J.n)    # heading: straight across the main road
    along = progress - leg1
    if right_turn:                                      # right turn -> far lane, drives towards +a
        return along, lane_b, J.far_bearing
    return -along, lane_b, J.near_bearing               # left turn -> near lane, drives towards -a


def gps_fields(car, now, junction=None):
    J = junction or se.default_junction()
    if car.behaviour == "red":
        a, b, heading = _red_pose(car, now, J)
    elif car.lane == MAIN_NEAR:
        a, b, heading = car.pos, -LANE_OFFSET_M, J.near_bearing
    else:
        a, b, heading = -car.pos, LANE_OFFSET_M, J.far_bearing
    lat, lon = J.from_local(a, b)
    return {"lat": round(lat, 7), "lon": round(lon, 7), "heading_deg": round(heading, 1)}
