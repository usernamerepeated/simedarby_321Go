"""
state_machine.py  —  the "negotiator" 
=========================================================
Called every 200 ms with tick(store, now). Each tick does 3 jobs:

  JOB 1  open_new_tickets()   a red car became TRAPPED -> open a Yield Request ticket
  JOB 2  update_ticket()      for each open ticket, per required lane:
                                 a) notice who YIELDED / who ABORTED
                                 b) drop offers that timed out or whose car already passed
                                 c) send an offer to the next suitable car
                                 d) decide "is this lane clear?"
                              then derive the ticket state from the lanes
                              and check if the red car has exited -> pay tokens
  JOB 3  write_ui_commands()  translate ticket states into screen commands for every car

Design idea (why it's easy to debug): the ticket state is RE-CALCULATED from the
lanes every tick instead of being "remembered". So if a car passes or aborts,
the state automatically corrects itself on the next tick.
"""
import math

from contract import (
    SIDE_ROAD, REQUIRED_LANES, DEFAULT_TURN,
    TRAPPED, YIELDING, ABORT_HAZARD,
    T_SCANNING, T_OFFER_SENT, T_PARTIAL_YIELD, T_CLEAR_TO_EXIT, T_ABORT_RED,
    T_COMPLETED, T_EXPIRED, T_CANCELLED, ACTIVE_TICKET_STATES,
    UI_SCANNING, UI_WAIT_HOLD, UI_GO_GREEN, UI_ABORT_RED, UI_DONE,
    UI_CRUISE, UI_PROMPT_YIELD, UI_YIELD_CONFIRMED, UI_PAYOUT,
    MATCH_MIN_M, MATCH_MAX_M, CRITICAL_GAP_S, YIELD_GAP_S, COMMITTED_SPEED_KMH,
    OFFER_TIMEOUT_S, ABORT_HOLD_S, TICKET_MAX_S, EXIT_SPEED_KMH,
)
import ledger

PAYOUT_SCREEN_S = 4   # how long the blue phone shows "+50 tokens"

RED_UI_FOR_STATE = {
    T_SCANNING: UI_SCANNING,
    T_OFFER_SENT: UI_SCANNING,
    T_PARTIAL_YIELD: UI_WAIT_HOLD,
    T_CLEAR_TO_EXIT: UI_GO_GREEN,
    T_ABORT_RED: UI_ABORT_RED,
}


# =============================================================================
# Main entry point
# =============================================================================
def tick(store, now):
    """Run one 200 ms cycle. Returns a list of human-readable events (for logs/god-view)."""
    events = []
    open_new_tickets(store, now, events)
    for t in list(store.tickets.values()):
        if t["state"] in ACTIVE_TICKET_STATES:
            update_ticket(store, t, now, events)
    write_ui_commands(store, now)
    return events


# =============================================================================
# JOB 1 — open a ticket when a red car is trapped
# =============================================================================
def open_new_tickets(store, now, events):
    busy_junctions = {t["junction_id"] for t in store.tickets.values()
                      if t["state"] in ACTIVE_TICKET_STATES}
    trapped_by_junction = {}
    for car in store.vehicles.values():
        if (car.get("lane_role") == SIDE_ROAD
                and car.get("kinematic_state") == TRAPPED
                and car.get("in_geofence")
                and not car.get("active_ticket_id")):
            trapped_by_junction.setdefault(car["junction_id"], []).append(car)

    for junction_id, cars in trapped_by_junction.items():
        if junction_id in busy_junctions:
            continue                      # one ticket per junction at a time (queue order)
        front = min(cars, key=lambda c: c.get("distance_to_junction_m", 1e9))
        turn = front.get("intended_turn") or DEFAULT_TURN
        lanes = REQUIRED_LANES[turn]
        tid = store.new_ticket_id()
        store.tickets[tid] = {
            "ticket_id": tid, "junction_id": junction_id, "red_car_id": front["car_id"],
            "turn": turn, "required_lanes": list(lanes),
            "state": T_SCANNING, "created_at": now, "state_since": now,
            "lanes": {ln: {"offered_car": None, "offered_at": None,
                           "lead_car": None, "clear": False} for ln in lanes},
            "yielders": [], "declined": [], "paid": False,
        }
        front["active_ticket_id"] = tid
        events.append(f"{now:6.1f}s  OPEN    {tid}: {front['car_id']} trapped, turn {turn}, "
                      f"needs {'+'.join(lanes)}")


# =============================================================================
# JOB 2 — update one ticket
# =============================================================================
def update_ticket(store, t, now, events):
    tid = t["ticket_id"]
    red = store.vehicles.get(t["red_car_id"])

    # ---- 2.0  Red car vanished (app closed)? ----
    if red is None:
        return finish(store, t, T_CANCELLED, now, events, "red car disappeared")

    # ---- 2.1  Did the red car leave the side road? ----
    if red.get("lane_role") != SIDE_ROAD and red.get("speed_kmh", 0) > EXIT_SPEED_KMH:
        if t["state"] == T_CLEAR_TO_EXIT:
            return finish(store, t, T_COMPLETED, now, events, "red car exited on green")
        return finish(store, t, T_CANCELLED, now, events, "red car left without green")

    # ---- 2.2  Ticket too old? ----
    if now - t["created_at"] > TICKET_MAX_S:
        return finish(store, t, T_EXPIRED, now, events, "timeout")

    # ---- 2.3  Work lane by lane ----
    abort_happened = False
    for lane in t["required_lanes"]:
        info = t["lanes"][lane]
        approaching = cars_approaching(store, t["junction_id"], lane)
        ids_here = [c["car_id"] for c in approaching]

        # a) who yielded / who aborted?
        for c in approaching:
            cid, ks = c["car_id"], c.get("kinematic_state")
            involved = cid in t["yielders"] or cid == info["offered_car"]
            if ks == ABORT_HAZARD and involved:
                if cid in t["yielders"]:
                    t["yielders"].remove(cid)            # aborting = no tokens
                if cid not in t["declined"]:
                    t["declined"].append(cid)
                if info["offered_car"] == cid:
                    info["offered_car"] = None
                abort_happened = True
                events.append(f"{now:6.1f}s  ABORT   {tid}: {cid} accelerated in {lane}")
            elif ks == YIELDING and cid == info["offered_car"] and cid not in t["yielders"]:
                t["yielders"].append(cid)
                events.append(f"{now:6.1f}s  YIELD   {tid}: {cid} is yielding in {lane}")

        # b) clean up the current offer
        oc = info["offered_car"]
        if oc and oc not in ids_here:
            info["offered_car"] = None                   # car already passed the junction
        elif oc and oc not in t["yielders"] and now - info["offered_at"] > OFFER_TIMEOUT_S:
            t["declined"].append(oc)
            info["offered_car"] = None
            events.append(f"{now:6.1f}s  IGNORED {tid}: {oc} didn't yield in "
                          f"{OFFER_TIMEOUT_S}s, asking next car")

        # c) send a new offer if nobody in this lane is currently helping
        someone_helping = any(cid in t["yielders"] for cid in ids_here)
        if info["offered_car"] is None and not someone_helping:
            for c in approaching:
                d = c.get("distance_to_junction_m", 1e9)
                if (MATCH_MIN_M <= d <= MATCH_MAX_M and c["car_id"] not in t["declined"]
                        and c["car_id"] not in t["yielders"]):
                    info["offered_car"], info["offered_at"] = c["car_id"], now
                    events.append(f"{now:6.1f}s  OFFER   {tid}: asked {c['car_id']} "
                                  f"({lane}, {d:.0f} m away)")
                    break

        # d) is there a safe gap in this lane?
        lead = approaching[0] if approaching else None
        info["lead_car"] = lead["car_id"] if lead else None
        info["clear"] = lane_is_clear(lead, t)

    # ---- 2.4  Derive the ticket state from the lanes ----
    req = [t["lanes"][ln] for ln in t["required_lanes"]]
    all_clear = all(i["clear"] for i in req)
    some_clear = any(i["clear"] for i in req)
    offers_out = any(i["offered_car"] for i in req) or bool(t["yielders"])
    prev = t["state"]

    if abort_happened and prev in (T_CLEAR_TO_EXIT, T_ABORT_RED):
        new = T_ABORT_RED                                  # danger! red car must stop
    elif prev == T_ABORT_RED and now - t["state_since"] < ABORT_HOLD_S:
        new = T_ABORT_RED                                  # keep red screen for a few seconds
    elif prev == T_CLEAR_TO_EXIT and red.get("speed_kmh", 0) >= COMMITTED_SPEED_KMH:
        new = T_CLEAR_TO_EXIT                              # red already moving: never flip the light
    elif all_clear:
        new = T_CLEAR_TO_EXIT
    elif some_clear and len(req) > 1:
        new = T_PARTIAL_YIELD
    elif offers_out:
        new = T_OFFER_SENT
    else:
        new = T_SCANNING
    set_state(t, new, now, events)


def cars_approaching(store, junction_id, lane):
    """App users in this lane, inside the geofence, heading TOWARDS the junction, nearest first."""
    cars = [c for c in store.vehicles.values()
            if c.get("junction_id") == junction_id
            and c.get("lane_role") == lane
            and c.get("in_geofence")
            and c.get("approaching")]
    return sorted(cars, key=lambda c: c.get("distance_to_junction_m", 1e9))


def lane_is_clear(lead, ticket):
    """
    A lane has a safe gap if ONE of these is true:
      1. no app car is approaching in this lane
      2. the nearest car is far/slow enough: time-to-arrival >= CRITICAL_GAP_S (7 s)
         (time-to-arrival = distance / speed, a "countdown" until it reaches us)
      3. the nearest car is a confirmed YIELDER and its time-to-arrival >= YIELD_GAP_S (4 s)
         (smaller gap is OK because that driver is slowing down and expects us)
    NOTE: the app can't see non-app cars, motorbikes or pedestrians, so the red
    phone must say "GAP OPENING — CHECK & GO", never "SAFE".
    """
    if lead is None:
        return True
    d = lead.get("distance_to_junction_m", 1e9)
    speed_ms = max(lead.get("speed_kmh", 0) / 3.6, 0.1)   # avoid divide-by-zero
    tta = d / speed_ms                                    # seconds until it reaches the junction
    needed = YIELD_GAP_S if lead["car_id"] in ticket["yielders"] else CRITICAL_GAP_S
    return tta >= needed


def set_state(t, new, now, events):
    if new != t["state"]:
        events.append(f"{now:6.1f}s  STATE   {t['ticket_id']}: {t['state']} -> {new}")
        t["state"], t["state_since"] = new, now


def finish(store, t, final_state, now, events, why):
    set_state(t, final_state, now, events)
    events.append(f"{now:6.1f}s  CLOSE   {t['ticket_id']}: {why}")
    red = store.vehicles.get(t["red_car_id"])
    if red:
        red["active_ticket_id"] = None
        red["ui_command"] = UI_DONE
    if final_state == T_COMPLETED:
        paid = ledger.pay_ticket(store, t, now)
        for cid, amount in paid.items():
            store.vehicles.get(cid, {})["payout_at"] = now
            events.append(f"{now:6.1f}s  PAY     {t['ticket_id']}: {cid} +{amount} tokens")
        if not paid and t["yielders"]:
            events.append(f"{now:6.1f}s  PAY     {t['ticket_id']}: yielders offline - paid when their phones sync")
        elif not paid:
            events.append(f"{now:6.1f}s  PAY     {t['ticket_id']}: nobody to pay (natural gap)")


# =============================================================================
# JOB 3 — tell every phone which screen to show
# =============================================================================
def write_ui_commands(store, now):
    # default for everyone
    for car in store.vehicles.values():
        if car.get("lane_role") == SIDE_ROAD and car.get("ui_command") == UI_DONE:
            continue
        recently_paid = now - car.get("payout_at", -1e9) < PAYOUT_SCREEN_S
        car["ui_command"] = UI_PAYOUT if recently_paid else UI_CRUISE
        if car.get("lane_role") != SIDE_ROAD:
            car["active_ticket_id"] = None

    # cars involved in an active ticket
    for t in store.tickets.values():
        if t["state"] not in ACTIVE_TICKET_STATES:
            continue
        red = store.vehicles.get(t["red_car_id"])
        if red:
            red["ui_command"] = RED_UI_FOR_STATE[t["state"]]
            red["active_ticket_id"] = t["ticket_id"]
        for lane in t["required_lanes"]:
            info = t["lanes"][lane]
            for cid in t["yielders"]:
                c = store.vehicles.get(cid)
                if c and c.get("approaching"):
                    c["ui_command"] = UI_YIELD_CONFIRMED
                    c["active_ticket_id"] = t["ticket_id"]
            oc = info["offered_car"]
            if oc and oc not in t["yielders"] and oc in store.vehicles:
                store.vehicles[oc]["ui_command"] = UI_PROMPT_YIELD
                store.vehicles[oc]["active_ticket_id"] = t["ticket_id"]
