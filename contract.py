"""
contract.py  —  THE DATA CONTRACT 
=====================================================
Every teammate imports names from here instead of typing strings by hand.
If two people spell a state differently ("YIELD" vs "YIELDING") the demo breaks,
so this file is the single source of truth.

RULE: only Member 3 edits this file. Any change -> bump CONTRACT_VERSION and
announce it in the group chat ("CONTRACT v1.1: added field X").
"""

CONTRACT_VERSION = "1.3"   # v1.1: raw IMU field names, Member 2 reset rules · v1.2: ALERT_SPEC · v1.3: reward 10 tokens

# ---------------------------------------------------------------------------
# 1. Timing + tunable numbers (change these to tune the demo)
# ---------------------------------------------------------------------------
TICK_S = 0.2                 # backend loop runs 5 times per second (200 ms)
GEOFENCE_RADIUS_M = 250      # Member 1: circle around junction centre
MATCH_MIN_M = 80             # closer than this, a car can't comfortably slow down -> don't ask it
MATCH_MAX_M = 250            # further than this, too early to ask
CRITICAL_GAP_S = 7.0         # a lane is "naturally clear" if the nearest car is >= 7 s away
YIELD_GAP_S = 4.0            # a confirmed yielder (slowing down, knows we're coming) only needs >= 4 s
OFFER_TIMEOUT_S = 12         # offered car didn't yield within 12 s -> skip it, ask the next car
ABORT_HOLD_S = 3             # red screen stays at least 3 s after an abort
TICKET_MAX_S = 300           # give up on a ticket after 5 minutes
EXIT_SPEED_KMH = 5           # red car counts as "moving out" above this speed
COMMITTED_SPEED_KMH = 2      # red car has started moving -> don't flip its green light (except ABORT)
REWARD_TOKENS = 10           # tokens per yielding car
DAILY_CAP_TOKENS = 100       # anti-farming: max tokens one user can earn per day
OFFLINE_RECEIPT_MAX_AGE_S = 24 * 3600

# ---------------------------------------------------------------------------
# 2. Lane roles  (written by Member 1)
#    Malaysia drives on the LEFT. Standing at the side road facing the main road:
#    MAIN_NEAR = the lane closest to you, traffic comes from your RIGHT
#    MAIN_FAR  = the lane further away, traffic comes from your LEFT
# ---------------------------------------------------------------------------
SIDE_ROAD = "SIDE_ROAD"      # red cars waiting to exit
MAIN_NEAR = "MAIN_NEAR"
MAIN_FAR = "MAIN_FAR"
OUTSIDE = "OUTSIDE"          # not in any zone / outside geofence

# Which lanes must have a gap for each turn
#   Turn LEFT  -> only join the near lane          -> need MAIN_NEAR clear
#   Turn RIGHT -> cross near lane, join far lane   -> need BOTH clear
REQUIRED_LANES = {
    "LEFT": [MAIN_NEAR],
    "RIGHT": [MAIN_NEAR, MAIN_FAR],
}
DEFAULT_TURN = "RIGHT"       # safest default (asks for both lanes)

# ---------------------------------------------------------------------------
# 3. Kinematic states  (written by Member 2)
#    IMPORTANT: YIELDING must be LATCHED -> once a car is YIELDING it stays
#    YIELDING until it passes the junction or triggers ABORT_HAZARD.
#    (Otherwise when the car reaches a steady slow speed, decel = 0 and it
#     would flip back to CRUISING.)
#    ABORT_HAZARD only counts while the car is still approaching the junction.
#    RESET RULES: TRAPPED -> CRUISING when the car moves off;
#                 YIELDING / ABORT_HAZARD -> CRUISING once approaching == False.
#    "In the side-road zone" = Member 1's lane_role == SIDE_ROAD (Member 2 must
#    not keep its own zone flag).
#    Every state string must be imported from here, never typed by hand.
# ---------------------------------------------------------------------------
CRUISING = "CRUISING"
TRAPPED = "TRAPPED"
YIELDING = "YIELDING"
ABORT_HAZARD = "ABORT_HAZARD"

# ---------------------------------------------------------------------------
# 4. Ticket states  (written by Member 3 — the state machine)
# ---------------------------------------------------------------------------
T_SCANNING = "SCANNING"            # ticket open, looking for cars to ask
T_OFFER_SENT = "OFFER_SENT"        # at least one blue car has been asked
T_PARTIAL_YIELD = "PARTIAL_YIELD"  # some required lanes clear, not all
T_CLEAR_TO_EXIT = "CLEAR_TO_EXIT"  # every required lane has a gap
T_ABORT_RED = "ABORT_RED"          # a yielding car accelerated during green
T_COMPLETED = "COMPLETED"          # red car exited during green, tokens paid
T_EXPIRED = "EXPIRED"              # timed out
T_CANCELLED = "CANCELLED"          # red car left without green / disappeared
ACTIVE_TICKET_STATES = {T_SCANNING, T_OFFER_SENT, T_PARTIAL_YIELD,
                        T_CLEAR_TO_EXIT, T_ABORT_RED}

# ---------------------------------------------------------------------------
# 5. UI commands  (written by Member 3, read by phone screens + god-view)
#    These map 1:1 to the Figma screens of Member 4.
# ---------------------------------------------------------------------------
# Red car screens
UI_SCANNING = "SCANNING"           # Screen A1: radar + wait timer
UI_WAIT_HOLD = "WAIT_HOLD"         # Screen A1 variant: "1 of 2 lanes yielding..."
UI_GO_GREEN = "GO_GREEN"           # Screen A2: "GAP OPENING — CHECK & GO"
UI_ABORT_RED = "ABORT_RED"         # Screen A3: "STOP — CAR ACCELERATING"
UI_DONE = "DONE"                   # back to normal
# Blue car screens
UI_CRUISE = "CRUISE"               # Screen B1
UI_PROMPT_YIELD = "PROMPT_YIELD"   # Screen B2: amber banner + voice
UI_YIELD_CONFIRMED = "YIELD_CONFIRMED"  # Screen B3a: "Yield detected, holding gap"
UI_PAYOUT = "PAYOUT"               # Screen B3b: "+10 Tokens"

# ---------------------------------------------------------------------------
# 5b. How each ui_command is DELIVERED (Members 4, 5 and the god-view use this)
#     RULE: play the sound / voice ONCE when a car's ui_command CHANGES,
#     not every 200 ms tick (the backend repeats the same command every tick).
#     Sound carries the meaning, colour only confirms. Never full screen on a
#     moving (blue) driver.
# ---------------------------------------------------------------------------
ALERT_SPEC = {
    UI_CRUISE:          {"visual": "none / dismiss banner", "sound": None, "voice": None,
                         "vibrate": None, "fullscreen": False},
    UI_PROMPT_YIELD:    {"visual": "amber banner over maps app", "sound": "soft_chime",
                         "voice": "Car waiting ahead. Ease off to earn 10 tokens.",
                         "vibrate": None, "fullscreen": False},
    UI_YIELD_CONFIRMED: {"visual": "banner turns green: Holding gap", "sound": "thank_you_chime",
                         "voice": None, "vibrate": None, "fullscreen": False},
    UI_PAYOUT:          {"visual": "notification: +10 Tokens", "sound": None,
                         "voice": None, "vibrate": None, "fullscreen": False},
    UI_SCANNING:        {"visual": "radar + wait timer", "sound": None, "voice": None,
                         "vibrate": None, "fullscreen": True},
    UI_WAIT_HOLD:       {"visual": "radar: 1 of 2 lanes yielding", "sound": None, "voice": None,
                         "vibrate": None, "fullscreen": True},
    UI_GO_GREEN:        {"visual": "full-screen green: GAP OPENING - CHECK & GO",
                         "sound": "rising_beeps", "voice": "Gap opening, check and go.",
                         "vibrate": "short_pulse", "fullscreen": True},
    UI_ABORT_RED:       {"visual": "full-screen flashing red: STOP - CAR ACCELERATING",
                         "sound": "urgent_alarm_repeat", "voice": "Stop.",
                         "vibrate": "strong_repeat", "fullscreen": True},
    UI_DONE:            {"visual": "back to normal", "sound": None, "voice": None,
                         "vibrate": None, "fullscreen": False},
}

# ---------------------------------------------------------------------------
# 6. Example records (give this to Member 4 + 5 so they can design around it)
# ---------------------------------------------------------------------------
EXAMPLE_VEHICLE = {
    # --- raw (from phone / simulator) ---
    "car_id": "car_blue_1", "user_id": "user_002", "intended_turn": None,  # "LEFT"/"RIGHT" (set before driving, optional)
    "lat": 3.1712, "lon": 101.5123, "speed_kmh": 42.0, "heading_deg": 355.0,
    "lin_x": 0.02, "lin_y": -0.21, "lin_z": 0.05,   # m/s^2, gravity removed (Phyphox linX/linY/linZ)
    "gyro_x": 0.01, "gyro_y": 0.00, "gyro_z": 0.02, # rad/s (Phyphox gyrX/gyrY/gyrZ)
    "updated_at": 0.0,
    # --- Member 1 writes ---
    "junction_id": "elmina_J1", "in_geofence": True, "lane_role": MAIN_NEAR,
    "distance_to_junction_m": 160.0, "approaching": True,
    "ble_rssi": -82, "link_mode": "ONLINE_GPS",   # or "OFFLINE_BLE"
    # --- Member 2 writes ---
    "kinematic_state": CRUISING, "accel_long_ms2": -0.2, "gyro_ok": True,
    # --- Member 3 writes ---
    "ui_command": UI_CRUISE, "active_ticket_id": None, "last_reward": 0,
}

EXAMPLE_TICKET = {
    "ticket_id": "T0001", "junction_id": "elmina_J1", "red_car_id": "car_red_1",
    "turn": "RIGHT", "required_lanes": [MAIN_NEAR, MAIN_FAR],
    "state": T_OFFER_SENT, "created_at": 0.0, "state_since": 0.0,
    "lanes": {
        MAIN_NEAR: {"offered_car": "car_blue_1", "offered_at": 0.0,
                    "lead_car": "car_blue_1", "clear": False},
        MAIN_FAR: {"offered_car": "car_blue_2", "offered_at": 0.0,
                   "lead_car": "car_blue_2", "clear": False},
    },
    "yielders": [], "declined": [], "paid": False,
}

EXAMPLE_WALLET = {"user_id": "user_002", "balance": 150,
                  "earned_today": 50, "day": "2026-10-22"}

# Raw fields every phone / the simulator sends (nobody computes these)
RAW_FIELDS = ["car_id", "user_id", "intended_turn", "lat", "lon", "speed_kmh", "heading_deg",
              "lin_x", "lin_y", "lin_z", "gyro_x", "gyro_y", "gyro_z", "updated_at"]

# Field ownership — each member ONLY writes their own fields (prevents overwriting)
FIELD_OWNER = {
    "member1": ["junction_id", "in_geofence", "lane_role", "distance_to_junction_m",
                "approaching", "ble_rssi", "link_mode"],
    "member2": ["kinematic_state", "accel_long_ms2", "gyro_ok"],
    "member3": ["ui_command", "active_ticket_id", "last_reward"],
}
