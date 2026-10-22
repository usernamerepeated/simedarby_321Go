"""
store.py  —  where all live data lives 
==========================================================
Think of this as a tiny database with 4 tables (Python dicts):
    vehicles  {car_id: {...}}      live position + states of every car
    tickets   {ticket_id: {...}}   active / finished exit requests
    wallets   {user_id: {...}}     token balances
    ledger    [ {...}, ... ]       append-only list of every token movement

Default: everything is in memory in ONE Python process (fast, nothing to set up).
Optional: call enable_firebase(url) and every snapshot is also pushed to a
Firebase Realtime Database, so the judges can watch the "cloud" update live
and other laptops can read it.
"""
import copy
import json
import threading

try:
    import requests  # only needed for Firebase mirror
except ImportError:  # pragma: no cover
    requests = None


class Store:
    def __init__(self):
        self.vehicles = {}
        self.tickets = {}
        self.wallets = {}
        self.ledger = []
        self.processed_receipts = set()   # offline receipts already paid (no double pay)
        self._ticket_counter = 0
        self._firebase_url = None
        self._last_push = -1.0
        self._push_busy = False         # True while an upload is still in flight

    # ---------------- vehicles ----------------
    def update_vehicle(self, car_id, fields):
        """Merge new fields into a car's record (creates it if new)."""
        car = self.vehicles.setdefault(car_id, {"car_id": car_id})
        car.update(fields)
        return car

    def remove_vehicle(self, car_id):
        self.vehicles.pop(car_id, None)

    # ---------------- tickets ----------------
    def new_ticket_id(self):
        self._ticket_counter += 1
        return f"T{self._ticket_counter:04d}"

    # ---------------- wallets ----------------
    def wallet(self, user_id):
        return self.wallets.setdefault(
            user_id, {"user_id": user_id, "balance": 0, "earned_today": 0, "day": None})

    # ---------------- snapshot ----------------
    def snapshot(self):
        """Everything the god-view / phones need, as plain JSON-safe dicts."""
        return copy.deepcopy({
            "vehicles": self.vehicles,
            "tickets": self.tickets,
            "wallets": self.wallets,
            "ledger": self.ledger[-20:],   # last 20 transactions is enough for display
        })

    def save_json(self, path="snapshot.json"):
        with open(path, "w") as f:
            json.dump(self.snapshot(), f, indent=2)

    # ---------------- optional Firebase mirror ----------------
    def enable_firebase(self, db_url):
        """db_url like 'https://your-project-default-rtdb.asia-southeast1.firebasedatabase.app'
        Create the DB in TEST MODE so no keys are needed for the hackathon."""
        if requests is None:
            raise RuntimeError("pip install requests")
        self._firebase_url = db_url.rstrip("/")

    def push_to_firebase(self, now, every_s=0.5):
        """Upload the snapshot at most every 0.5 s, in a BACKGROUND thread, so a slow
        network can never slow the 200 ms backend loop. If the previous upload is
        still going, this one is simply skipped (the next one carries newer data)."""
        if not self._firebase_url or self._push_busy or now - self._last_push < every_s:
            return
        self._last_push, self._push_busy = now, True
        snap = self.snapshot()                       # copy taken now, uploaded in background

        def upload():
            try:
                r = requests.put(f"{self._firebase_url}/live.json", json=snap, timeout=3)
                if r.status_code != 200:             # e.g. 401 = rules locked / test mode expired
                    print(f"[firebase] HTTP {r.status_code}: {r.text[:120]}")
            except Exception as e:                   # never let the network crash the demo
                print(f"[firebase] push failed: {e}")
            finally:
                self._push_busy = False

        threading.Thread(target=upload, daemon=True).start()
