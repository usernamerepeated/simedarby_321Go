"""
offline_receipts.py  —   what each phone saves when there is NO internet
==================================================================================
When link_mode == "OFFLINE_BLE" the cloud can't pay anyone live, so each phone keeps
a receipt on the phone and uploads it later. ledger.sync_offline_receipts() pays a
YIELDER only if the waiting car's EXITER receipt carries the same encounter_id.

How the two phones agree on the same encounter_id without internet:
  1. The waiting (red) phone becomes TRAPPED -> it starts a BLE beacon that advertises
     an encounter_id, e.g. "enc-elmina_J1-car_red_1-15".
  2. A main-road (blue) phone that HEARS the beacon (BLE signal >= BEACON_HEARD_DBM)
     while it is YIELDING remembers that encounter_id.
  3. Blue phone passes the junction without aborting -> saves a YIELDER receipt.
     Red phone exits the side road           -> saves an EXITER receipt.
  4. Back online -> both upload -> ledger pays matching pairs once (no duplicates).

In the prototype, the simulator stands in for the phones: OfflineRecorder.observe()
looks at every car each tick and writes the receipts the phones would write.
Prototype limitation (say so if asked): BLE signal is simulated from distance.
"""
from contract import SIDE_ROAD, TRAPPED, YIELDING, ABORT_HAZARD, EXIT_SPEED_KMH

OFFLINE = "OFFLINE_BLE"
BEACON_HEARD_DBM = -96      # ~50 m with the spatial_engine path-loss model (BLE advertising range)


class OfflineRecorder:
    def __init__(self):
        self.receipts = []          # everything the phones have saved, waiting to upload
        self.encounters = {}        # red car_id -> open encounter
        self.pending = {}           # blue car_id -> encounter_id it yielded for
        self._n = 0

    def _receipt(self, role, car, enc_id, junction_id, now):
        self._n += 1
        r = {"receipt_id": f"r{self._n:03d}-{car['car_id']}", "encounter_id": enc_id, "role": role,
             "car_id": car["car_id"], "user_id": car.get("user_id", car["car_id"]),
             "junction_id": junction_id, "timestamp": now}
        self.receipts.append(r)
        return r

    def observe(self, store, now):
        """Call once per tick after the state machine. Returns event lines for the log."""
        events = []
        V = store.vehicles

        # 1) red phones: start a beacon when trapped offline, write EXITER when they leave
        for v in V.values():
            if v.get("link_mode") != OFFLINE:
                continue
            cid = v["car_id"]
            if (v.get("lane_role") == SIDE_ROAD and v.get("kinematic_state") == TRAPPED
                    and cid not in self.encounters):
                enc_id = f"enc-{v.get('junction_id')}-{cid}-{int(now)}"
                self.encounters[cid] = {"id": enc_id, "junction_id": v.get("junction_id"), "open": True}
                events.append(f"{now:6.1f}s  BLE     {cid} broadcasting {enc_id}")
            enc = self.encounters.get(cid)
            if (enc and enc["open"] and v.get("lane_role") != SIDE_ROAD
                    and v.get("speed_kmh", 0) > EXIT_SPEED_KMH):
                enc["open"] = False
                self._receipt("EXITER", v, enc["id"], enc["junction_id"], now)
                events.append(f"{now:6.1f}s  BLE     {cid} saved EXIT receipt ({enc['id']})")

        open_by_junction = {e["junction_id"]: e for e in self.encounters.values() if e["open"]}

        # 2) blue phones: remember the encounter while yielding in beacon range
        for v in V.values():
            cid = v["car_id"]
            if v.get("link_mode") != OFFLINE or cid in self.encounters:
                continue
            enc = open_by_junction.get(v.get("junction_id"))
            ks = v.get("kinematic_state")
            if ks == ABORT_HAZARD and cid in self.pending:
                events.append(f"{now:6.1f}s  BLE     {cid} sped up - no receipt")
                del self.pending[cid]
            elif (enc and ks == YIELDING and v.get("approaching")
                  and v.get("ble_rssi", -999) >= BEACON_HEARD_DBM and cid not in self.pending):
                self.pending[cid] = enc["id"]
                events.append(f"{now:6.1f}s  BLE     {cid} heard beacon while yielding")
            # passed the junction after yielding -> save the YIELDER receipt
            if cid in self.pending and not v.get("approaching"):
                enc_id = self.pending.pop(cid)
                jid = enc_id.split("-")[1] if enc_id.count("-") >= 3 else v.get("junction_id")
                self._receipt("YIELDER", v, enc_id, v.get("junction_id", jid), now)
                events.append(f"{now:6.1f}s  BLE     {cid} saved YIELD receipt ({enc_id})")
        return events

    def take_all(self):
        """Phones come back online: hand over everything saved, clear the phones."""
        out, self.receipts = self.receipts, []
        return out
