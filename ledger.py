"""
ledger.py  —  tokens (owned by Member 3)
========================================
Two ways tokens get paid:
  1. ONLINE:  state machine calls pay_ticket() when the red car exits on green.
  2. OFFLINE: phones were in BLE mode, saved receipts locally; when internet
              comes back, sync_offline_receipts() checks them and pays.

Safety rules built in:
  * Idempotent: a ticket / receipt can only ever be paid once.
  * Daily cap: one user can't earn more than DAILY_CAP_TOKENS per day (anti-farming).
  * Every movement is appended to store.ledger (an audit trail = "the ledger").
"""
import datetime as dt

from contract import (REWARD_TOKENS, DAILY_CAP_TOKENS, OFFLINE_RECEIPT_MAX_AGE_S)


def _day_of(ts):
    return dt.datetime.fromtimestamp(ts).strftime("%Y-%m-%d")


def credit(store, user_id, amount, reason, ref, now):
    """Add tokens to one wallet, respecting the daily cap. Returns amount actually paid."""
    w = store.wallet(user_id)
    today = _day_of(now)
    if w["day"] != today:                 # new day -> reset the daily counter
        w["day"], w["earned_today"] = today, 0
    allowed = max(0, DAILY_CAP_TOKENS - w["earned_today"])
    paid = min(amount, allowed)
    if paid > 0:
        w["balance"] += paid
        w["earned_today"] += paid
    store.ledger.append({"t": now, "user_id": user_id, "amount": paid,
                         "requested": amount, "reason": reason, "ref": ref})
    return paid


def pay_ticket(store, ticket, now):
    """Pay every car that yielded (and didn't abort) for this ticket. Runs once."""
    if ticket["paid"]:
        return {}
    ticket["paid"] = True
    results = {}
    for car_id in ticket["yielders"]:
        car = store.vehicles.get(car_id, {})
        user_id = car.get("user_id", car_id)
        results[car_id] = credit(store, user_id, REWARD_TOKENS, "yield_online",
                                 ticket["ticket_id"], now)
        if car:
            car["last_reward"] = results[car_id]
    return results


def sync_offline_receipts(store, receipts, now):
    """
    receipts: list of dicts saved on phones while offline (Member 2 creates them):
      {"receipt_id": "r-abc", "encounter_id": "enc-77", "role": "YIELDER" or "EXITER",
       "car_id": "car_blue_1", "user_id": "user_002", "junction_id": "elmina_J1",
       "timestamp": 1729580000.0}

    A YIELDER is paid only if the EXITER (the red car) uploaded a matching receipt
    with the same encounter_id -> two phones must agree (two-party proof).
    Returns a list of (receipt_id, status) for logging.
    """
    log = []
    exiters = {r["encounter_id"] for r in receipts if r.get("role") == "EXITER"}
    for r in receipts:
        rid = r.get("receipt_id")
        if r.get("role") != "YIELDER":
            continue
        if rid in store.processed_receipts:
            log.append((rid, "DUPLICATE_IGNORED")); continue
        if now - r.get("timestamp", 0) > OFFLINE_RECEIPT_MAX_AGE_S:
            log.append((rid, "TOO_OLD")); continue
        if r.get("encounter_id") not in exiters:
            log.append((rid, "NO_MATCHING_EXIT")); continue
        paid = credit(store, r["user_id"], REWARD_TOKENS, "yield_offline", rid, now)
        store.processed_receipts.add(rid)
        log.append((rid, f"PAID_{paid}"))
    return log
