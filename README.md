# 321Go T-Junction Yield — Backend

```
python run_scenarios.py       # runs all 9 test scenarios, prints the event log (no install needed)
python run_scenarios.py rogue # one scenario: normal | rogue | left | ignore | offline | queue | no_green | empty | offline_live
python run_scenarios.py --real-m2   # same tests with the real sensor_fusion.py
python god_view.py            # LIVE DEMO SCREEN in your browser (keys 1-8, R, Space, arrows, M, F)
python spatial_engine.py      # check junctions.json + Member 1 self-test
python run_scenarios.py --real-m1 --real-m2   # all tests with everyone's real code
python main.py                # real-time 200 ms loop with simulated cars (text only)
python main.py --rogue        # same, with a driver who floors it during green
python sensor_fusion.py       # live phone test with Phyphox (set the phone IP in PHYPHOX_URL)
python sensor_fusion.py --calibrate sensor_log.csv   # find the phone's forward axis from a recorded drive

# optional, only if Firebase is used later (needs: pip install requests)
python firebase_check.py https://<db>.asia-southeast1.firebasedatabase.app
python main.py --firebase https://<db>.asia-southeast1.firebasedatabase.app
```

| File | Owner | What it does |
|---|---|---|
| contract.py | Member 3 | All names, thresholds, example JSON. Single source of truth. |
| store.py | Member 3 | In-memory "database" (vehicles, tickets, wallets, ledger) + optional Firebase push |
| state_machine.py | Member 3 | Opens tickets, matches cars, decides GO_GREEN / ABORT_RED, triggers payout |
| ledger.py | Member 3 | Token payouts, daily cap, offline receipt sync |
| run_scenarios.py | Member 3 (mocks of M1+M2) | Fake traffic to test the state machine alone |
| main.py | Member 3 | Live loop that plugs in spatial_engine.py (M1) and sensor_fusion.py (M2) |
| firebase_check.py | Member 3 | OPTIONAL: tests a Firebase connection. Nothing imports it; safe to delete |
| mock_snapshot.json | — | Example live data at the GO_GREEN moment — give to Members 4 & 5 |
| junctions.json | Member 1 | The junction: 3 GPS points (PLACEHOLDER - replace with your real junction) |
| spatial_engine.py | Member 1 | `compute(raw)`: GPS -> lane, distance, approaching, geofence, simulated BLE RSSI |
| sim_geo.py | Member 1 | Gives simulated cars real GPS lat/lon/heading on the junction |
| god_view.py + god_view.html | Member 1 | Live demo screen: Python server + browser page, sound + voice, no install needed |
| sensor_fusion.py | Member 2 | `compute(raw + M1 fields, now)` -> TRAPPED / YIELDING / ABORT_HAZARD. Live Phyphox mode with GPS checks + real lane detection; `--calibrate` finds the phone axis |
| offline_receipts.py | Member 2 | No-internet mode: phones save BLE yield/exit receipts, paid after sync |
