#!/usr/bin/env python3
"""
tb_export.py -- pull device telemetry from ThingsBoard into a CSV for analysis.

Usage (from the repo root, any Python 3 with `requests`; the PlatformIO one works):

    # 1. list your devices (find the right name):
    python tools/tb_export.py --user you@ethz.ch

    # 2. export the last 24 h for one device:
    python tools/tb_export.py --user you@ethz.ch --device "YOUR-DEVICE-NAME" --hours 24

Password is prompted interactively (never stored, never in shell history).
Output: tools/tb_export_<device>_<date>.csv  (long format: one row per datapoint)
"""
import argparse
import csv
import getpass
import sys
import time
from datetime import datetime, timezone

try:
    import requests
except ImportError:
    sys.exit("needs `requests` -- run with PlatformIO's python: "
             "%USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe tools/tb_export.py ...")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="eu.thingsboard.cloud")
    ap.add_argument("--user", required=True, help="ThingsBoard login email")
    ap.add_argument("--password", help="omit to be prompted securely")
    ap.add_argument("--device", help="device name; omit to list devices")
    ap.add_argument("--hours", type=float, default=24.0)
    ap.add_argument("--out", help="output CSV path (default: auto-named)")
    args = ap.parse_args()

    base = f"https://{args.host}"
    pw = args.password or getpass.getpass("ThingsBoard password: ")

    # ---- login ----
    r = requests.post(f"{base}/api/auth/login",
                      json={"username": args.user, "password": pw}, timeout=20)
    if r.status_code != 200:
        sys.exit(f"login failed: HTTP {r.status_code} {r.text[:200]}")
    hdr = {"X-Authorization": "Bearer " + r.json()["token"]}

    # ---- find device(s) ----
    r = requests.get(f"{base}/api/tenant/devices?pageSize=200&page=0",
                     headers=hdr, timeout=20)
    r.raise_for_status()
    devices = r.json().get("data", [])
    if not args.device:
        print("Devices on this tenant:")
        for d in devices:
            print(f"  - {d['name']}  (type: {d.get('type','-')}, id: {d['id']['id']})")
        print("\nRe-run with --device \"<name>\"")
        return
    dev = next((d for d in devices if d["name"] == args.device), None)
    if not dev:
        sys.exit(f"device '{args.device}' not found -- run without --device to list")
    dev_id = dev["id"]["id"]

    # ---- keys ----
    r = requests.get(f"{base}/api/plugins/telemetry/DEVICE/{dev_id}/keys/timeseries",
                     headers=hdr, timeout=20)
    r.raise_for_status()
    keys = r.json()
    print(f"{len(keys)} telemetry keys: {', '.join(sorted(keys))}")

    # ---- timeseries ----
    end_ms = int(time.time() * 1000)
    start_ms = end_ms - int(args.hours * 3600 * 1000)
    r = requests.get(
        f"{base}/api/plugins/telemetry/DEVICE/{dev_id}/values/timeseries",
        params={"keys": ",".join(keys), "startTs": start_ms, "endTs": end_ms,
                "limit": 50000, "orderBy": "ASC"},
        headers=hdr, timeout=60)
    r.raise_for_status()
    series = r.json()

    # ---- write long-format CSV ----
    out = args.out or (f"tools/tb_export_{args.device.replace(' ', '_')}_"
                       f"{datetime.now().strftime('%Y%m%d_%H%M')}.csv")
    n = 0
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["ts_ms", "ts_utc", "key", "value"])
        for key, points in series.items():
            for p in points:
                iso = datetime.fromtimestamp(p["ts"] / 1000, tz=timezone.utc)
                w.writerow([p["ts"], iso.strftime("%Y-%m-%d %H:%M:%S"),
                            key, p["value"]])
                n += 1
    print(f"wrote {n} datapoints across {len(series)} keys -> {out}")
    print("hand that file to Claude for analysis")


if __name__ == "__main__":
    main()
