#!/usr/bin/env python3
"""
js_power.py -- measure current with a Joulescope, from the command line.

Why this exists
---------------
Every power number on this project so far came from one of two bad instruments:

  * battery discharge slope -- ~0.5 mA resolution, 2-3 DAYS per data point.
    It could not resolve a 6 % change, which is why the 2026-09-04/05 sleep
    work looked like it did nothing.
  * the BQ25792's IBAT ADC via a differential audit -- ~0.1 mA at best, 60 s
    per point, and it leans on activity dithering a 1 mA quantiser. It also
    can only measure a DIFFERENCE between two board states, never an absolute.

A Joulescope measures absolute current directly, from nanoamps to amps, at
2 MHz. It replaces both. Put it in series with the ultrasonic board's supply
and every firmware change becomes a number you read in seconds.

Usage
-----
    python tools/js_power.py --list
    python tools/js_power.py --seconds 10
    python tools/js_power.py --seconds 10 --label "boost duty-cycled"
    python tools/js_power.py --seconds 60 --csv run.csv

The default 'auto' config powers the target from the Joulescope's own supply.
Use --no-power if the target is powered from elsewhere and the Joulescope is
only sensing in-line, which is the case when the USS board is fed by the comm
board's SENSOR rail.
"""
import argparse
import statistics
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true", help="enumerate and exit")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--label", default="", help="tag printed with the result")
    ap.add_argument("--csv", help="append one summary row to this CSV")
    ap.add_argument("--no-power", action="store_true",
                    help="do not source target power (in-line sensing only)")
    args = ap.parse_args()

    try:
        from joulescope import scan, scan_require_one
    except ImportError:
        sys.exit("joulescope package missing -- pip install joulescope")

    if args.list:
        devs = scan()
        print("devices found: %d" % len(devs))
        for d in devs:
            print("   ", d)
        return 0 if devs else 1

    # 'auto' closes the current path and powers the target; 'off' leaves the
    # sensor powered but the target unpowered, which is wrong for in-line use.
    js = scan_require_one(config="auto")
    samples = []

    def on_stats(s):
        # v1 statistics: s['signals']['current']['µ']['value'] is the mean over
        # the reporting window (nominally 2 Hz), already in amps.
        try:
            samples.append(s["signals"]["current"]["µ"]["value"])
        except (KeyError, TypeError):
            pass

    with js:
        if args.no_power:
            try:
                js.parameter_set("io_voltage", "3.3V")
            except Exception:
                pass
        js.statistics_callback_register(on_stats, "sensor")
        js.start()
        t_end = time.time() + args.seconds
        while time.time() < t_end:
            time.sleep(0.05)
        js.stop()

    if not samples:
        sys.exit("no statistics received -- is the device streaming?")

    ua = [x * 1e6 for x in samples]
    mean = statistics.mean(ua)
    sd = statistics.pstdev(ua) if len(ua) > 1 else 0.0
    # standard error of the mean: the number that says whether two runs differ
    se = sd / (len(ua) ** 0.5) if len(ua) > 1 else 0.0

    label = (" [%s]" % args.label) if args.label else ""
    print("n=%d windows over %.1f s%s" % (len(ua), args.seconds, label))
    print("  mean   %10.1f uA  +/- %.1f (SE)" % (mean, se))
    print("  min    %10.1f uA" % min(ua))
    print("  max    %10.1f uA" % max(ua))
    print("  sd     %10.1f uA" % sd)
    print("  charge %10.3f mAh/day at this mean" % (mean * 24.0 / 1000.0))

    if args.csv:
        import csv
        import os
        new = not os.path.exists(args.csv)
        with open(args.csv, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["utc", "label", "seconds", "n",
                            "mean_ua", "se_ua", "min_ua", "max_ua", "sd_ua"])
            w.writerow([time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                        args.label, args.seconds, len(ua),
                        "%.1f" % mean, "%.1f" % se,
                        "%.1f" % min(ua), "%.1f" % max(ua), "%.1f" % sd])
    return 0


if __name__ == "__main__":
    sys.exit(main())
