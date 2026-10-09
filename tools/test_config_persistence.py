#!/usr/bin/env python3
"""Prove a capture start does not disturb the device configuration.

Regression guard for the "reset the engine before every Start" mistake: an
engine reset (CTRL=RST) wipes the pattern generator and the vref setting, so
the second capture no longer matches what the user configured.

The check configures pattern+vref exactly once, then runs N back-to-back
Start/Stop cycles without touching the configuration again.  Every capture
after the first must still show the structured test pattern; floating input
noise would mean the configuration was lost.

Usage: test_config_persistence.py [cycles] [samples]
"""
import argparse
import json
import time
import urllib.request

URL = "http://127.0.0.1:10110/mcp"
VTH = 2.0


def call(name, args=None, timeout=180):
    body = json.dumps({"jsonrpc": "2.0", "id": int(time.time() * 1000) % 10**9,
                       "method": "tools/call",
                       "params": {"name": name, "arguments": args or {}}}).encode()
    req = urllib.request.Request(URL, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        text = json.load(resp)["result"]["content"][0]["text"]
    try:
        return json.loads(text)
    except ValueError:
        return text


def read_csv(path, max_samples):
    rows = []
    with open(path) as handle:
        handle.readline()
        for line in handle:
            rows.append([int(v) for v in line.strip().split(",")[1:]])
            if len(rows) >= max_samples:
                break
    return rows


def analyse(path, max_samples):
    rows = read_csv(path, max_samples)
    nch = len(rows[0])
    trans = [0] * nch
    ones = [0] * nch
    for a, b in zip(rows, rows[1:]):
        for c in range(nch):
            trans[c] += a[c] != b[c]
            ones[c] += b[c]
    duty = [100.0 * ones[c] / len(rows) for c in range(nch)]
    quiet = [c for c in range(nch) if trans[c] <= len(rows) * 0.02]
    return rows, duty, trans, quiet


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("cycles", nargs="?", type=int, default=3)
    parser.add_argument("samples", nargs="?", type=int, default=20000000)
    args = parser.parse_args()

    # Configure once: pattern generator + a non-default vref.
    call("select_device", {"index": 0})
    time.sleep(0.4)
    call("configure", {"pattern": "Emulation", "vth": VTH, "channel_mode": 0,
                       "samplerate": "200MHz", "sample_count": args.samples})
    status = call("get_status")
    print(f"configured: pattern={status.get('pattern')} vth={VTH} "
          f"samplerate={status.get('samplerate_hz')} "
          f"samples={status.get('sample_count')}")

    failed = False
    for cycle in range(1, args.cycles + 1):
        # No configure() here on purpose - only Start/Stop.
        # The U3 firmware occasionally misses RUN on the first attempt after
        # the engine was reconfigured; that is orthogonal to what this test
        # checks, so an empty transfer is retried instead of failing.
        for attempt in range(1, 4):
            t0 = time.time()
            call("start_capture")
            state = call("wait_capture", {"timeout_seconds": 60})
            got = state.get("captured_samples", -1)
            status = call("get_status")
            wall = time.time() - t0
            tag = ("FULL" if got == args.samples else
                   "EMPTY" if got == 0 else "EARLY")
            print(f"  cycle {cycle}.{attempt}: samples={got} ({tag}) "
                  f"wall={wall:.3f}s pattern={status.get('pattern')}",
                  flush=True)
            if status.get("pattern") != "Emulation":
                failed = True
            if got > 0:
                break

        path = f"/tmp/config_persistence_{cycle}.csv"
        call("export_csv", {"path": path, "max_samples": 4000})
        rows, duty, trans, quiet = analyse(path, 4000)
        print(f"    exported={len(rows)} quiet_channels={len(quiet)}")
        print("    duty%: " + " ".join(f"{d:4.0f}" for d in duty))
        print("    trans: " + " ".join(f"{t:4d}" for t in trans))
        if len(quiet) < 4:
            print("    FAIL: data looks like floating inputs - the pattern "
                  "generator did not survive this Start")
            failed = True

    print("RESULT: " + ("FAIL" if failed else
                        f"PASS - pattern+vref survived {args.cycles} "
                        "Start/Stop cycles with no reconfigure"))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
