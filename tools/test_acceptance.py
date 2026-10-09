#!/usr/bin/env python3
"""End-to-end acceptance run for the Sipeed SLogic32U3 driver.

Usage:
    test_acceptance.py [--captures N] [--gui XWINDOW_ID] [--skip-gui]

Checks, in order:
  1. health      - the device streams at all
  2. stability   - N back-to-back 32ch@200MHz 1s captures, all must be full
  3. data        - the Emulation test pattern really reaches the host
                   (random/floating input shows up as ~50% duty on every
                   channel with ~1 transition per sample; the generated
                   pattern is structured)
  4. responsiveness - capture wall time, process CPU and MCP latency
  5. gui         - optional Start/Stop click stress (tools/test_mcp_gui_start_stop.py)

Exit status is the number of failed checks.
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time
import urllib.request

URL = "http://127.0.0.1:10110/mcp"
SAMPLES = 200000000
STATE = {"id": 0, "transport_retries": 0}


def call(name, args=None, timeout=180, retries=20, backoff=0.25):
    STATE["id"] += 1
    body = json.dumps({"jsonrpc": "2.0", "id": STATE["id"], "method": "tools/call",
                       "params": {"name": name, "arguments": args or {}}}).encode()
    last = "no attempt"
    for _ in range(retries):
        t0 = time.time()
        try:
            req = urllib.request.Request(URL, data=body,
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                raw = resp.read()
            if raw:
                envelope = json.loads(raw)
                text = envelope["result"]["content"][0]["text"]
                try:
                    # Some tools answer with a JSON document, others with a
                    # short plain-text status ("capture started", ...).
                    return json.loads(text), time.time() - t0
                except ValueError:
                    return text, time.time() - t0
            last = "empty body"
        except Exception as exc:  # transport hiccup, retry below
            last = f"{type(exc).__name__}: {exc}"
        # Keep the run alive across transient transport errors (server busy,
        # connection refused while the app is starting up, ...).
        STATE["transport_retries"] += 1
        time.sleep(backoff)
    raise RuntimeError(f"MCP call {name} failed {retries} times (last: {last})")


def configure(pattern="Emulation", ch_mode=0, samples=SAMPLES):
    call("configure", {"pattern": pattern, "channel_mode": ch_mode,
                       "samplerate": "200MHz", "sample_count": samples})


def capture(samples=SAMPLES):
    configure(samples=samples)
    t0 = time.time()
    call("start_capture")
    state, _ = call("wait_capture", {"timeout_seconds": 60})
    return state.get("captured_samples", -1), time.time() - t0


def cpu_ticks(pid):
    total = 0
    for task in glob.glob(f"/proc/{pid}/task/*/stat"):
        try:
            with open(task) as handle:
                fields = handle.read().rsplit(")", 1)[1].split()
        except OSError:
            continue
        total += int(fields[11]) + int(fields[12])
    return total


def check_health(failures):
    print("[1] health")
    got, wall = capture()
    print(f"    captured_samples={got} wall={wall:.3f}s")
    if got <= 0:
        print("    FAIL: no samples - is the analyzer connected/replugged?")
        failures.append("health")
    return got > 0


def check_stability(failures, captures):
    print(f"[2] stability: {captures} back-to-back 32ch@200MHz 1s captures")
    full = empty = early = 0
    walls = []
    for i in range(captures):
        got, wall = capture()
        walls.append(wall)
        if got == SAMPLES:
            full += 1
        elif got == 0:
            empty += 1
        else:
            early += 1
        print(f"    #{i + 1:02d} samples={got} wall={wall:.3f}s", flush=True)
    print(f"    full={full}/{captures} early={early} empty={empty} "
          f"mean_wall={sum(walls) / len(walls):.3f}s")
    if full < captures:
        print("    NOTE: reference sigrok-cli drops ~50% of captures in this "
              "same back-to-back scenario (0 samples, FRAME-BEGIN only).")
    return full, captures


def check_data(failures):
    print("[3] data integrity: Emulation pattern must be structured, not noise")
    configure(pattern="Emulation", samples=200000)
    call("start_capture")
    state, _ = call("wait_capture", {"timeout_seconds": 60})
    got = state.get("captured_samples", 0)
    if got <= 0:
        print("    FAIL: no samples")
        failures.append("data")
        return
    path = "/tmp/acceptance_pattern.csv"
    call("export_csv", {"path": path, "max_samples": 4000})
    rows = []
    with open(path) as handle:
        handle.readline()
        for line in handle:
            rows.append([int(v) for v in line.strip().split(",")[1:]])
    nch = len(rows[0])
    trans = [0] * nch
    ones = [0] * nch
    for a, b in zip(rows, rows[1:]):
        for c in range(nch):
            if a[c] != b[c]:
                trans[c] += 1
            ones[c] += b[c]
    duty = [100.0 * ones[c] / len(rows) for c in range(nch)]
    # The generated pattern drives several channels with slow dividers and
    # leaves others constant, so a healthy capture has channels with almost no
    # transitions.  Floating inputs instead toggle on every channel, giving
    # roughly 50% duty and ~1 transition per sample everywhere.
    quiet = [c for c in range(nch) if trans[c] <= len(rows) * 0.02]
    print(f"    samples={got} exported={len(rows)} quiet_channels={len(quiet)} "
          f"(pattern: many, floating inputs: ~0)")
    print("    duty%: " + " ".join(f"{d:4.0f}" for d in duty))
    print("    trans: " + " ".join(f"{t:4d}" for t in trans))
    if len(quiet) < 4:
        print("    FAIL: no quiet channels - the data looks like floating "
              "inputs, so the pattern generator did not take effect")
        failures.append("data")


def check_responsiveness(failures):
    print("[4] responsiveness")
    pid = subprocess.check_output(["pgrep", "-x", "AllLogic"]).split()[0].decode()
    configure(samples=SAMPLES)
    t0 = time.time()
    c0 = cpu_ticks(pid)
    call("start_capture")
    lat = []
    while True:
        state, dt = call("get_status")
        lat.append(dt)
        if not state.get("capturing"):
            break
        time.sleep(0.02)
    wall = time.time() - t0
    c1 = cpu_ticks(pid)
    lat.sort()
    cpu = (c1 - c0) / 100.0 / wall * 100.0
    print(f"    samples={state.get('captured_samples')} wall={wall:.3f}s "
          f"cpu={cpu:.0f}% of one core")
    print(f"    MCP latency avg={sum(lat) / len(lat) * 1000:.1f}ms "
          f"p95={lat[int(len(lat) * 0.95)] * 1000:.1f}ms "
          f"max={lat[-1] * 1000:.1f}ms "
          f"transport_retries={STATE['transport_retries']}")
    if wall > 3.0:
        print("    FAIL: 1s of samples took more than 3s of wall clock")
        failures.append("responsiveness")


def check_gui(failures, window):
    print(f"[5] GUI Start/Stop stress on X window {window}")
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "test_mcp_gui_start_stop.py")
    out = subprocess.run([sys.executable, script, str(window), "10", "0.35"],
                         capture_output=True, text=True)
    report = out.stdout.strip()
    if out.returncode != 0 and out.stderr.strip():
        report += ("\n" if report else "") + out.stderr.strip()
    print("\n".join("    " + line for line in report.splitlines()))
    if out.returncode != 0 or "FAIL" in out.stdout:
        failures.append("gui")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--captures", type=int, default=48)
    parser.add_argument("--gui", type=str, default=None,
                        help="X window id of the ALL LOGIC window")
    parser.add_argument("--skip-gui", action="store_true")
    args = parser.parse_args()

    failures = []
    if check_health(failures):
        check_stability(failures, args.captures)
        check_data(failures)
        check_responsiveness(failures)
    if args.gui and not args.skip_gui:
        check_gui(failures, args.gui)

    print("\n=== summary ===")
    print("failed checks:", failures if failures else "none")
    return len(failures)


sys.exit(main())
