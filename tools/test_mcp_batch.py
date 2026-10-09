#!/usr/bin/env python3
"""Batch capture harness for the ALL-LOGIC MCP server.

Usage: test_mcp_batch.py [batches] [captures_per_batch] [delay_s] [samples]

Each batch re-selects (re-opens) the device, then performs N captures
back-to-back.  `delay_s` inserts a wait between the end of one capture and
the start of the next, which is used to test whether the analyzer needs
quiet time after STOP before it accepts the next RUN.
"""
import json
import sys
import time
import urllib.request

URL = "http://127.0.0.1:10110/mcp"
SAMPLES = 200000000
_id = 0


def call(name, args=None, timeout=180):
    global _id
    _id += 1
    body = json.dumps({"jsonrpc": "2.0", "id": _id, "method": "tools/call",
                       "params": {"name": name, "arguments": args or {}}}).encode()
    req = urllib.request.Request(URL, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        payload = json.load(resp)
    text = payload["result"]["content"][0]["text"]
    try:
        return json.loads(text)
    except ValueError:
        return text


def capture(samples):
    call("configure", {"pattern": "Emulation", "channel_mode": 0,
                       "samplerate": "200MHz", "sample_count": samples})
    t0 = time.time()
    call("start_capture")
    state = call("wait_capture", {"timeout_seconds": 60})
    return state.get("captured_samples", -1), time.time() - t0


def main():
    batches = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    per = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    delay = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    samples = int(sys.argv[4]) if len(sys.argv) > 4 else SAMPLES
    rows = []
    for batch in range(batches):
        call("select_device", {"index": 0})
        time.sleep(0.4)
        for index in range(per):
            got, wall = capture(samples)
            tag = ("FULL" if got == samples else
                   "EMPTY" if got == 0 else "EARLY")
            rows.append((batch, index, got, wall, tag))
            print(f"batch={batch} cap={index} samples={got} "
                  f"wall={wall:.3f}s {tag}", flush=True)
            if delay > 0:
                time.sleep(delay)
    print("\n=== summary ===")
    for index in range(per):
        sub = [r for r in rows if r[1] == index]
        full = sum(1 for r in sub if r[4] == "FULL")
        print(f"capture#{index}: {full}/{len(sub)} full  "
              f"walls={[round(r[3], 2) for r in sub]}")
    full_total = sum(1 for r in rows if r[4] == "FULL")
    print(f"total full {full_total}/{len(rows)}")
    print("failures:", [r for r in rows if r[4] != "FULL"])


main()
