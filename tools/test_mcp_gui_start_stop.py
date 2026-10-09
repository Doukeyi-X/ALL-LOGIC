#!/usr/bin/env python3
"""Click-driven Start/Stop stress test for the ALL-LOGIC window.

Usage: test_mcp_gui_start_stop.py <xwin-id> [rounds] [hold_s]

Each round clicks the toolbar Start button, waits `hold_s`, clicks Stop and
measures how long the UI takes to fall back to idle.  MCP is polled
throughout so the responsiveness of the GUI thread is measured at the same
time.
"""
import json
import subprocess
import sys
import time
import urllib.request

URL = "http://127.0.0.1:10110/mcp"
# Toolbar Start/Stop button centre, measured on a 3440x1440 fullscreen window.
START_BTN = (635, 72)
_id = 0


def call(name, args=None, timeout=30):
    global _id
    _id += 1
    body = json.dumps({"jsonrpc": "2.0", "id": _id, "method": "tools/call",
                       "params": {"name": name, "arguments": args or {}}}).encode()
    req = urllib.request.Request(URL, data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        payload = json.load(resp)
    latency = time.time() - t0
    text = payload["result"]["content"][0]["text"]
    try:
        return json.loads(text), latency
    except ValueError:
        return text, latency


def click(win, x, y):
    subprocess.run(["xdotool", "mousemove", "--window", str(win), str(x), str(y)],
                   check=True)
    subprocess.run(["xdotool", "click", "1"], check=True)


def capturing():
    state, latency = call("get_status")
    return bool(state.get("capturing")), state, latency


def main():
    win = sys.argv[1]
    rounds = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    hold = float(sys.argv[3]) if len(sys.argv) > 3 else 0.35
    # Clicking by window-relative coordinates only lands on the toolbar if the
    # window is the active one, so ask the compositor for focus first.  The
    # compositor may refuse (BadMatch) while the window is being remapped, so
    # ignore failures here and let the click retry below handle it.
    for args in (["windowactivate", str(win)], ["windowfocus", str(win)]):
        subprocess.run(["xdotool"] + args, check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.4)
    latencies = []
    for r in range(rounds):
        click(win, *START_BTN)
        t_click = time.time()
        start_wait = None
        while time.time() - t_click < 5:
            busy, state, latency = capturing()
            latencies.append(latency)
            if busy:
                start_wait = time.time() - t_click
                break
            time.sleep(0.02)
        if start_wait is None:
            # A click can be swallowed while the window is still settling;
            # click once more before declaring the round failed.
            click(win, *START_BTN)
            t_retry = time.time()
            while time.time() - t_retry < 5:
                busy, state, latency = capturing()
                latencies.append(latency)
                if busy:
                    start_wait = time.time() - t_retry
                    break
                time.sleep(0.02)
        time.sleep(hold)
        click(win, *START_BTN)
        t_stop = time.time()
        stop_wait = None
        stop_retries = 0
        while time.time() - t_stop < 5:
            busy, state, latency = capturing()
            latencies.append(latency)
            if not busy:
                stop_wait = time.time() - t_stop
                break
            # The toolbar action is ignored while the button is disabled
            # (e.g. right around a state transition), so click again the way
            # a user would and count how often that is needed.
            if time.time() - t_stop > 0.3 * (stop_retries + 1) and stop_retries < 2:
                click(win, *START_BTN)
                stop_retries += 1
            time.sleep(0.02)
        samples = state.get("captured_samples", -1)
        ok = "OK" if (start_wait is not None and stop_wait is not None
                      and samples > 0) else "FAIL"
        start_ms = "-" if start_wait is None else f"{start_wait * 1000:.0f}"
        stop_ms = "-" if stop_wait is None else f"{stop_wait * 1000:.0f}"
        print(f"round={r:02d} start_ms={start_ms} "
              f"stop_ms={stop_ms} stop_retries={stop_retries} "
              f"samples={samples} {ok}",
              flush=True)
    latencies.sort()
    if latencies:
        print(f"\nMCP latency n={len(latencies)} "
              f"avg={sum(latencies) / len(latencies) * 1000:.1f}ms "
              f"p95={latencies[int(len(latencies) * 0.95)] * 1000:.1f}ms "
              f"max={latencies[-1] * 1000:.1f}ms")


main()
