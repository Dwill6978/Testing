#!/usr/bin/env python3
"""InterLink-X (and any joystick) input tester / mapping prototyper.

Purpose
-------
The glider GUI could not "register" any InterLink-X input. This standalone
script isolates the raw pygame read pipeline from the GUI/Crazyflie so we can
answer two questions on real hardware:

  1. Can pygame read the InterLink-X at all in this environment/thread?
  2. What are the ACTUAL axis / button / hat indices and rest values?

Once we know the real layout, we plug those numbers straight into RC_PROFILE
in gui_GliderControlTest.py (roll_axis / pitch_axis / yaw_axis / throttle_axis
and the matching *_sign flags).

Usage
-----
  python3 interlink_tester.py            # live monitor: prints every axis/button/hat
  python3 interlink_tester.py --wizard   # guided: move one control at a time,
                                          #   script auto-detects which axis it is
  python3 interlink_tester.py --list     # just enumerate connected devices and exit
  # Add --no-reset to any invocation to skip the startup USB soft-replug.

Notes
-----
* Pure pygame + stdlib. No cflib, no Qt. If this can't read the stick, the GUI
  never will, and the problem is the driver/device, not the GUI code.
* Mirrors exactly how the GUI reads: pygame.init() -> joystick.init() ->
  event.pump() -> get_axis()/get_button()/get_hat(). Same call sequence.
"""

import glob
import os
import subprocess
import sys
import time

try:
    import pygame
except ImportError:
    sys.exit("pygame is not installed. Install with: pip install pygame")

# Without this hint SDL only reports joystick input while its own window has
# input focus, so reads go dead the moment the terminal (or any other window)
# is focused. Must be set before pygame.init().
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

# Root-owned copy of reset_controller.sh that a NOPASSWD sudoers entry allows us
# to run unattended. See the one-time setup in the project notes. If it isn't
# installed / not granted, the soft-replug below quietly no-ops.
RESET_CMD = "/usr/local/sbin/reset_controller.sh"


def _soft_replug():
    """Best-effort USB re-enumerate of the InterLink-X so a repeat run streams
    axis motion again (Parallels passthrough wedges it after the first run).

    Runs `sudo -n` so it never blocks on a password prompt: if the NOPASSWD
    entry isn't set up it just returns and you can physically replug instead.
    """
    if not os.path.exists(RESET_CMD):
        return
    try:
        r = subprocess.run(["sudo", "-n", RESET_CMD],
                           timeout=8, capture_output=True, text=True)
    except Exception:
        return
    if r.returncode != 0:
        return  # most likely: no passwordless-sudo entry -> replug manually
    print("Soft-replugged the controller (USB re-enumerate).")
    # Wait for the joystick node to reappear after re-enumeration.
    end = time.time() + 4.0
    while time.time() < end:
        if glob.glob("/dev/input/js*"):
            time.sleep(0.5)  # small settle once the node is back
            return
        time.sleep(0.1)


def _is_streaming(js, dwell=1.0):
    """True once the device sends real data. A device that's present but not
    streaming (wedged, or reopened before it finished re-enumerating) reads
    exactly -1.0 on every axis; a live one never does (throttle rests ~+0.8).
    Pump for up to `dwell` seconds waiting for any axis to leave -1.0."""
    n = js.get_numaxes()
    if not n:
        return False
    end = time.time() + dwell
    while time.time() < end:
        pygame.event.get()
        if not all(js.get_axis(i) == -1.0 for i in range(n)):
            return True
        time.sleep(0.05)
    return False


def open_joystick(retries=8, settle=1.5):
    """Open the InterLink-X, retrying until it actually streams. A too-fast
    reopen (or a device mid-reset) shows up as all axes -1.0; rather than hang
    on that, we tear SDL down and retry, re-initialising each time so a
    re-enumerated device with a new node is picked up."""
    for attempt in range(retries):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            js = pygame.joystick.Joystick(0)
            js.init()
            if _is_streaming(js):
                return js
            print(f"  controller present but not streaming yet "
                  f"(attempt {attempt + 1}/{retries}); waiting...")
        else:
            print(f"  no joystick yet (attempt {attempt + 1}/{retries}); "
                  f"waiting for re-enumeration...")
        close_joystick(None)   # full SDL teardown before the next attempt
        time.sleep(settle)
    sys.exit("Controller never started streaming (all axes stuck at -1.0). "
             "Physically unplug/replug it and try again.")


def describe(js):
    print("=" * 60)
    print(f"Device : {js.get_name()}")
    try:
        print(f"GUID   : {js.get_guid()}")
    except Exception:
        pass
    print(f"Axes   : {js.get_numaxes()}")
    print(f"Buttons: {js.get_numbuttons()}")
    print(f"Hats   : {js.get_numhats()}")
    print("=" * 60)


def _drain():
    """Pump AND clear the SDL event queue.

    Polling with only pygame.event.pump() lets the queue fill with the flood of
    JOYAXISMOTION events a moving stick produces; once it's full SDL stops
    updating and get_axis() freezes at the last value (buttons, being rare,
    still sneak through — which is exactly the "rest state OK, no movement"
    symptom). Draining with get() every tick keeps joystick state live.
    """
    pygame.event.get()


def read_axes(js):
    return [round(js.get_axis(i), 3) for i in range(js.get_numaxes())]


def read_buttons(js):
    return [js.get_button(i) for i in range(js.get_numbuttons())]


def read_hats(js):
    return [js.get_hat(i) for i in range(js.get_numhats())]


def cmd_list():
    pygame.init()
    pygame.joystick.init()
    n = pygame.joystick.get_count()
    print(f"{n} joystick(s) detected:")
    for i in range(n):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(f"  [{i}] {js.get_name()}  "
              f"axes={js.get_numaxes()} buttons={js.get_numbuttons()} "
              f"hats={js.get_numhats()}")


def cmd_monitor(js, move_thresh=0.08):
    """Event-style live monitor: prints a NEW line only when something changes,
    so it works even if your terminal mangles carriage-return redraws, and it's
    easy to read off which index reacted. A periodic heartbeat proves the loop
    is alive even when the device is quiet."""
    describe(js)
    axes = read_axes(js)
    print("\nLive monitor. Move ONE control at a time; the index that reacts")
    print("prints on its own line. Ctrl-C to quit.")
    print(f"Rest/center axis values: {axes}\n")

    last_axes = list(axes)
    last_btns = read_buttons(js)
    last_hats = read_hats(js)
    last_beat = time.time()
    try:
        while True:
            _drain()
            axes = read_axes(js)
            btns = read_buttons(js)
            hats = read_hats(js)

            changed = False
            for i in range(len(axes)):
                if abs(axes[i] - last_axes[i]) >= move_thresh:
                    print(f"  axis a{i}: {last_axes[i]:+.2f} -> {axes[i]:+.2f}")
                    last_axes[i] = axes[i]
                    changed = True
            for i in range(len(btns)):
                if btns[i] != (last_btns[i] if i < len(last_btns) else 0):
                    print(f"  button {i}: {'PRESSED' if btns[i] else 'released'}")
                    changed = True
            last_btns = btns
            if hats != last_hats:
                print(f"  hats: {last_hats} -> {hats}")
                last_hats = hats
                changed = True

            now = time.time()
            if changed:
                last_beat = now
            elif now - last_beat >= 2.0:
                # Heartbeat: distinguishes "running, device quiet" from "dead".
                print(f"  ...monitoring (move a control)   "
                      f"axes now = {axes}")
                last_beat = now
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\nFinal axis values:", read_axes(js))
        print("Done.")


def _sample_baseline(js, seconds=0.6):
    """Pump for a moment with sticks centered, return the resting axis values."""
    end = time.time() + seconds
    axes = read_axes(js)
    while time.time() < end:
        _drain()
        axes = read_axes(js)
        time.sleep(0.02)
    return axes


def _capture_axis(js, baseline, seconds=5.0):
    """Watch every axis for `seconds` and lock the one that deflects the MOST
    from baseline.

    This replaces the old fixed-threshold wait, which hung forever on axes that
    rest near an end-stop (InterLink-X a2/a3/a4/a6 idle at ~+0.8, so they can
    only travel ~0.2 and never crossed a 0.30 threshold). By ranking on peak
    deviation instead, an end-stop axis is caught as easily as a centered one,
    and the live countdown means it never looks hung.
    """
    n = js.get_numaxes()
    peak = [0.0] * n           # signed peak deviation from baseline per axis
    end = time.time() + seconds
    while time.time() < end:
        _drain()
        axes = read_axes(js)
        for i in range(n):
            d = axes[i] - baseline[i]
            if abs(d) > abs(peak[i]):
                peak[i] = d
        lead = max(range(n), key=lambda i: abs(peak[i]))
        remaining = end - time.time()
        print(f"    capturing... {remaining:3.1f}s   "
              f"leading a{lead} (dev {peak[lead]:+.2f})".ljust(70),
              end="\r", flush=True)
        time.sleep(0.03)
    idx = max(range(n), key=lambda i: abs(peak[i]))
    print(" " * 72, end="\r")   # clear the countdown line
    return idx, (1.0 if peak[idx] > 0 else -1.0), round(peak[idx], 3)


def cmd_wizard(js):
    describe(js)
    print("\nGuided mapping. For each prompt, press Enter and you then have ~5s to")
    print("move ONLY that control fully in the named direction (a live countdown")
    print("shows which axis is leading). The biggest-moving axis is locked.\n")
    print("Standard Mode-2 layout assumed:")
    print("  right stick = elevator(pitch) + aileron(roll)")
    print("  left  stick = rudder(yaw)     + throttle\n")
    input("Center all sticks, then press Enter to sample the rest state...")

    baseline = _sample_baseline(js)
    print(f"Rest/center axis values: {[round(v, 3) for v in baseline]}\n")

    steps = [
        ("roll_axis",     "Move RIGHT stick fully RIGHT (aileron)"),
        ("pitch_axis",    "Move RIGHT stick fully UP (elevator)"),
        ("yaw_axis",      "Move LEFT stick fully RIGHT (rudder)"),
        ("throttle_axis", "Move LEFT stick fully UP (throttle high)"),
    ]
    result = {}
    for key, prompt in steps:
        input(f">>> {prompt}. Press Enter, then move it...")
        idx, direction, dev = _capture_axis(js, baseline)
        if abs(dev) < 0.15:
            print(f"    WARNING: barely any movement detected (peak dev {dev:+.2f}). "
                  f"This axis may be wrong \u2014 re-run the wizard if so.")
        result[key] = (idx, direction, dev)
        print(f"    locked {key}: axis a{idx}  (peak dev {dev:+.2f} "
              f"when moved in the named direction)\n")

    print("=" * 60)
    print("Detected mapping -> paste these into RC_PROFILE:")
    print("=" * 60)
    print(f"    roll_axis={result['roll_axis'][0]}, "
          f"pitch_axis={result['pitch_axis'][0]}, "
          f"yaw_axis={result['yaw_axis'][0]}, "
          f"throttle_axis={result['throttle_axis'][0]},")
    print("    # Signs default to +1.0. Fly/test each surface; if one moves the")
    print("    # wrong way, flip that single *_sign to -1.0. Observed raw")
    print("    # direction for the named gesture (for reference):")
    for key in ("roll_axis", "pitch_axis", "yaw_axis", "throttle_axis"):
        idx, direction, dev = result[key]
        print(f"    #   {key:<13} a{idx}: moved {'+' if direction > 0 else '-'} "
              f"({dev:+.2f})")
    print("=" * 60)
    print("\nNow press each BUTTON you want to use; indices print live.")
    print("Ctrl-C when done.\n")
    seen = set()
    try:
        while True:
            _drain()
            for i in range(js.get_numbuttons()):
                if js.get_button(i) and i not in seen:
                    seen.add(i)
                    print(f"    button index {i} pressed")
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\nButtons seen:", sorted(seen) or "none")
        print("Done.")


def close_joystick(js=None):
    """Release the device the way SDL wants: quit the joystick object, then the
    joystick subsystem, then pygame. Skipping this (just letting the process
    die) can leave the InterLink-X wedged under Parallels USB passthrough so a
    SECOND run reads only static values until you replug. Best-effort."""
    try:
        if js is not None:
            js.quit()
    except Exception:
        pass
    try:
        pygame.joystick.quit()
    except Exception:
        pass
    try:
        pygame.quit()
    except Exception:
        pass


def main():
    args = set(sys.argv[1:])
    if "--no-reset" not in args:
        _soft_replug()   # re-enumerate first so a repeat run isn't wedged
    if "--list" in args:
        cmd_list()
        close_joystick()
        return
    js = open_joystick()
    try:
        if "--wizard" in args:
            cmd_wizard(js)
        else:
            cmd_monitor(js)
    finally:
        close_joystick(js)


if __name__ == "__main__":
    main()
