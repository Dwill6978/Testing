#!/usr/bin/env bash
# Soft "replug" for the GREAT PLANES InterLink-X (USB 1781:0e59).
#
# Under Parallels USB passthrough the controller stops streaming axis motion
# after the first program run and needs a physical unplug/replug. This issues a
# real USB port reset (USBDEVFS_RESET ioctl) on the device, which is the closest
# software equivalent to unplugging and replugging.
#
# An earlier version toggled the sysfs `authorized` flag; that re-enumerated the
# descriptors but left the InterLink-X streaming nothing (all axes stuck at
# -1.0), so it's kept only as a last-ditch fallback.
#
# Usage:  sudo ./reset_controller.sh
#
# The device path/bus number changes on every replug, so we locate it by its
# fixed vendor:product ID rather than a hard-coded path.

set -euo pipefail

VENDOR="1781"
PRODUCT="0e59"

if [[ $EUID -ne 0 ]]; then
    echo "Needs root to reset the USB device. Re-run: sudo $0" >&2
    exit 1
fi

dev=""
for d in /sys/bus/usb/devices/*/; do
    [[ -f "$d/idVendor" && -f "$d/idProduct" ]] || continue
    if [[ "$(cat "$d/idVendor")" == "$VENDOR" && "$(cat "$d/idProduct")" == "$PRODUCT" ]]; then
        dev="$d"
        break
    fi
done

if [[ -z "$dev" ]]; then
    echo "InterLink-X ($VENDOR:$PRODUCT) not found on the USB bus." >&2
    echo "Is it plugged in and passed through to the VM?" >&2
    exit 1
fi

name="$(cat "$dev/product" 2>/dev/null || echo unknown)"
busnum="$(cat "$dev/busnum")"
devnum="$(cat "$dev/devnum")"
node="$(printf '/dev/bus/usb/%03d/%03d' "$busnum" "$devnum")"
echo "Found $name at $dev -> $node — USB port reset..."

# Real port reset via USBDEVFS_RESET (_IO('U', 20) == 0x5514). Runs as root
# here, so the embedded python inherits root and can ioctl the device node.
if python3 - "$node" <<'PY'
import fcntl, sys
USBDEVFS_RESET = 0x5514
node = sys.argv[1]
with open(node, "wb") as f:
    fcntl.ioctl(f, USBDEVFS_RESET, 0)
print("  USBDEVFS_RESET ok")
PY
then
    sleep 2
    echo "Done. Give it ~2s to re-enumerate, then run the tester/GUI."
    exit 0
fi

# Fallback: authorized toggle (descriptor re-enumerate). Known to leave this
# device non-streaming under Parallels, but better than nothing on other setups.
echo "  USBDEVFS_RESET failed; falling back to authorized toggle..." >&2
echo 0 > "$dev/authorized"
sleep 1
echo 1 > "$dev/authorized"
sleep 1
echo "Done (fallback). If axes read all -1.0, physically replug instead."
