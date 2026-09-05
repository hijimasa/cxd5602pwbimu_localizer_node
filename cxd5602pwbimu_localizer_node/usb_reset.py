#!/usr/bin/env python3
"""Reset the USB-serial bridge (CP210x) of the Spresense board.

A high-rate binary stream (RAW_LOG_OUTPUT) read at the wrong baud rate can
leave the CP210x bridge in a state where nothing is received any more and
the flash writer hangs waiting for the boot prompt. A USB port reset
(re-enumeration) recovers it without unplugging. The board itself keeps
running; use it before uploading if the upload hangs.

Usage: usb_reset_cp210x.py [VID:PID]   (default 10c4:ea60)
"""
import fcntl
import glob
import os
import sys

USBDEVFS_RESET = 0x5514


def find_device(vid, pid):
    for dev in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(dev, "idVendor")) as f:
                v = f.read().strip()
            with open(os.path.join(dev, "idProduct")) as f:
                p = f.read().strip()
        except OSError:
            continue
        if v == vid and p == pid:
            with open(os.path.join(dev, "busnum")) as f:
                bus = int(f.read())
            with open(os.path.join(dev, "devnum")) as f:
                num = int(f.read())
            return f"/dev/bus/usb/{bus:03d}/{num:03d}"
    return None


def reset_by_vid_pid(vid_pid="10c4:ea60"):
    """Reset the first USB device matching VID:PID; returns its device node.
    Raises on failure (no such device, no permission, not Linux)."""
    if not sys.platform.startswith("linux"):
        raise RuntimeError("USB reset is only supported on Linux")
    vid, pid = vid_pid.lower().split(":")
    node = find_device(vid, pid)
    if node is None:
        raise RuntimeError(f"no USB device {vid}:{pid} found")
    fd = os.open(node, os.O_WRONLY)
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)
    return node


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    try:
        node = reset_by_vid_pid(argv[0] if argv else "10c4:ea60")
    except Exception as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)
    print(f"reset {node}")


if __name__ == "__main__":
    main()
