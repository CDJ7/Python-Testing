#!/usr/bin/env python3
"""Step 2: WASD keyboard control for a Unitree Go2 over WiFi (go2_webrtc_connect).

W/S forward/back   A/D turn left/right   Q/E strafe left/right
SPACE stop         ESC or Ctrl+C stop and exit

Only sends Move / StopMove. No stand up, lie down or other special moves.
Run:  python unitree/go2_wasd.py            (firmware < 1.1.15)
      UNITREE_AES_128_KEY=<32 hex> python unitree/go2_wasd.py   (firmware >= 1.1.15)
"""
import argparse
import asyncio
import json
import os
import signal
import sys
import termios
import time
import tty

from unitree_webrtc_connect.constants import DATA_CHANNEL_TYPE, RTC_TOPIC, SPORT_CMD
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

# ---- Tunable settings (start low!) -------------------------------------
SPEED_FORWARD = 0.25    # m/s
SPEED_BACKWARD = 0.20   # m/s
SPEED_STRAFE = 0.15     # m/s
SPEED_TURN = 0.50       # rad/s
SEND_HZ = 10            # how often the current command is re-sent
# A terminal can't report key *release*, so "held" = key events keep arriving.
# After the first press the OS waits before auto-repeating, so allow a longer
# window for that first press, then a short one once repeats are flowing.
FIRST_PRESS_HOLD = 0.5  # s of movement after a fresh press (also = a single tap)
REPEAT_HOLD = 0.15      # s of movement after each auto-repeat event
# -------------------------------------------------------------------------

# key -> (x forward, y left, z yaw-left)
KEYS = {
    "w": (SPEED_FORWARD, 0.0, 0.0),
    "s": (-SPEED_BACKWARD, 0.0, 0.0),
    "a": (0.0, 0.0, SPEED_TURN),
    "d": (0.0, 0.0, -SPEED_TURN),
    "q": (0.0, SPEED_STRAFE, 0.0),
    "e": (0.0, -SPEED_STRAFE, 0.0),
}


class Dog:
    def __init__(self, conn):
        self.conn = conn

    def _channel_open(self):
        return self.conn.datachannel.channel.readyState == "open"

    def send(self, topic, api_id, parameter=None):
        if not self._channel_open():
            raise ConnectionError("data channel is not open")
        # fire-and-forget request (same payload publish_request_new builds)
        payload = {
            "header": {"identity": {"id": int(time.time() * 1000) % 2147483648, "api_id": api_id}},
            "parameter": json.dumps(parameter) if parameter is not None else "",
        }
        self.conn.datachannel.pub_sub.publish_without_callback(
            topic, payload, DATA_CHANNEL_TYPE["REQUEST"])

    def move(self, x, y, z):
        self.send(RTC_TOPIC["SPORT_MOD"], SPORT_CMD["Move"], {"x": x, "y": y, "z": z})

    def stop(self):
        """Best effort, never raises. Sent several times in case a packet is lost."""
        for _ in range(3):
            try:
                self.move(0.0, 0.0, 0.0)
                self.send(RTC_TOPIC["SPORT_MOD"], SPORT_CMD["StopMove"])
            except Exception:
                pass


def status(text):
    sys.stdout.write("\r\x1b[K" + text)
    sys.stdout.flush()


async def check_motion_mode(conn):
    """Read-only: warn if the dog isn't in 'normal' motion mode (we never switch it)."""
    try:
        resp = await asyncio.wait_for(conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001}), 5)
        mode = json.loads(resp["data"]["data"])["name"]
        print(f"Motion mode: {mode}")
        if mode != "normal":
            print("  WARNING: not 'normal' mode, the dog may ignore Move commands.")
    except Exception as e:
        print(f"(could not read motion mode: {e})")


async def run(ip, aes_key):
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip=ip,
                                   aes_128_key=aes_key or None)
    print("Connecting to the Go2 (close the Unitree phone app first)...")
    await conn.connect()
    dog = Dog(conn)
    print("Connected.")
    await check_motion_mode(conn)

    loop = asyncio.get_running_loop()
    quit_evt = asyncio.Event()
    last = {"key": None, "deadline": 0.0}

    def on_sigint():
        quit_evt.set()

    loop.add_signal_handler(signal.SIGINT, on_sigint)
    loop.add_signal_handler(signal.SIGTERM, on_sigint)

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)  # keeps Ctrl+C working, no Enter needed, no echo

    def on_stdin():
        data = os.read(fd, 64)
        if data == b"\x1b":          # bare ESC (arrow keys send longer sequences)
            quit_evt.set()
            return
        now = time.monotonic()
        for ch in data.decode(errors="ignore").lower():
            if ch == " ":
                last["key"], last["deadline"] = None, 0.0
                dog.stop()
            elif ch in KEYS:
                fresh = ch != last["key"] or now > last["deadline"]
                last["key"] = ch
                last["deadline"] = now + (FIRST_PRESS_HOLD if fresh else REPEAT_HOLD)

    loop.add_reader(fd, on_stdin)
    print("\nW/S fwd/back  A/D turn  Q/E strafe  SPACE stop  ESC/Ctrl+C quit\n")
    was_moving = False
    try:
        while not quit_evt.is_set():
            now = time.monotonic()
            key = last["key"] if now < last["deadline"] else None
            state = conn.pc.connectionState if conn.pc else "closed"
            if state in ("failed", "closed", "disconnected"):
                raise ConnectionError(f"connection {state}")
            if key:
                x, y, z = KEYS[key]
                dog.move(x, y, z)
                status(f"SENDING  key={key.upper()}  x={x:+.2f} y={y:+.2f} yaw={z:+.2f}")
                was_moving = True
            else:
                if was_moving:
                    dog.stop()
                    was_moving = False
                status("STOPPED  (no key held)")
            await asyncio.sleep(1 / SEND_HZ)
    except Exception as e:
        print(f"\nERROR: {e}")
    finally:
        loop.remove_reader(fd)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        dog.stop()
        print("\nStop sent. Disconnecting...")
        try:
            await asyncio.wait_for(conn.disconnect(), 5)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dog-ip", default="192.168.12.1")
    ap.add_argument("--aes-key", default=os.environ.get("UNITREE_AES_128_KEY", ""),
                    help="32-hex per-device key, needed on Go2 firmware >= 1.1.15")
    args = ap.parse_args()
    if not sys.stdin.isatty():
        sys.exit("Run this in an interactive terminal.")
    try:
        asyncio.run(run(args.dog_ip, args.aes_key))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
