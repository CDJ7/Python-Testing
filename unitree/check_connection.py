#!/usr/bin/env python3
"""Step 1: connection check for a Unitree dog over its WiFi hotspot.

Standard library only. Sends NO movement commands.

Usage:
    python3 check_connection.py [--dog-ip 192.168.12.1] [--model go1|go2|auto]
"""
import argparse
import importlib.util
import json
import re
import socket
import subprocess
import sys
import urllib.request

PREFIX = "192.168.12."
results = []


def report(name, ok, detail="", hint=""):
    results.append((name, ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" - {detail}" if detail else ""))
    if not ok and hint:
        print(f"       hint: {hint}")


def tcp_open(ip, port, timeout=2.0):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def is_wsl():
    try:
        return "microsoft" in open("/proc/version").read().lower()
    except OSError:
        return False


def check_wifi_ip():
    try:
        out = subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True,
                             text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError) as e:
        report("Laptop has a 192.168.12.x address", False, str(e),
               "Install iproute2 (`sudo apt install iproute2`).")
        return
    addrs = re.findall(r"^\d+:\s+(\S+)\s+inet\s+([\d.]+)", out, re.M)
    match = [(i, a) for i, a in addrs if a.startswith(PREFIX)]
    if match:
        report("Laptop has a 192.168.12.x address", True,
               ", ".join(f"{a} on {i}" for i, a in match))
    elif is_wsl():
        report("Laptop has a 192.168.12.x address", True,
               "running in WSL2 (NAT via Windows); Windows must be on the dog's WiFi - "
               "the ping check below confirms the route")
    else:
        report("Laptop has a 192.168.12.x address", False,
               "addresses seen: " + (", ".join(f"{a} ({i})" for i, a in addrs) or "none"),
               "Join the dog's WiFi hotspot (e.g. 'GO2-xxxx' / 'UnitreeGo1-xxxx') and "
               "make sure the laptop isn't also holding a different network.")


def check_ping(ip):
    try:
        r = subprocess.run(["ping", "-c", "3", "-W", "2", ip],
                           capture_output=True, text=True, timeout=15)
        ok = r.returncode == 0
        loss = re.search(r"(\d+)% packet loss", r.stdout)
        report(f"Dog responds to ping ({ip})", ok, loss.group(0) if loss else "",
               "Check the WiFi link, that the dog is fully booted (wait ~1 min), "
               "and that the IP is right (`ip route | grep default`).")
    except (OSError, subprocess.SubprocessError) as e:
        report(f"Dog responds to ping ({ip})", False, str(e),
               "Install ping (`sudo apt install iputils-ping`).")


def probe_ports(ip):
    ports = {1883: "MQTT (Go1)", 9991: "WebRTC signalling, new firmware (Go2)",
             8081: "WebRTC signalling, old firmware (Go2)",
             8080: "web/other", 22: "ssh"}
    print("\nPort scan (used to identify the model):")
    found = {}
    for p, label in ports.items():
        found[p] = tcp_open(ip, p)
        print(f"   {p:>5}  {'open  ' if found[p] else 'closed'}  {label}")
    print()
    return found


def guess_model(found):
    if found.get(1883):
        return "go1"
    if found.get(9991) or found.get(8081):
        return "go2"
    return None


def check_go1(ip, found):
    report(f"MQTT broker reachable on {ip}:1883", found.get(1883),
           hint="Go1 only. Wait for boot to finish; if this is a Go2, use --model go2.")
    if not found.get(1883):
        return
    if importlib.util.find_spec("paho") is None:
        report("paho-mqtt installed", False, hint="pip install paho-mqtt")
        return
    import paho.mqtt.client as mqtt
    state = {"connected": False}

    def on_connect(*args):
        state["connected"] = True

    try:
        try:  # paho-mqtt 2.x
            c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except AttributeError:  # 1.x
            c = mqtt.Client()
        c.on_connect = on_connect
        c.connect(ip, 1883, keepalive=5)
        c.loop_start()
        for _ in range(30):
            if state["connected"]:
                break
            socket.setdefaulttimeout(None)
            import time
            time.sleep(0.1)
        c.loop_stop()
        c.disconnect()
    except Exception as e:
        report("MQTT handshake", False, str(e))
        return
    report("MQTT handshake (connect/disconnect only)", state["connected"],
           hint="Broker port is open but handshake failed; retry after a reboot of the dog.")


def check_go2(ip, found):
    report(f"WebRTC signalling port reachable on {ip}", found.get(9991) or found.get(8081),
           hint="Go2 only. Close the Unitree phone app (it holds the single WebRTC slot), "
                "and make sure the dog is in AP mode with you on its hotspot.")
    ok = importlib.util.find_spec("go2_webrtc_driver") is not None
    report("go2_webrtc_connect installed", ok,
           hint="pip install go2-webrtc-connect   (apt: portaudio19-dev libopus-dev ffmpeg may be needed)")
    if found.get(9991):
        try:  # new-firmware signalling endpoint; any HTTP answer shows it is alive
            req = urllib.request.Request(f"http://{ip}:9991/con_notify")
            with urllib.request.urlopen(req, timeout=3) as r:
                body = r.read(200).decode(errors="replace")
            report("Go2 signalling HTTP answers on :9991/con_notify", True, body[:80])
        except Exception as e:
            report("Go2 signalling HTTP answers on :9991/con_notify", False, str(e),
                   "Port open but no HTTP answer; close other WebRTC clients and retry.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dog-ip", default="192.168.12.1")
    ap.add_argument("--model", choices=["go1", "go2", "auto"], default="auto")
    a = ap.parse_args()

    print("=== Unitree connection check (no movement is sent) ===\n")
    check_wifi_ip()
    check_ping(a.dog_ip)
    found = probe_ports(a.dog_ip)

    model = a.model if a.model != "auto" else guess_model(found)
    if model is None:
        report("Model identified", False, "no 1883 / 9991 / 8081 ports open",
               "Dog may still be booting, or you're not on its hotspot. Re-run in a minute.")
    else:
        print(f"Treating robot as: {model.upper()} (Go2 covers Air/Pro/EDU over WiFi)\n")
        (check_go1 if model == "go1" else check_go2)(a.dog_ip, found)

    failed = [n for n, ok in results if not ok]
    print("\n=== Summary ===")
    print("ALL CHECKS PASSED" if not failed else "FAILED: " + "; ".join(failed))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
