#!/usr/bin/env python3
"""Terminal operator console for the QKRT rover (JSON over UDP).

Interactive mode (curses):
    python3 tools/operator_console.py --host 127.0.0.1 --port 47800
    keys: w/s = forward/back, a/d = turn, space = stop, e = e-stop, c = clear e-stop, q = quit

Scripted mode (no terminal needed), e.g. for a smoke test against teleop_server:
    python3 tools/operator_console.py --script "drive 0.8 0.0 2.0; drive 0.5 0.6 2.0; estop 0.5; clear 0.2"
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass, field

SEND_HZ = 20.0


def encode(msg_type: str, seq: int, v: float = 0.0, w: float = 0.0) -> bytes:
    msg: dict = {"type": msg_type, "seq": seq}
    if msg_type == "drive":
        msg["v"] = round(float(v), 4)
        msg["w"] = round(float(w), 4)
    return json.dumps(msg, separators=(",", ":")).encode()


def decode_telemetry(data: bytes) -> dict | None:
    try:
        msg = json.loads(data.decode())
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(msg, dict) or msg.get("type") != "telemetry":
        return None
    for key in ("seq", "x", "y", "theta"):
        if not isinstance(msg.get(key), (int, float)):
            return None
    return msg


@dataclass
class OperatorLink:
    host: str
    port: int
    seq: int = 0
    last_telemetry: dict | None = None
    received: int = 0
    sock: socket.socket = field(init=False)

    def __post_init__(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def send(self, msg_type: str, v: float = 0.0, w: float = 0.0) -> None:
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self.sock.sendto(encode(msg_type, self.seq, v, w), (self.host, self.port))

    def poll(self) -> None:
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                return
            tel = decode_telemetry(data)
            if tel is not None:
                self.last_telemetry = tel
                self.received += 1


def format_telemetry(t: dict | None) -> str:
    if t is None:
        return "no telemetry yet"
    return (f"t={t.get('t', 0):7.2f}s  x={t['x']:7.2f} y={t['y']:7.2f} th={t['theta']:+.2f}  "
            f"v={t.get('v', 0):+.2f} w={t.get('w', 0):+.2f}  state={t.get('state', '?'):8s} "
            f"slip={'Y' if t.get('slip') else 'n'}  loop={t.get('loop_us', 0):.0f}us")


def parse_script(script: str) -> list[tuple[str, float, float, float]]:
    steps = []
    for part in filter(None, (p.strip() for p in script.split(";"))):
        tok = part.split()
        if tok[0] == "drive" and len(tok) == 4:
            steps.append(("drive", float(tok[1]), float(tok[2]), float(tok[3])))
        elif tok[0] in ("estop", "clear", "idle") and len(tok) == 2:
            steps.append((tok[0], 0.0, 0.0, float(tok[1])))
        else:
            raise ValueError(f"bad script step: {part!r}")
    return steps


def run_script(link: OperatorLink, script: str) -> int:
    for kind, v, w, duration in parse_script(script):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if kind == "drive":
                link.send("drive", v, w)
            elif kind == "estop":
                link.send("estop")
            elif kind == "clear":
                link.send("clear_estop")
            # "idle": send nothing so the robot-side watchdog trips
            time.sleep(1.0 / SEND_HZ)
            link.poll()
        print(f"[{kind:5s}] {format_telemetry(link.last_telemetry)}")
    link.poll()
    print(f"telemetry packets received: {link.received}")
    return 0 if link.received > 0 else 1


def run_interactive(link: OperatorLink) -> int:
    import curses

    def loop(stdscr) -> None:
        curses.curs_set(0)
        stdscr.nodelay(True)
        v = w = 0.0
        while True:
            key = stdscr.getch()
            if key == ord("q"):
                break
            if key == ord("w"):
                v = min(v + 0.1, 2.0)
            elif key == ord("s"):
                v = max(v - 0.1, -2.0)
            elif key == ord("a"):
                w = min(w + 0.2, 2.5)
            elif key == ord("d"):
                w = max(w - 0.2, -2.5)
            elif key == ord(" "):
                v = w = 0.0
            if key == ord("e"):
                link.send("estop")
                v = w = 0.0
            elif key == ord("c"):
                link.send("clear_estop")
            else:
                link.send("drive", v, w)
            link.poll()
            stdscr.erase()
            stdscr.addstr(0, 0, f"QKRT operator console -> {link.host}:{link.port}")
            stdscr.addstr(1, 0, "w/s speed  a/d turn  space stop  e estop  c clear  q quit")
            stdscr.addstr(3, 0, f"command: v={v:+.2f} m/s  w={w:+.2f} rad/s  seq={link.seq}")
            stdscr.addstr(4, 0, format_telemetry(link.last_telemetry))
            stdscr.refresh()
            time.sleep(1.0 / SEND_HZ)

    curses.wrapper(loop)
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=47800)
    ap.add_argument("--script", help="semicolon-separated steps: 'drive V W SEC', 'estop SEC', 'clear SEC', 'idle SEC'")
    args = ap.parse_args(argv)
    link = OperatorLink(args.host, args.port)
    return run_script(link, args.script) if args.script else run_interactive(link)


if __name__ == "__main__":
    sys.exit(main())
