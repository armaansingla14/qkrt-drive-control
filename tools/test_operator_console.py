import json
import socket
import threading

import pytest

import operator_console as oc


def test_encode_drive_message():
    msg = json.loads(oc.encode("drive", 5, 0.123456, -1.0))
    assert msg == {"type": "drive", "seq": 5, "v": 0.1235, "w": -1.0}


def test_encode_estop_has_no_velocity():
    assert json.loads(oc.encode("estop", 1)) == {"type": "estop", "seq": 1}


@pytest.mark.parametrize("payload", [b"not json", b"[]", b'{"type":"telemetry","seq":1}', b'{"type":"other"}'])
def test_decode_rejects_bad_telemetry(payload):
    assert oc.decode_telemetry(payload) is None


def test_decode_valid_telemetry():
    t = oc.decode_telemetry(b'{"type":"telemetry","seq":3,"x":1.0,"y":2,"theta":0.1,"state":"active"}')
    assert t["y"] == 2 and "active" in oc.format_telemetry(t)


def test_parse_script():
    steps = oc.parse_script("drive 0.5 0.1 1.0; estop 0.2; clear 0.1; idle 0.5")
    assert [s[0] for s in steps] == ["drive", "estop", "clear", "idle"]
    with pytest.raises(ValueError):
        oc.parse_script("jump 1")


def test_link_round_trip_against_fake_robot():
    robot = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot.bind(("127.0.0.1", 0))
    robot.settimeout(2.0)
    port = robot.getsockname()[1]

    def serve():
        data, addr = robot.recvfrom(2048)
        msg = json.loads(data)
        reply = {"type": "telemetry", "seq": msg["seq"], "x": msg["v"], "y": 0, "theta": 0}
        robot.sendto(json.dumps(reply).encode(), addr)

    th = threading.Thread(target=serve)
    th.start()
    link = oc.OperatorLink("127.0.0.1", port)
    link.send("drive", 0.7, 0.0)
    th.join(timeout=2.0)
    for _ in range(100):
        link.poll()
        if link.last_telemetry:
            break
        threading.Event().wait(0.01)
    robot.close()
    assert link.last_telemetry is not None and link.last_telemetry["x"] == pytest.approx(0.7)
