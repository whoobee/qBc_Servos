#!/usr/bin/env python3
"""qB Companion - MQTT Servo Controller

Subscribes to robot/joints/cmd for JSON commands to move servos.
Loads calibration from servo_calibration.json and initialises all joints
to their calibrated zero positions on startup.

Usage:
    python servo_controller.py                              # defaults
    python servo_controller.py --mqtt-broker 192.168.1.10   # remote broker
    python servo_controller.py --simulate                   # no hardware
    python servo_controller.py --config path/to/cal.json

MQTT topics:
    Subscribe: robot/joints/cmd
    Publish:   robot/joints/status

Message types (publish JSON to robot/joints/cmd):

1) joint_move_request
   {"type": "joint_move_request",
    "joint_name": "neck",
    "target_position": 45.0,
    "speed": 30.0,
    "movement_type": "linear"}

2) joint_animate_request
   {"type": "joint_animate_request",
    "joint_name": "neck",
    "target_position": 45.0,
    "duration": 2.0,
    "movement_type": "linear"}

3) get_joint_status
   {"type": "get_joint_status",
    "joint_name": "neck"}
"""

import sys
import os
import json
import math
import argparse
import logging
import signal
import threading
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import paho.mqtt.client as mqtt

try:
    from scservo_sdk import *
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False

# ─── Logging ─────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("servo_ctrl")

# ─── Constants ───────────────────────────────────────────────────────────────

STEPS_PER_DEGREE = 4096.0 / 360.0  # ≈ 11.378 steps per degree
DEFAULT_CENTER = 2048
DEFAULT_BAUDRATE = 1000000
DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_MQTT_BROKER = "localhost"
DEFAULT_MQTT_PORT = 1883
DEFAULT_CONFIG = "servo_calibration.json"

# MQTT topics
TOPIC_JOINTS_CMD = "robot/joints/cmd"
TOPIC_JOINTS_STATUS = "robot/joints/status"
TOPIC_HEARTBEAT = "robot/system/heartbeat/servos"

# Movement type → (ACC value, speed multiplier)
# ACC on ST3215: lower = faster ramp ⇒ more "constant-speed" profile.
# Higher ACC = gentler ramp ⇒ more time spent accelerating/decelerating.
# Speed multiplier compensates so the servo still arrives roughly on time
# when a significant fraction of the move is spent ramping.
MOVEMENT_PROFILES = {
    "linear":      {"acc": 10,  "speed_mult": 1.0},
    "quadratic":   {"acc": 80,  "speed_mult": 1.5},
    "exponential": {"acc": 180, "speed_mult": 2.0},
}

# ─── Joint Registry ─────────────────────────────────────────────────────────

# Populated from the calibration file.
# Each entry: { id, name, zero_position, min_deg, max_deg, min_raw, max_raw, torque }
JOINTS: dict[str, dict] = {}

# Torque defaults per servo (applied on startup)
# ST3215 range: 0–1000  (1000 ≈ 100 % of stall torque)
DEFAULT_TORQUES = {
    200: 300,   # Neck
    201: 500,   # Left Ear
    202: 500,   # Right Ear
    101: 400,   # Left Front Leg
    102: 400,   # Right Front Leg
    103: 400,   # Left Back Leg
    104: 400,   # Right Back Leg
}


def load_calibration(config_path: str) -> None:
    """Load servo_calibration.json into the JOINTS dict."""
    path = Path(config_path)
    if not path.exists():
        log.error("Calibration file not found: %s", path)
        sys.exit(1)

    with open(path) as f:
        config = json.load(f)

    for key, data in config.get("servos", {}).items():
        JOINTS[key] = {
            "id":            data["id"],
            "name":          data["name"],
            "zero_position": data["zero_position"],
            "min_degrees":   data["min_degrees"],
            "max_degrees":   data["max_degrees"],
            "min_raw":       data["min_raw"],
            "max_raw":       data["max_raw"],
            "torque":        DEFAULT_TORQUES.get(data["id"], 200),
        }

    log.info("Loaded %d joints from %s", len(JOINTS), path)
    for key, j in JOINTS.items():
        log.info(
            "  %-20s ID:%d  zero:%d  range:[%d..%d]  (%.0f° .. %.0f°)",
            j["name"], j["id"], j["zero_position"],
            j["min_raw"], j["max_raw"],
            j["min_degrees"], j["max_degrees"],
        )


# ─── Conversion helpers ─────────────────────────────────────────────────────

def degrees_to_raw(degrees: float, zero_pos: int) -> int:
    return int(zero_pos + degrees * STEPS_PER_DEGREE)


def raw_to_degrees(raw: int, zero_pos: int) -> float:
    return (raw - zero_pos) / STEPS_PER_DEGREE


def degrees_per_sec_to_raw(deg_per_sec: float) -> int:
    """Convert degrees/sec to servo speed units (steps/sec)."""
    return max(1, int(abs(deg_per_sec) * STEPS_PER_DEGREE))


def raw_speed_to_degrees(raw_speed: int) -> float:
    """Convert servo speed (signed, steps/sec) to degrees/sec."""
    return raw_speed / STEPS_PER_DEGREE


# ─── Hardware Wrapper ────────────────────────────────────────────────────────

class ServoHardware:
    """Thread-safe wrapper around the scservo_sdk serial bus.

    All public methods acquire a lock before touching the bus so that
    concurrent calls don't corrupt packets.
    """

    def __init__(self, port: str, baudrate: int, simulate: bool):
        self.simulate = simulate
        self._lock = threading.Lock()
        self.port_handler = None
        self.packet_handler = None

        if simulate:
            log.info("Running in SIMULATION mode – no hardware")
            return

        if not SDK_AVAILABLE:
            log.error("scservo_sdk not importable – cannot run in hardware mode")
            sys.exit(1)

        self.port_handler = PortHandler(port)
        self.packet_handler = sms_sts(self.port_handler)

        if not self.port_handler.openPort():
            log.error("Cannot open serial port %s", port)
            sys.exit(1)
        if not self.port_handler.setBaudRate(baudrate):
            log.error("Cannot set baudrate %d", baudrate)
            sys.exit(1)

        log.info("Serial port %s open @ %d baud", port, baudrate)

    # ── public API ───────────────────────────────────────────────────────

    def enable_torque(self, servo_id: int, torque: int) -> None:
        if self.simulate:
            return
        with self._lock:
            # SDK WriteTorqueEx is buggy (uses loword/hiword instead of
            # lobyte/hibyte), so write the 2-byte torque limit directly.
            ph = self.packet_handler
            txpacket = [ph.scs_lobyte(torque), ph.scs_hibyte(torque)]
            ph.writeTxRx(servo_id, 48, len(txpacket), txpacket)  # 48 = SMS_STS_TORQUE_LIMIT

    def move(self, servo_id: int, position: int, speed: int, acc: int) -> None:
        """Send a position/speed/acc command to one servo."""
        if self.simulate:
            log.debug(
                "SIM move ID:%d pos:%d spd:%d acc:%d", servo_id, position, speed, acc
            )
            return
        with self._lock:
            comm, err = self.packet_handler.WritePosEx(
                servo_id, position, speed, acc
            )
            if comm != COMM_SUCCESS:
                log.warning(
                    "WritePosEx ID:%d comm error: %s",
                    servo_id,
                    self.packet_handler.getTxRxResult(comm),
                )

    def move_timed(self, servo_id: int, position: int, time_ms: int, acc: int) -> None:
        """Send a position command with Goal Time instead of Goal Speed.

        The servo computes the required speed internally to arrive at
        *position* in *time_ms* milliseconds.  Goal Speed is set to 0.
        """
        if self.simulate:
            log.debug(
                "SIM move_timed ID:%d pos:%d time:%dms acc:%d",
                servo_id, position, time_ms, acc,
            )
            return
        with self._lock:
            ph = self.packet_handler
            txpacket = [
                acc,
                ph.scs_lobyte(position), ph.scs_hibyte(position),
                ph.scs_lobyte(time_ms),  ph.scs_hibyte(time_ms),
                0, 0,  # Goal Speed = 0  →  servo uses Goal Time
            ]
            comm, err = ph.writeTxRx(
                servo_id, 41, len(txpacket), txpacket   # 41 = SMS_STS_ACC
            )
            if comm != COMM_SUCCESS:
                log.warning(
                    "move_timed ID:%d comm error: %s",
                    servo_id, ph.getTxRxResult(comm),
                )

    def read_position(self, servo_id: int) -> int | None:
        if self.simulate:
            return None
        with self._lock:
            pos, comm, err = self.packet_handler.ReadPos(servo_id)
            return pos if comm == COMM_SUCCESS else None

    def read_speed(self, servo_id: int) -> int | None:
        if self.simulate:
            return None
        with self._lock:
            speed, comm, err = self.packet_handler.ReadSpeed(servo_id)
            return speed if comm == COMM_SUCCESS else None

    def read_moving(self, servo_id: int) -> bool | None:
        if self.simulate:
            return None
        with self._lock:
            moving, comm, err = self.packet_handler.ReadMoving(servo_id)
            return bool(moving) if comm == COMM_SUCCESS else None

    def read_temperature(self, servo_id: int) -> int | None:
        if self.simulate:
            return None
        with self._lock:
            temp, comm, err = self.packet_handler.read1ByteTxRx(
                servo_id, 63  # SMS_STS_PRESENT_TEMPERATURE
            )
            return temp if comm == COMM_SUCCESS else None

    def close(self) -> None:
        if self.port_handler and not self.simulate:
            self.port_handler.closePort()
            log.info("Serial port closed")


# ─── Request Handlers ────────────────────────────────────────────────────────

def _validate_joint(joint_name: str) -> dict | None:
    """Return the joint dict or None if not found."""
    return JOINTS.get(joint_name)


def _clamp_raw(position: int, joint: dict) -> int:
    return max(joint["min_raw"], min(joint["max_raw"], position))


def handle_joint_move(msg: dict, hw: ServoHardware) -> dict:
    """Process a joint_move_request.

    Required fields: joint_name, target_position (°), speed (°/s), movement_type
    """
    joint_name = msg.get("joint_name")
    joint = _validate_joint(joint_name)
    if joint is None:
        return {"type": "error", "message": f"Unknown joint: {joint_name}"}

    target_deg = msg.get("target_position", 0.0)
    speed_deg = msg.get("speed", 30.0)
    movement_type = msg.get("movement_type", "linear")

    if movement_type not in MOVEMENT_PROFILES:
        return {
            "type": "error",
            "message": f"Unknown movement_type: {movement_type}. "
                       f"Valid: {list(MOVEMENT_PROFILES.keys())}",
        }

    # Clamp target to joint limits
    target_deg = max(joint["min_degrees"], min(joint["max_degrees"], target_deg))
    target_raw = _clamp_raw(
        degrees_to_raw(target_deg, joint["zero_position"]), joint
    )

    profile = MOVEMENT_PROFILES[movement_type]
    servo_speed = degrees_per_sec_to_raw(speed_deg * profile["speed_mult"])
    acc = profile["acc"]

    hw.move(joint["id"], target_raw, servo_speed, acc)

    log.info(
        "MOVE %s → %.1f° (raw %d) spd=%d acc=%d [%s]",
        joint_name, target_deg, target_raw, servo_speed, acc, movement_type,
    )

    return {
        "type": "joint_move_response",
        "joint_name": joint_name,
        "status": "ok",
        "target_degrees": target_deg,
        "target_raw": target_raw,
        "servo_speed": servo_speed,
        "acc": acc,
        "movement_type": movement_type,
    }


def handle_joint_animate(msg: dict, hw: ServoHardware) -> dict:
    """Process a joint_animate_request.

    Computes the speed needed so the servo arrives at *target_position* in
    *duration* seconds, taking the movement_type profile into account.

    Required fields: joint_name, target_position (°), duration (s), movement_type
    """
    joint_name = msg.get("joint_name")
    joint = _validate_joint(joint_name)
    if joint is None:
        return {"type": "error", "message": f"Unknown joint: {joint_name}"}

    target_deg = msg.get("target_position", 0.0)
    duration = msg.get("duration", 1.0)
    movement_type = msg.get("movement_type", "linear")

    if duration <= 0:
        return {"type": "error", "message": "duration must be > 0"}
    if movement_type not in MOVEMENT_PROFILES:
        return {
            "type": "error",
            "message": f"Unknown movement_type: {movement_type}. "
                       f"Valid: {list(MOVEMENT_PROFILES.keys())}",
        }

    target_deg = max(joint["min_degrees"], min(joint["max_degrees"], target_deg))
    target_raw = _clamp_raw(
        degrees_to_raw(target_deg, joint["zero_position"]), joint
    )

    profile = MOVEMENT_PROFILES[movement_type]
    acc = profile["acc"]
    time_ms = max(1, int(duration * 1000))

    # Optional per-command torque override (0–1000). Applied before move,
    # reverts to the joint's default torque afterwards.
    torque = msg.get("torque")
    if torque is not None:
        torque = max(0, min(1000, int(torque)))
        hw.enable_torque(joint["id"], torque)

    hw.move_timed(joint["id"], target_raw, time_ms, acc)

    log.info(
        "ANIMATE %s → %.1f° in %.2fs (raw %d) time=%dms acc=%d torque=%s [%s]",
        joint_name, target_deg, duration, target_raw, time_ms, acc,
        torque if torque is not None else "default",
        movement_type,
    )

    resp = {
        "type": "joint_animate_response",
        "joint_name": joint_name,
        "status": "ok",
        "target_degrees": target_deg,
        "target_raw": target_raw,
        "time_ms": time_ms,
        "acc": acc,
        "movement_type": movement_type,
        "duration": duration,
    }
    if torque is not None:
        resp["torque"] = torque
    return resp


def handle_get_status(msg: dict, hw: ServoHardware) -> dict:
    """Process a get_joint_status request.

    Returns: status, current_angle, current_speed, current_temperature,
             current_position (raw).
    """
    joint_name = msg.get("joint_name")
    joint = _validate_joint(joint_name)
    if joint is None:
        return {"type": "error", "message": f"Unknown joint: {joint_name}"}

    raw_pos = hw.read_position(joint["id"])
    raw_speed = hw.read_speed(joint["id"])
    moving = hw.read_moving(joint["id"])
    temperature = hw.read_temperature(joint["id"])

    if raw_pos is not None:
        angle = round(raw_to_degrees(raw_pos, joint["zero_position"]), 2)
    else:
        angle = None

    if raw_speed is not None:
        speed_deg = round(raw_speed_to_degrees(raw_speed), 2)
    else:
        speed_deg = None

    if moving is not None:
        status = "moving" if moving else "stationary"
    else:
        status = "unknown"

    return {
        "type": "joint_status",
        "joint_name": joint_name,
        "status": status,
        "current_angle": angle,
        "current_speed": speed_deg,
        "current_temperature": temperature,
        "current_position": raw_pos,
    }


# ─── Dispatch ────────────────────────────────────────────────────────────────

HANDLERS = {
    "joint_move_request":    handle_joint_move,
    "joint_animate_request": handle_joint_animate,
    "get_joint_status":      handle_get_status,
}


# ─── MQTT Callbacks ─────────────────────────────────────────────────────────

def _on_connect(client, userdata, connect_flags, reason_code, properties):
    hw = userdata
    if reason_code.is_failure:
        log.error("MQTT connection failed: %s", reason_code)
        return
    log.info("Connected to MQTT broker")
    client.subscribe(TOPIC_JOINTS_CMD, qos=1)
    client.publish(
        TOPIC_JOINTS_STATUS,
        json.dumps({"status": "online", "joints": list(JOINTS.keys())}),
        qos=1, retain=True,
    )


def _on_disconnect(client, userdata, disconnect_flags, reason_code, properties):
    if reason_code.is_failure:
        log.warning("Disconnected from MQTT broker: %s", reason_code)


def _on_message(client, userdata, msg):
    hw = userdata
    try:
        data = json.loads(msg.payload)
    except (json.JSONDecodeError, UnicodeDecodeError):
        log.warning("Invalid JSON on %s", msg.topic)
        return

    msg_type = data.get("type")
    handler = HANDLERS.get(msg_type)
    if handler is None:
        log.warning("Unknown request type: %s", msg_type)
        return

    try:
        resp = handler(data, hw)
    except Exception:
        log.exception("Handler error for %s", msg_type)
        return

    # Publish response for status queries and errors
    if resp.get("type") in ("joint_status", "error"):
        client.publish(TOPIC_JOINTS_STATUS, json.dumps(resp), qos=0)


# ─── Joint Initialisation ───────────────────────────────────────────────────

def initialise_joints(hw: ServoHardware) -> None:
    """Enable torque and move every joint to its calibrated zero position."""
    log.info("Initialising all joints to zero position...")
    for key, joint in JOINTS.items():
        hw.enable_torque(joint["id"], joint["torque"])

    # Small delay to let torque enable settle
    time.sleep(0.1)

    for key, joint in JOINTS.items():
        zero = joint["zero_position"]
        # Move gently to zero
        speed = degrees_per_sec_to_raw(30)  # 30 °/s
        hw.move(joint["id"], zero, speed, 50)
        log.info("  %s (ID:%d) → zero = %d", joint["name"], joint["id"], zero)

    log.info("All joints sent to zero position")


# ─── Main ────────────────────────────────────────────────────────────────────

def main_run(args):
    hw = ServoHardware(args.port, args.baudrate, args.simulate)

    try:
        initialise_joints(hw)

        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id="qbc_servos",
            userdata=hw,
        )
        client.on_connect = _on_connect
        client.on_disconnect = _on_disconnect
        client.on_message = _on_message
        client.will_set(
            TOPIC_JOINTS_STATUS,
            json.dumps({"status": "offline"}),
            qos=1, retain=True,
        )

        client.connect(args.mqtt_broker, args.mqtt_port)
        client.loop_start()

        log.info(
            "Servo controller running — MQTT %s:%d — subscribed to %s",
            args.mqtt_broker, args.mqtt_port, TOPIC_JOINTS_CMD,
        )

        # Block until signal, publish heartbeat every second
        stop = threading.Event()
        signal.signal(signal.SIGINT, lambda *_: stop.set())
        signal.signal(signal.SIGTERM, lambda *_: stop.set())

        while not stop.is_set():
            client.publish(TOPIC_HEARTBEAT, b"1", qos=0)
            stop.wait(1.0)

        log.info("Shutting down...")
        client.publish(
            TOPIC_JOINTS_STATUS,
            json.dumps({"status": "offline"}),
            qos=1, retain=True,
        )
        client.loop_stop()
        client.disconnect()
    finally:
        hw.close()


def main():
    parser = argparse.ArgumentParser(
        description="qB Companion – MQTT Servo Controller"
    )
    parser.add_argument(
        "--port", default=DEFAULT_PORT,
        help=f"Serial port (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--baudrate", type=int, default=DEFAULT_BAUDRATE,
        help=f"Baudrate (default: {DEFAULT_BAUDRATE})",
    )
    parser.add_argument(
        "--mqtt-broker", default=DEFAULT_MQTT_BROKER,
        help=f"MQTT broker address (default: {DEFAULT_MQTT_BROKER})",
    )
    parser.add_argument(
        "--mqtt-port", type=int, default=DEFAULT_MQTT_PORT,
        help=f"MQTT broker port (default: {DEFAULT_MQTT_PORT})",
    )
    parser.add_argument(
        "--config", default=DEFAULT_CONFIG,
        help=f"Calibration JSON file (default: {DEFAULT_CONFIG})",
    )
    parser.add_argument(
        "--simulate", action="store_true",
        help="Simulation mode – no hardware required",
    )
    args = parser.parse_args()

    load_calibration(args.config)
    main_run(args)


if __name__ == "__main__":
    main()
