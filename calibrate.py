#!/usr/bin/env python3
"""qB Companion - Servo Calibration Tool

Interactive TUI application for calibrating the zero position of each servo joint.
Uses mouse-enabled sliders to move servos and saves calibration to JSON.

Usage:
    python calibrate.py                         # Normal mode (requires hardware)
    python calibrate.py --simulate              # Simulation mode (no hardware needed)
    python calibrate.py --port /dev/ttyUSB0     # Custom serial port

Controls:
    Mouse:  Click/drag sliders to move servos
    Keys:   Left/Right arrows for fine adjustment (±1 step)
            Shift+Left/Right for coarse adjustment (±10 steps)
    s       Save configuration
    l       Load configuration
    r       Reset all servos to center (2048)
    q       Quit
"""

import sys
import os
import json
import argparse
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.message import Message
from textual.widget import Widget
from textual.widgets import Header, Footer, Static, Button, Label
from textual.containers import Horizontal, VerticalScroll
from textual.reactive import reactive
from textual import on
from rich.text import Text

try:
    from scservo_sdk import *
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False

# ─── Constants ───────────────────────────────────────────────────────────────

STEPS_PER_DEGREE = 4096.0 / 360.0  # ≈ 11.378
DEFAULT_CENTER = 2048

CALIB_SPEED = 500
CALIB_ACC = 50

DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUDRATE = 1000000
CONFIG_FILE = "servo_calibration.json"

SERVO_DEFS = [
    {"key": "neck",            "id": 200, "name": "Neck",             "group": "Head",  "min_deg": -90, "max_deg": 90,  "torque": 150},
    {"key": "left_ear",        "id": 201, "name": "Left Ear",         "group": "Head",  "min_deg": -90, "max_deg": 90,  "torque": 100},
    {"key": "right_ear",       "id": 202, "name": "Right Ear",        "group": "Head",  "min_deg": -90, "max_deg": 90,  "torque": 100},
    {"key": "left_front_leg",  "id": 101, "name": "Left Front Leg",   "group": "Legs",  "min_deg": -45, "max_deg": 45,  "torque": 200},
    {"key": "right_front_leg", "id": 102, "name": "Right Front Leg",  "group": "Legs",  "min_deg": -45, "max_deg": 45,  "torque": 200},
    {"key": "left_back_leg",   "id": 103, "name": "Left Back Leg",    "group": "Legs",  "min_deg": -45, "max_deg": 45,  "torque": 200},
    {"key": "right_back_leg",  "id": 104, "name": "Right Back Leg",   "group": "Legs",  "min_deg": -45, "max_deg": 45,  "torque": 200},
]

MARGIN_DEG = 30


def calc_slider_range(min_deg, max_deg):
    total_half = int(((max_deg - min_deg) / 2 + MARGIN_DEG) * STEPS_PER_DEGREE)
    return max(0, DEFAULT_CENTER - total_half), min(4096, DEFAULT_CENTER + total_half)


def raw_to_degrees(raw_pos, zero_pos):
    return (raw_pos - zero_pos) / STEPS_PER_DEGREE


def degrees_to_raw(degrees, zero_pos):
    return int(zero_pos + degrees * STEPS_PER_DEGREE)


# ─── Custom Slider Widget ───────────────────────────────────────────────────

class SliderBar(Widget):
    """A horizontal slider with click, drag and keyboard support."""

    can_focus = True

    value: reactive[int] = reactive(DEFAULT_CENTER, init=False)

    class Changed(Message):
        def __init__(self, slider: "SliderBar", value: int) -> None:
            super().__init__()
            self.slider = slider
            self.value = value

    def __init__(
        self,
        min_value: int,
        max_value: int,
        value: int | None = None,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.min_value = min_value
        self.max_value = max_value
        self._dragging = False
        if value is not None:
            self.value = max(min_value, min(max_value, value))

    def render(self) -> Text:
        width = self.size.width
        if width < 4:
            return Text("...")
        track_w = width - 2  # space for [ ]
        rng = self.max_value - self.min_value
        if rng <= 0:
            pos = 0
        else:
            pos = int((self.value - self.min_value) / rng * (track_w - 1))
            pos = max(0, min(track_w - 1, pos))

        bar = Text()
        bar.append("╶", style="dim")
        bar.append("━" * pos, style="bright_cyan" if self.has_focus else "cyan")
        bar.append("●", style="bold bright_white" if self.has_focus else "bold white")
        remaining = track_w - pos - 1
        bar.append("━" * remaining, style="dim")
        bar.append("╴", style="dim")
        return bar

    def _value_from_x(self, x: int) -> int:
        track_w = self.size.width - 2
        if track_w <= 1:
            return self.value
        col = max(0, min(track_w - 1, x - 1))
        rng = self.max_value - self.min_value
        return int(self.min_value + (col / (track_w - 1)) * rng)

    def _set_value(self, new_val: int) -> None:
        new_val = max(self.min_value, min(self.max_value, new_val))
        if new_val != self.value:
            self.value = new_val

    def watch_value(self, new_value: int) -> None:
        self.post_message(self.Changed(self, new_value))
        self.refresh()

    # ── mouse ────────────────────────────────────────────────────────────

    def on_mouse_down(self, event) -> None:
        self.focus()
        self._dragging = True
        self.capture_mouse()
        self._set_value(self._value_from_x(event.x))

    def on_mouse_move(self, event) -> None:
        if self._dragging:
            self._set_value(self._value_from_x(event.x))

    def on_mouse_up(self, event) -> None:
        if self._dragging:
            self._dragging = False
            self.release_mouse()

    # ── keyboard ─────────────────────────────────────────────────────────

    def on_key(self, event) -> None:
        step = 10 if event.shift else 1
        if event.key in ("left", "shift+left"):
            self._set_value(self.value - step)
            event.stop()
        elif event.key in ("right", "shift+right"):
            self._set_value(self.value + step)
            event.stop()


# ─── Servo Control Widget ───────────────────────────────────────────────────

class ServoControl(Static):
    """Composite widget for controlling and calibrating one servo."""

    def __init__(self, servo_def: dict, **kwargs):
        super().__init__(**kwargs)
        self.servo_def = servo_def
        self.key = servo_def["key"]
        self.servo_id = servo_def["id"]
        self.servo_name = servo_def["name"]
        self.min_deg = servo_def["min_deg"]
        self.max_deg = servo_def["max_deg"]
        self.slider_min, self.slider_max = calc_slider_range(self.min_deg, self.max_deg)
        self.zero_pos = DEFAULT_CENTER
        self.current_pos = DEFAULT_CENTER

    def compose(self) -> ComposeResult:
        with Horizontal(classes="servo-header-row"):
            yield Label(
                f"{self.servo_name}  (ID: {self.servo_id})",
                classes="servo-label",
            )
            yield Label(
                f"Raw: {DEFAULT_CENTER}   Angle:  0.0°",
                id=f"info_{self.key}",
                classes="servo-info",
            )
        yield SliderBar(
            min_value=self.slider_min,
            max_value=self.slider_max,
            value=DEFAULT_CENTER,
            id=f"slider_{self.key}",
        )
        with Horizontal(classes="servo-btn-row"):
            yield Button("Set as Zero", id=f"zero_{self.key}", variant="primary")
            yield Button("Go to Zero", id=f"goto_{self.key}")
            yield Label(
                f"  Zero position: {self.zero_pos}",
                id=f"zlabel_{self.key}",
                classes="zero-info",
            )

    def update_info(self, raw_pos: int):
        self.current_pos = raw_pos
        angle = raw_to_degrees(raw_pos, self.zero_pos)
        try:
            self.query_one(f"#info_{self.key}", Label).update(
                f"Raw: {raw_pos}   Angle: {angle:+.1f}°"
            )
        except Exception:
            pass

    def set_zero(self):
        self.zero_pos = self.current_pos
        try:
            self.query_one(f"#zlabel_{self.key}", Label).update(
                f"  Zero position: {self.zero_pos}"
            )
        except Exception:
            pass
        self.update_info(self.current_pos)

    def go_to_zero(self):
        try:
            self.query_one(f"#slider_{self.key}", SliderBar).value = self.zero_pos
        except Exception:
            pass

    def set_slider(self, value: int):
        value = max(self.slider_min, min(self.slider_max, value))
        try:
            self.query_one(f"#slider_{self.key}", SliderBar).value = value
        except Exception:
            pass
        self.update_info(value)


# ─── Application ─────────────────────────────────────────────────────────────

class CalibrationApp(App):
    """TUI servo calibration application with mouse-enabled sliders."""

    TITLE = "qB Companion - Servo Calibration"

    CSS = """
    Screen {
        overflow-y: auto;
    }

    #title-bar {
        width: 100%;
        height: 3;
        content-align: center middle;
        text-style: bold;
        background: $primary;
        color: $text;
    }

    .group-header {
        margin: 1 2 0 2;
        text-style: bold italic;
        color: $text-muted;
    }

    ServoControl {
        height: auto;
        margin: 0 2 1 2;
        padding: 1 2;
        border: solid $primary;
    }

    .servo-header-row {
        height: 1;
    }

    .servo-label {
        width: 1fr;
        text-style: bold;
    }

    .servo-info {
        width: auto;
        min-width: 32;
        text-align: right;
    }

    SliderBar {
        height: 1;
        margin: 1 0;
    }

    SliderBar:focus {
        background: $surface;
    }

    .servo-btn-row {
        height: 3;
        margin-top: 1;
    }

    .servo-btn-row Button {
        margin-right: 2;
    }

    .zero-info {
        color: $text-muted;
        margin-left: 1;
        content-align: left middle;
    }

    #action-bar {
        height: auto;
        margin: 1 2 2 2;
        align: center middle;
    }

    #action-bar Button {
        margin: 0 2;
    }

    #status-bar {
        dock: bottom;
        height: 1;
        background: $surface;
        color: $text;
        padding: 0 2;
    }
    """

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("s", "save", "Save"),
        Binding("l", "load", "Load"),
        Binding("r", "reset", "Reset All"),
    ]

    def __init__(self, port: str, baudrate: int, simulate: bool):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.simulate = simulate
        self.packet_handler = None
        self.port_handler_obj = None
        self.servo_controls: dict[str, ServoControl] = {}

    # ── compose ──────────────────────────────────────────────────────────

    def compose(self) -> ComposeResult:
        yield Header()
        with VerticalScroll():
            yield Static(
                "qB Companion - Servo Calibration Tool\n"
                "Click/drag sliders to move joints. Press [Set as Zero] to calibrate.",
                id="title-bar",
            )

            current_group = None
            for sdef in SERVO_DEFS:
                if sdef["group"] != current_group:
                    current_group = sdef["group"]
                    yield Label(f"-- {current_group} --", classes="group-header")
                ctrl = ServoControl(sdef, id=f"ctrl_{sdef['key']}")
                self.servo_controls[sdef["key"]] = ctrl
                yield ctrl

            with Horizontal(id="action-bar"):
                yield Button("Save Configuration", id="btn_save", variant="success")
                yield Button("Load Configuration", id="btn_load", variant="primary")
                yield Button("Reset All to Center", id="btn_reset", variant="warning")

        yield Static("Initializing...", id="status-bar")
        yield Footer()

    # ── lifecycle ────────────────────────────────────────────────────────

    def on_mount(self) -> None:
        if self.simulate:
            self._status("SIMULATION MODE - no hardware connected")
            self._load_config_silent()
            return

        if not SDK_AVAILABLE:
            self._status("ERROR: scservo_sdk not available - run with --simulate")
            return

        try:
            self.port_handler_obj = PortHandler(self.port)
            self.packet_handler = sms_sts(self.port_handler_obj)

            if not self.port_handler_obj.openPort():
                self._status(f"ERROR: cannot open {self.port}")
                return
            if not self.port_handler_obj.setBaudRate(self.baudrate):
                self._status(f"ERROR: cannot set baudrate {self.baudrate}")
                return

            for sdef in SERVO_DEFS:
                self.packet_handler.WriteTorqueEx(sdef["id"], sdef["torque"])

            self._load_config_silent()

            for sdef in SERVO_DEFS:
                try:
                    pos, comm, err = self.packet_handler.ReadPos(sdef["id"])
                    if comm == COMM_SUCCESS:
                        self.servo_controls[sdef["key"]].set_slider(pos)
                except Exception:
                    pass

            self._status(f"Connected to {self.port} @ {self.baudrate}")

        except Exception as e:
            self._status(f"ERROR: {e}")

    # ── events ───────────────────────────────────────────────────────────

    @on(SliderBar.Changed)
    def on_slider_changed(self, event: SliderBar.Changed) -> None:
        sid = event.slider.id
        if not sid or not sid.startswith("slider_"):
            return
        key = sid.removeprefix("slider_")
        raw = int(event.value)
        if key in self.servo_controls:
            self.servo_controls[key].update_info(raw)
            self._move_servo(self.servo_controls[key].servo_id, raw)

    @on(Button.Pressed)
    def on_button_pressed(self, event: Button.Pressed) -> None:
        bid = event.button.id
        if not bid:
            return

        if bid.startswith("zero_"):
            key = bid.removeprefix("zero_")
            if key in self.servo_controls:
                ctrl = self.servo_controls[key]
                ctrl.set_zero()
                self._status(
                    f"Zero set for {ctrl.servo_name} -> raw {ctrl.zero_pos}"
                )

        elif bid.startswith("goto_"):
            key = bid.removeprefix("goto_")
            if key in self.servo_controls:
                ctrl = self.servo_controls[key]
                ctrl.go_to_zero()
                self._move_servo(ctrl.servo_id, ctrl.zero_pos)

        elif bid == "btn_save":
            self.action_save()
        elif bid == "btn_load":
            self.action_load()
        elif bid == "btn_reset":
            self.action_reset()

    # ── servo I/O ────────────────────────────────────────────────────────

    def _move_servo(self, servo_id: int, position: int) -> None:
        if self.simulate or self.packet_handler is None:
            return
        try:
            self.packet_handler.WritePosEx(servo_id, position, CALIB_SPEED, CALIB_ACC)
        except Exception as e:
            self._status(f"Servo comm error: {e}")

    # ── config persistence ───────────────────────────────────────────────

    def _config_path(self) -> Path:
        return Path(os.path.dirname(os.path.abspath(__file__))) / CONFIG_FILE

    def action_save(self) -> None:
        config = {
            "description": "qB Companion servo calibration data",
            "serial": {"port": self.port, "baudrate": self.baudrate},
            "servos": {},
        }
        for key, ctrl in self.servo_controls.items():
            config["servos"][key] = {
                "id": ctrl.servo_id,
                "name": ctrl.servo_name,
                "zero_position": ctrl.zero_pos,
                "min_degrees": ctrl.min_deg,
                "max_degrees": ctrl.max_deg,
                "min_raw": degrees_to_raw(ctrl.min_deg, ctrl.zero_pos),
                "max_raw": degrees_to_raw(ctrl.max_deg, ctrl.zero_pos),
            }

        path = self._config_path()
        with open(path, "w") as f:
            json.dump(config, f, indent=2)
        self._status(f"Saved to {path}")

    def action_load(self) -> None:
        path = self._config_path()
        if not path.exists():
            self._status(f"No config found at {path}")
            return
        try:
            with open(path) as f:
                config = json.load(f)
            self._apply_config(config, move_servos=True)
            self._status(f"Loaded from {path}")
        except (json.JSONDecodeError, KeyError) as e:
            self._status(f"Config error: {e}")

    def _load_config_silent(self) -> None:
        path = self._config_path()
        if not path.exists():
            return
        try:
            with open(path) as f:
                config = json.load(f)
            self._apply_config(config, move_servos=False)
        except Exception:
            pass

    def _apply_config(self, config: dict, move_servos: bool) -> None:
        for key, data in config.get("servos", {}).items():
            if key not in self.servo_controls:
                continue
            ctrl = self.servo_controls[key]
            ctrl.zero_pos = data["zero_position"]
            try:
                ctrl.query_one(f"#zlabel_{key}", Label).update(
                    f"  Zero position: {ctrl.zero_pos}"
                )
            except Exception:
                pass

            if move_servos:
                ctrl.set_slider(ctrl.zero_pos)
                self._move_servo(ctrl.servo_id, ctrl.zero_pos)
            else:
                ctrl.update_info(ctrl.current_pos)

    def action_reset(self) -> None:
        for key, ctrl in self.servo_controls.items():
            ctrl.set_slider(DEFAULT_CENTER)
            self._move_servo(ctrl.servo_id, DEFAULT_CENTER)
        self._status("All servos reset to center (2048)")

    # ── quit & cleanup ───────────────────────────────────────────────────

    def action_quit(self) -> None:
        if self.port_handler_obj and not self.simulate:
            try:
                self.port_handler_obj.closePort()
            except Exception:
                pass
        self.exit()

    # ── helpers ──────────────────────────────────────────────────────────

    def _status(self, msg: str) -> None:
        try:
            self.query_one("#status-bar", Static).update(msg)
        except Exception:
            pass


# ─── Entry Point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="qB Companion - Interactive Servo Calibration Tool"
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial port (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"Baudrate (default: {DEFAULT_BAUDRATE})",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Run in simulation mode without hardware",
    )
    args = parser.parse_args()

    app = CalibrationApp(
        port=args.port, baudrate=args.baudrate, simulate=args.simulate
    )
    app.run()


if __name__ == "__main__":
    main()
