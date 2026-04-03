# qBc_Servos

WebSocket-based servo controller for the qB Companion robot. Drives Feetech ST3215 servos over a serial bus, accepting JSON commands for position control with configurable movement profiles. Includes an interactive TUI calibration tool.

![Python](https://img.shields.io/badge/Python-3.13-blue) ![Platform](https://img.shields.io/badge/Platform-ARM%20%2F%20Raspberry%20Pi-lightgrey) ![Servos](https://img.shields.io/badge/Servos-Feetech%20ST3215-orange)

## Quick Start

```bash
# Clone and setup
git clone <repo-url>
cd qBc_Servos
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# Run
python servo_controller.py
```

The server starts on `ws://0.0.0.0:8765`, initialises all joints to their calibrated zero positions, and waits for commands.

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  servo_controller.py                                         │
│  WebSocket server + command dispatch                         │
│                                                              │
│  ┌───────────────┐   ┌──────────────────┐                   │
│  │ ServoHardware │   │  Request Handlers │                   │
│  │ (thread-safe) │◄──┤                   │                   │
│  │  serial bus   │   │ joint_move_request│                   │
│  └───────┬───────┘   │ joint_animate_req │                   │
│          │            │ get_joint_status  │                   │
│          ▼            └────────┬─────────┘                   │
│  ┌───────────────┐             │                             │
│  │  scservo_sdk  │    ┌───────┴──────┐                       │
│  │  (ST3215 bus) │    │ JOINTS dict  │◄── servo_calibration  │
│  └───────────────┘    │ (from JSON)  │        .json          │
│                       └──────────────┘                       │
├──────────────────────────────────────────────────────────────┤
│  calibrate.py                                                │
│  Textual TUI: mouse sliders, set-zero, save/load config     │
└──────────────────────────────────────────────────────────────┘
```

## Project Structure

```
qBc_Servos/
├── servo_controller.py      # WebSocket server + servo control
├── calibrate.py             # Interactive TUI calibration tool
├── servo_calibration.json   # Calibrated joint definitions
├── requirements.txt         # pyserial, websockets, textual, keyboard
└── scservo_sdk/             # Feetech SCServo SDK (vendored)
    ├── __init__.py
    ├── port_handler.py      # Serial port wrapper
    ├── protocol_packet_handler.py  # Base packet protocol
    ├── sms_sts.py           # SMS/STS servo protocol (used)
    ├── scscl.py             # SCSCL protocol (alternate)
    ├── scservo_def.py       # Protocol constants
    ├── group_sync_read.py   # Synchronized bus reads
    └── group_sync_write.py  # Synchronized bus writes
```

## Joint Map

| Key | ID | Name | Range | Group |
|---|---|---|---|---|
| `neck` | 200 | Neck | ±90° | Head |
| `left_ear` | 201 | Left Ear | ±90° | Head |
| `right_ear` | 202 | Right Ear | ±90° | Head |
| `left_front_leg` | 101 | Left Front Leg | ±45° | Legs |
| `right_front_leg` | 102 | Right Front Leg | ±45° | Legs |
| `left_back_leg` | 103 | Left Back Leg | ±45° | Legs |
| `right_back_leg` | 104 | Right Back Leg | ±45° | Legs |

## Hardware

- **Servos:** Feetech ST3215 (12-bit encoder, 4096 steps/revolution)
- **Bus:** Serial TTL @ 1Mbaud on `/dev/ttyAMA0` (Raspberry Pi UART)
- **Encoding:** `STEPS_PER_DEGREE = 4096 / 360 ≈ 11.378`
- **Protocol:** SMS/STS (little-endian), via vendored `scservo_sdk`

## WebSocket API

Default port: `8765`. All messages are JSON.

### 1. Move Joint (speed-based)

Move a joint at a specified speed in °/s:

```json
{
  "type": "joint_move_request",
  "joint_name": "neck",
  "target_position": 45.0,
  "speed": 30.0,
  "movement_type": "linear"
}
```

**Response:**
```json
{
  "type": "joint_move_response",
  "joint_name": "neck",
  "status": "ok",
  "target_degrees": 45.0,
  "target_raw": 2425,
  "servo_speed": 341,
  "acc": 10,
  "movement_type": "linear"
}
```

### 2. Animate Joint (duration-based)

Move a joint to arrive in a specified duration — speed is computed automatically:

```json
{
  "type": "joint_animate_request",
  "joint_name": "neck",
  "target_position": 45.0,
  "duration": 2.0,
  "movement_type": "linear"
}
```

**Response:**
```json
{
  "type": "joint_animate_response",
  "joint_name": "neck",
  "status": "ok",
  "target_degrees": 45.0,
  "target_raw": 2425,
  "computed_speed": 256,
  "acc": 10,
  "movement_type": "linear",
  "distance_degrees": 45.0,
  "duration": 2.0
}
```

Returns `"status": "already_at_target"` if the joint is already at the requested position.

### 3. Get Joint Status

```json
{"type": "get_joint_status", "joint_name": "neck"}
```

**Response:**
```json
{
  "type": "joint_status",
  "joint_name": "neck",
  "status": "moving",
  "current_angle": 12.5,
  "current_speed": 5.3,
  "current_temperature": 35,
  "current_position": 2190
}
```

`status` is `"moving"`, `"stationary"`, or `"unknown"`.

### Error Response

```json
{"type": "error", "message": "Unknown joint: foo"}
```

## Movement Profiles

The `movement_type` parameter controls the servo's acceleration curve. Higher ACC values produce gentler ramps; the speed multiplier compensates so the servo still arrives on time.

| Type | ACC | Speed Mult | Behaviour |
|---|---|---|---|
| `linear` | 10 | 1.0× | Near-constant speed, minimal ramp |
| `quadratic` | 80 | 1.5× | Moderate acceleration curve |
| `exponential` | 180 | 2.0× | Gentle ramp, highest peak speed |

## Calibration

The calibration tool determines the true zero position of each servo — the raw encoder value where the physical joint is at 0°. This is necessary because robot parts are not mounted exactly aligned with the servo's midpoint (2048).

```bash
python calibrate.py              # requires hardware
python calibrate.py --simulate   # no hardware needed
```

### Controls

| Input | Action |
|---|---|
| Mouse click/drag | Move servo via slider |
| Left/Right arrows | Fine adjustment (±1 step) |
| Shift+Left/Right | Coarse adjustment (±10 steps) |
| `s` | Save calibration |
| `l` | Load calibration |
| `r` | Reset all to center (2048) |
| `q` | Quit |

### Workflow

1. Launch `calibrate.py`
2. For each servo, drag the slider until the robot part is physically at its intended neutral position
3. Click **"Set as Zero"** to record that raw value as the zero reference
4. Click **"Save Configuration"** — writes `servo_calibration.json`

The saved file is loaded by `servo_controller.py` on startup. All degree-based commands are then relative to the calibrated zero.

### Calibration File Format

```json
{
  "description": "qB Companion servo calibration data",
  "serial": {"port": "/dev/ttyAMA0", "baudrate": 1000000},
  "servos": {
    "neck": {
      "id": 200,
      "name": "Neck",
      "zero_position": 1913,
      "min_degrees": -90,
      "max_degrees": 90,
      "min_raw": 889,
      "max_raw": 2937
    }
  }
}
```

`min_raw` and `max_raw` are computed from `zero_position ± range × STEPS_PER_DEGREE`.

## CLI Arguments

### servo_controller.py

| Argument | Default | Description |
|---|---|---|
| `--port` | `/dev/ttyAMA0` | Serial port |
| `--baudrate` | `1000000` | Baud rate |
| `--ws-port` | `8765` | WebSocket listen port |
| `--config` | `servo_calibration.json` | Calibration file path |
| `--simulate` | — | Simulation mode (no hardware) |

### calibrate.py

| Argument | Default | Description |
|---|---|---|
| `--port` | `/dev/ttyAMA0` | Serial port |
| `--baudrate` | `1000000` | Baud rate |
| `--simulate` | — | Simulation mode (no hardware) |

## Integration with qBc_Animation

The animation system connects to this server as a WebSocket client and sends `joint_animate_request` commands driven by `.ani` file keyframes. Joint parameters in animations:

```json
{
  "time": 0.4,
  "transition": "quadratic",
  "left_eye": {"offset_y": 5.0},
  "right_eye": {"offset_y": 5.0},
  "joints": {
    "neck": {"position": 15.0, "movement_type": "linear"},
    "left_ear": {"position": 30.0}
  }
}
```

Start the servo server first, then the animation system:

```bash
# Terminal 1
cd qBc_Servos && python servo_controller.py

# Terminal 2
cd qBc_Animation && python main.py
```

## Default Torque Limits

| Joint | Torque |
|---|---|
| Neck | 150 |
| Ears | 100 |
| Legs | 200 |

Applied on startup before moving to zero positions.
