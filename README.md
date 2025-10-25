# Omni Link Husky Simulation

This project provides a lightweight Python application for driving a fleet of simulated Clearpath Husky robots in PyBullet while exposing REST endpoints over Flask. It is useful for experimenting with differential drive control, prototyping coordinated motion behaviors, and integrating with other systems that can send HTTP commands. Everything runs in a single Python process, so it is easy to understand, extend, and debug without needing a ROS stack.

## Features

- **Realtime physics** powered by PyBullet running at 240 Hz.
- **Five-robot formation** spawned in a line (`husky_0` … `husky_4`) for fleet-style experiments.
- **REST API control** for commanding linear and angular velocity with optional duration.
- **Convenience helpers** for common motions such as forward, backward, and turning.
- **Live pose telemetry** exposed through the `GET /robots/<robot_id>/pose` endpoint.
- **Simple reset & stop endpoints** for quickly recovering from tests.

## Requirements

- Python 3.9+
- [PyBullet](https://pybullet.org)
- [Flask](https://flask.palletsprojects.com/)

Install the required packages with pip:

```bash
pip install pybullet flask
```

> **Note:** PyBullet opens a GUI window by default. On headless systems you may need to use a virtual display such as Xvfb.

## Running the Simulator

1. (Optional) Create and activate a virtual environment.

   ```bash
   python -m venv .venv
   source .venv/bin/activate
   ```

2. Navigate to the omni-link-husky-fleet folder and Launch the simulation and REST server:

   ```bash
   python husky_fleet.py
   ```

3. A PyBullet window will open showing the five-Husky formation and an obstacle. The Flask server listens on `http://127.0.0.1:5000`. Update the `app.run()` call in `husky_fleet.py` if you need to expose the API on a different interface. Use the sliders in the **Params** tab to move the camera and change the robot's viewpoint while you experiment.

   **Camera shortcuts** (right-hand keys):

   | Action | Keys |
   | ------ | ---- |
  | Move forward/back | `↑` / `↓` (arrow keys) |
  | Strafe left/right | `←` / `→` (arrow keys) |
  | Raise/lower camera target | `Keypad 9` / `Keypad 3` |
  | Zoom in/out | Mouse wheel |
  | Rotate yaw left/right | `Keypad 7` / `Keypad 1` |
  | Pitch up/down | `Keypad /` / `Keypad *` |
  | Reset camera offset | `Home` or `Keypad 5` |

4. On startup the module's `__main__` block lists example endpoints for every robot ID. Use the printed URLs, `curl`, `httpie`, Postman, or any HTTP-capable client to send commands.

5. Send HTTP requests to control the robot. You can use `curl`, `httpie`, Postman, or any HTTP-capable client.

For headless environments, start an X virtual framebuffer (e.g., `xvfb-run python husky_fleet.py`) or modify the fleet initialisation code to use `p.DIRECT` instead of `p.GUI`.

## Fleet layout & robot IDs

The simulator spawns five Huskies at one-metre intervals along the Y axis, centred on the origin. Robot IDs follow the pattern `husky_<index>`:

| Robot ID  | Initial pose (x, y, yaw) | Notes |
|-----------|--------------------------|-------|
| `husky_0` | `(0.0, -2.0, 0 rad)`      | Primary robot; camera controls enabled. |
| `husky_1` | `(0.0, -1.0, 0 rad)`      | Offset −1 m on the Y axis. |
| `husky_2` | `(0.0, 0.0, 0 rad)`       | Centred at the origin. |
| `husky_3` | `(0.0, 1.0, 0 rad)`       | Offset +1 m on the Y axis. |
| `husky_4` | `(0.0, 2.0, 0 rad)`       | Offset +2 m on the Y axis. |

Use `GET /robots` to query the active fleet; IDs in the response match the table above unless you modify the formation logic in `husky_fleet.py`.

## REST API

Endpoints that operate on a specific robot require the `<robot_id>` placeholder to be replaced with one of the IDs reported by `/robots`.

| Method & Endpoint | Description | Example Payload |
| ----------------- | ----------- | --------------- |
| `GET /health` | Returns `{ "ok": true }` for quick liveness checks. | _None_ |
| `GET /robots` | Lists all robots with their latest pose and command state. | _None_ |
| `GET /robots/<robot_id>/pose` | Returns the latest pose and queued command metadata for the selected robot. | _None_ |
| `POST /robots/<robot_id>/drive` | Command linear (`vx`) and angular (`wz`) velocity with an optional duration. Positive `vx` drives forward; positive `wz` rotates counter-clockwise. | `{ "vx": 0.6, "wz": 0.0, "duration": 2.0 }` |
| `POST /robots/<robot_id>/stop` | Immediately zeroes all velocity commands. | _None_ |
| `POST /robots/<robot_id>/reset` | Resets the robot pose/velocity and clears the current command. | _None_ |
| `POST /robots/<robot_id>/forward` | Convenience wrapper for driving forward (`speed` in m/s). | `{ "speed": 0.5, "duration": 1.0 }` |
| `POST /robots/<robot_id>/backward` | Convenience wrapper for reversing. | `{ "speed": 0.4, "duration": 1.0 }` |
| `POST /robots/<robot_id>/turn_left` | Convenience wrapper that rotates counter-clockwise (`rate` in rad/s). | `{ "rate": 1.2, "duration": 0.8 }` |
| `POST /robots/<robot_id>/turn_right` | Convenience wrapper that rotates clockwise (`rate` in rad/s). | `{ "rate": 1.2, "duration": 0.8 }` |
| `POST /fleet/reset` | Queues a reset command for every robot in the fleet. | _None_ |

Example `curl` request targeting `husky_2`:

```bash
curl -X POST http://127.0.0.1:5000/robots/husky_2/drive \
  -H "Content-Type: application/json" \
  -d '{"vx": 0.6, "wz": 0.0, "duration": 2.0}'
```

Each motion endpoint responds with HTTP 202 to confirm that the command was accepted and queued for execution.

## Coordinating concurrent commands

- Commands are queued asynchronously. Use the `command` object returned by `GET /robots/<robot_id>/pose` to inspect `duration_remaining`, `has_timeout`, and `pending_reset` before issuing follow-up commands.
- Provide a `duration` whenever possible. When a command expires the fleet automatically ramps velocities back to zero, reducing the chance that long-running commands overlap.
- If you must interrupt a robot, send `POST /robots/<robot_id>/stop` to clear the active command before queueing a new manoeuvre.
- Multiple robots can be controlled simultaneously by targeting different IDs; the fleet scheduler handles the per-robot locks to prevent conflicting updates.
- Use `POST /fleet/reset` to synchronise the entire formation before starting new multi-robot trials.

## Configuration

Key simulation constants are defined at the top of `husky_fleet.py`, including wheel geometry, acceleration limits, damping factors, and obstacle properties. Adjust them to tune vehicle behavior or test different dynamics.

## Project Structure

```
.
├── README.md          # Project overview and usage
├── husky_fleet.py     # Fleet manager, simulation, and REST server script
└── robot_link/        # Optional helper clients and adapters for Omni Link demos
```

### Helper scripts (robot_link/)

The `robot_link` directory contains optional bridges that rely on `robot_link/robot_api.py` for HTTP access. The helpers now default to the primary robot (`husky_0`) and understand the multi-robot REST structure.

- Set `HUSKY_API_URL` to target a different host/port (defaults to `http://127.0.0.1:5000`).
- Set `HUSKY_ROBOT_ID` to control a specific robot without changing code. All helper scripts (`link_mqtt.py`, `link_remote.py`, etc.) honour this environment variable via the shared API module.
- Call `robot_api.list_robots()` to discover IDs programmatically before issuing commands.

Update any bespoke integrations to include the robot ID in their request paths if they bypass `robot_api.py`.

To get started with omnilink bridge: 

Configure and Start the MQTT Broker (Mosquitto)

The OmniLink Agent UI requires the WebSockets protocol on port 9001. Standard Mosquitto defaults to TCP on port 1883, which causes connection failures.

    Stop the Default Service: Ensure no background instance is blocking the port.
    Bash

sudo systemctl stop mosquitto.service

Enable WebSockets: Edit the main Mosquitto configuration file.
Bash

sudo nano /etc/mosquitto/mosquitto.conf

Add/Verify Configuration: Ensure the following lines are present to allow the required protocol and port, and to prevent the rc=5 (Not Authorised) connection error.
Code snippet

allow_anonymous true # CRITICAL: Allows connection from the local bridge client
listener 9001
protocol websockets # CRITICAL: Required to match the OmniLink UI setting (ws://...)

Start the Broker Service:
Bash

    sudo systemctl start mosquitto.service

    (Terminal 2): The broker is now listening correctly.

3. Launch the OmniLink Bridge Script

The Python script receives MQTT messages and translates them into Flask API calls.

    Set Environment Variables: These tell the bridge script which robot API to talk to.
    Bash

export HUSKY_API_URL=http://127.0.0.1:5000
export HUSKY_ROBOT_ID=husky_0 # Default target robot

Navigate to Bridge Directory:
Bash

cd robot_link/

Run the Bridge:
Bash

    python link_mqtt.py

    (Terminal 3): A successful connection will show: [OmniLinkMQTT] Connected localhost:9001 (transport=websockets). This terminal now remains open and idle, waiting for commands.

Phase 2: OmniLink Agent Configuration

The final step is to configure the web UI (Agent) to publish commands to your running local environment.

1. Configure Connection Settings

In the OmniLink UI, navigate to the Connection Settings and verify the following fields to match your broker setup:
Setting	Value	CRITICAL NOTE
BROKER/WEBSOCKET URL	ws://localhost:9001	Must use the ws:// protocol and port 9001.
COMMAND TOPIC	olink/commands	This must match the topic the bridge script is subscribing to.

2. Define Command Templates (Crucial Syntax Fix)

The AI Agent must generate commands that exactly match the simplified placeholders expected by the Python bridge script. Avoid using full units like meters_per_second.
Action	Recommended Template to Enter in OmniLink UI
Drive Command	drive_[robot_id]_[direction]_[speed]_[duration]
Turn Command	turn_[robot_id]_[direction]_[rate]_[duration]
Stop Command	stop_[robot_id]
Reset Fleet	reset_fleet

3. Test the End-to-End Control Loop

    In the OmniLink UI (voice or text), issue a simplified command that matches the template:

        "Husky zero, move forward at zero point five, for three."

    Verification: Observe the command flow:

        UI Status: Last Command shows the simple string (e.g., drive_husky_0_forward_0.5_3).

        Bridge Terminal (T3): Logs the incoming command string and the successful translation (no "did not match" error).

        Simulation (T1/GUI): The husky_0 robot moves forward in the PyBullet window.

## Troubleshooting

- **No GUI appears:** Ensure you are running in an environment with an available display or configure PyBullet for headless rendering (see the headless note above).
- **Robot does not move:** Check that commands are within the clamped limits (`MAX_VX`, `MAX_WZ`) and that the `/drive` endpoint is receiving float values. The JSON body must contain numbers, not strings.
- **Camera controls are awkward:** Use the on-screen sliders in the PyBullet UI to adjust yaw and pitch, or change the default values at the top of `husky_fleet.py`.
- **Need to restart:** Use the `/reset` endpoint or restart the script to clear any unexpected state.

