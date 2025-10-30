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

## 🔗 OmniLink Bridge Setup (MQTT → Simulator Control)

This section explains how to connect the **OmniLink Agent UI** (web UI) with the **Husky Fleet Simulator** by using **MQTT**. The OmniLink UI communicates over **WebSockets**, so we must configure Mosquitto to support `protocol websockets` on port **9001**.

---

### 1) Configure & Start the Mosquitto Broker

The default Mosquitto broker listens on `1883` (TCP only), which **will not work** with the OmniLink UI.  
We must enable **WebSockets** on port **9001**.

#### Stop any running Mosquitto instance:
```bash
sudo systemctl stop mosquitto.service
```
Edit the Mosquitto configuration:
```bash
sudo nano /etc/mosquitto/mosquitto.conf
```
Add (or verify) these lines:
```bash
allow_anonymous true      # Required for local UI connections (avoid rc=5 auth errors)
listener 9001
protocol websockets       # Enables ws:// communication required by the OmniLink UI
```

Note: Using allow_anonymous true is acceptable for local development.
For production use, configure authentication.

Restart the broker:

```bash
sudo systemctl start mosquitto.service
```

✅ Verification:

Run in another terminal:
```bash
sudo netstat -tulpn | grep 9001
```
You should see Mosquitto listening on port 9001 (websockets).

2) Launch the OmniLink Bridge

This script receives messages from the OmniLink UI via MQTT and converts them into REST API calls to control the robots.

```bash
export HUSKY_API_URL=http://127.0.0.1:5000     # Address of the simulator REST API
```
Run the bridge:
```bash
cd robot_link/
python link_mqtt.py
```
Expected output:
```bash
[OmniLinkMQTT] Connected to localhost:9001 (transport=websockets)
```
Leave this terminal running.
It listens for control messages.

3) Configure the OmniLink Agent UI

Open the OmniLink Web UI → Connection Settings

Setting	Value	Important Note
Broker / WebSocket URL	ws://localhost:9001	Must use ws:// and port 9001
Command Topic	olink/commands	Must match the topic used in the bridge script

| Action                | Template Format                                   |
| --------------------- | ------------------------------------------------- |
| Drive (linear motion) | `drive_<robot_id>_<direction>_<speed>_<duration>` |
| Turn (rotation)       | `turn_<robot_id>_<direction>_<rate>_<duration>`   |
| Stop robot            | `stop_<robot_id>`                                 |
| Reset entire fleet    | `reset_fleet`                                     |

5) End-to-End Test

In OmniLink UI (voice or text), try:

"Husky zero, move forward at zero point five for three seconds."

You should observe:

UI displays: drive_husky_0_forward_0.5_3

Bridge terminal logs: command matched and translated

Simulator window: husky_0 moves forward smoothly

## Troubleshooting

- **No GUI appears:** Ensure you are running in an environment with an available display or configure PyBullet for headless rendering (see the headless note above).
- **Robot does not move:** Check that commands are within the clamped limits (`MAX_VX`, `MAX_WZ`) and that the `/drive` endpoint is receiving float values. The JSON body must contain numbers, not strings.
- **Camera controls are awkward:** Use the on-screen sliders in the PyBullet UI to adjust yaw and pitch, or change the default values at the top of `husky_fleet.py`.
- **Need to restart:** Use the `/reset` endpoint or restart the script to clear any unexpected state.

