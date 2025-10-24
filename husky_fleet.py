#!/usr/bin/env python3
"""Husky fleet simulator and REST interface."""

from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set, Tuple

from flask import Flask, request

import pybullet as p
import pybullet_data


def reset_user_debug_parameter(parameter_id: int, value: float) -> None:
    """Best-effort wrapper for resetting a PyBullet debug slider."""

    if hasattr(p, "resetUserDebugParameter"):
        p.resetUserDebugParameter(parameter_id, value)

# ---------------- Simulation Tunables ----------------
HZ = 240  # physics rate
DT = 1.0 / HZ
WHEEL_RADIUS = 0.095  # Husky wheels ~9.5 cm
WHEEL_BASE = 0.55  # left-right distance
MAX_VX = 1.2  # m/s safety clamp
MAX_WZ = 2.0  # rad/s safety clamp
ACCEL_VX = 2.5  # m/s^2 accel ramp
ACCEL_WZ = 5.0  # rad/s^2 accel ramp
MOTOR_TORQUE = 24.0  # Nm per wheel
LINEAR_DAMP = 0.05  # natural decay
ANGULAR_DAMP = 0.08

BOX_SCALE = 4.0  # obstacle size
BOX_MASS = 25.0


PRIMARY_ROBOT_NAME = "husky_0"


# ---------------- Utility helpers ----------------
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def smooth_towards(curr: float, target: float, accel: float, dt: float) -> float:
    if target > curr:
        return min(target, curr + accel * dt)
    return max(target, curr - accel * dt)


def diff_to_wheels(vx: float, wz: float) -> Tuple[float, float]:
    v_l = vx - (wz * WHEEL_BASE / 2.0)
    v_r = vx + (wz * WHEEL_BASE / 2.0)
    wl = v_l / WHEEL_RADIUS  # rad/s
    wr = v_r / WHEEL_RADIUS
    return wl, wr


def get_pose(body_id: int) -> Tuple[float, float, float]:
    pos, orn = p.getBasePositionAndOrientation(body_id)
    eul = p.getEulerFromQuaternion(orn)
    return pos[0], pos[1], eul[2]


# ---------------- Husky robot model ----------------
class HuskyRobot:
    """Represents a single Husky within the PyBullet world."""

    def __init__(
        self,
        name: str,
        body_id: int,
        wheels_left: Tuple[int, int],
        wheels_right: Tuple[int, int],
        start_pos: Tuple[float, float, float],
        start_orn: Tuple[float, float, float, float],
    ) -> None:
        self.name = name
        self.body_id = body_id
        self.wheels_left = wheels_left
        self.wheels_right = wheels_right
        self.wheel_joint_indices = tuple(wheels_left + wheels_right)
        self._left_wheel_indices = set(wheels_left)
        self.start_pos = start_pos
        self.start_orn = start_orn

        self._command_lock = threading.Lock()
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0
        self._cmd_until: Optional[float] = None
        self._pending_reset = False

        self._vx = 0.0
        self._wz = 0.0
        self._last_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        self.camera_distance = 3.0
        self.camera_yaw = -90.0
        self.camera_pitch = -45.0
        self.camera_yaw_slider_id: Optional[int] = None
        self.camera_pitch_slider_id: Optional[int] = None

        self._initialise_wheel_motors()

    # ----- Command helpers -----
    def set_command(self, vx: float, wz: float, duration: Optional[float] = None) -> None:
        until = None
        if duration is not None:
            duration = max(0.0, float(duration))
            until = time.time() + duration
        with self._command_lock:
            self._cmd_vx = float(vx)
            self._cmd_wz = float(wz)
            self._cmd_until = until

    def stop(self) -> None:
        with self._command_lock:
            self._cmd_vx = 0.0
            self._cmd_wz = 0.0
            self._cmd_until = None

    def request_reset(self) -> None:
        with self._command_lock:
            self._pending_reset = True

    # ----- Telemetry -----
    def get_pose(self) -> Tuple[float, float, float]:
        return self._last_pose

    def get_command_state(self) -> Dict[str, object]:
        with self._command_lock:
            remaining: Optional[float]
            if self._cmd_until is None:
                remaining = None
            else:
                remaining = max(0.0, self._cmd_until - time.time())
            return {
                "vx": self._cmd_vx,
                "wz": self._cmd_wz,
                "duration_remaining": remaining,
                "has_timeout": self._cmd_until is not None,
                "pending_reset": self._pending_reset,
            }

    def describe(self) -> Dict[str, object]:
        x, y, yaw = self.get_pose()
        return {
            "id": self.name,
            "pose": {"x": x, "y": y, "yaw": yaw},
            "command": self.get_command_state(),
        }

    # ----- Debug helpers -----
    def configure_debug_elements(self, enable_camera_controls: bool = True) -> None:
        if not enable_camera_controls:
            return
        self.camera_yaw_slider_id = p.addUserDebugParameter(
            f"{self.name} Camera Yaw", -180.0, 180.0, self.camera_yaw
        )
        self.camera_pitch_slider_id = p.addUserDebugParameter(
            f"{self.name} Camera Pitch", -89.0, 45.0, self.camera_pitch
        )

    # ----- Simulation loop -----
    def pre_step(self, dt: float) -> None:
        pending_reset = False
        now = time.time()
        with self._command_lock:
            if self._pending_reset:
                pending_reset = True
                self._pending_reset = False
            desired_vx = self._cmd_vx
            desired_wz = self._cmd_wz
            if self._cmd_until is not None and now > self._cmd_until:
                desired_vx = 0.0
                desired_wz = 0.0
                self._cmd_vx = 0.0
                self._cmd_wz = 0.0
                self._cmd_until = None
        if pending_reset:
            self._perform_reset()
            desired_vx = 0.0
            desired_wz = 0.0

        desired_vx = clamp(desired_vx, -MAX_VX, MAX_VX)
        desired_wz = clamp(desired_wz, -MAX_WZ, MAX_WZ)

        self._vx = smooth_towards(self._vx, desired_vx, ACCEL_VX, dt)
        self._wz = smooth_towards(self._wz, desired_wz, ACCEL_WZ, dt)

        if abs(desired_vx) < 1e-4:
            self._vx *= 1.0 - LINEAR_DAMP
        if abs(desired_wz) < 1e-4:
            self._wz *= 1.0 - ANGULAR_DAMP

        wl, wr = diff_to_wheels(self._vx, self._wz)
        for joint in self.wheel_joint_indices:
            target_velocity = wl if joint in self._left_wheel_indices else wr
            p.setJointMotorControl2(
                self.body_id,
                joint,
                p.VELOCITY_CONTROL,
                targetVelocity=target_velocity,
                force=MOTOR_TORQUE,
            )

    def post_step(self) -> None:
        self._update_pose()

    def _initialise_wheel_motors(self) -> None:
        for joint in self.wheel_joint_indices:
            p.setJointMotorControl2(
                self.body_id,
                joint,
                p.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
            )

    def _perform_reset(self) -> None:
        p.resetBasePositionAndOrientation(self.body_id, self.start_pos, self.start_orn)
        p.resetBaseVelocity(self.body_id, [0, 0, 0], [0, 0, 0])
        self._vx = 0.0
        self._wz = 0.0
        self._last_pose = (self.start_pos[0], self.start_pos[1], 0.0)
        self.stop()

    def read_camera_orientation(self) -> Tuple[float, float]:
        if self.camera_yaw_slider_id is not None:
            self.camera_yaw = p.readUserDebugParameter(self.camera_yaw_slider_id)
        if self.camera_pitch_slider_id is not None:
            self.camera_pitch = p.readUserDebugParameter(self.camera_pitch_slider_id)
        return self.camera_yaw, self.camera_pitch

    def _update_pose(self) -> None:
        self._last_pose = get_pose(self.body_id)


# ---------------- Viewport controls ----------------
class ViewportKeyboardController:
    """Handles keyboard and mouse input for moving the PyBullet camera."""

    MOVE_SPEED = 3.0  # metres per second
    HEIGHT_SPEED = 1.5
    ZOOM_STEP = 0.8
    YAW_SPEED = 90.0  # degrees per second
    PITCH_SPEED = 60.0
    MIN_PITCH = -89.0
    MAX_PITCH = 45.0
    MIN_DISTANCE = 1.0
    MAX_DISTANCE = 60.0
    MIN_HEIGHT_OFFSET = -2.0
    MAX_HEIGHT_OFFSET = 3.0

    # Keypad controls (Num Lock on) and fallbacks for environments that
    # report special keys instead of ASCII digits.
    _KEY_FORWARD = {ord("8")}
    _KEY_BACK = {ord("2")}
    _KEY_LEFT = {ord("4")}
    _KEY_RIGHT = {ord("6")}
    _KEY_RAISE_TARGET = {ord("9")}
    _KEY_LOWER_TARGET = {ord("3")}
    _KEY_YAW_LEFT = {ord("7")}
    _KEY_YAW_RIGHT = {ord("1")}
    _KEY_PITCH_UP = {ord("/")}
    _KEY_PITCH_DOWN = {ord("*")}
    _KEY_RESET = {p.B3G_HOME, ord("5")}

    _MOUSE_WHEEL_UP = {3}
    _MOUSE_WHEEL_DOWN = {4}

    def __init__(self) -> None:
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._height_offset = 0.0

        # Extend keypad mappings if PyBullet exposes dedicated constants.
        key_aliases = (
            (("B3G_NUMPAD8", "B3G_KP8", "B3G_KP_8", "B3G_UP_ARROW"), self._KEY_FORWARD),
            (("B3G_NUMPAD2", "B3G_KP2", "B3G_KP_2", "B3G_DOWN_ARROW"), self._KEY_BACK),
            (("B3G_NUMPAD4", "B3G_KP4", "B3G_KP_4", "B3G_LEFT_ARROW"), self._KEY_LEFT),
            (("B3G_NUMPAD6", "B3G_KP6", "B3G_KP_6", "B3G_RIGHT_ARROW"), self._KEY_RIGHT),
            (("B3G_NUMPAD9", "B3G_KP9", "B3G_KP_9"), self._KEY_RAISE_TARGET),
            (("B3G_NUMPAD3", "B3G_KP3", "B3G_KP_3"), self._KEY_LOWER_TARGET),
            (("B3G_NUMPAD7", "B3G_KP7", "B3G_KP_7"), self._KEY_YAW_LEFT),
            (("B3G_NUMPAD1", "B3G_KP1", "B3G_KP_1"), self._KEY_YAW_RIGHT),
            (("B3G_NUMPAD_SLASH", "B3G_KP_SLASH"), self._KEY_PITCH_UP),
            (("B3G_NUMPAD_ASTERISK", "B3G_KP_ASTERISK", "B3G_KP_MULTIPLY"), self._KEY_PITCH_DOWN),
            (("B3G_NUMPAD5", "B3G_KP5", "B3G_KP_5"), self._KEY_RESET),
        )
        for names, target_set in key_aliases:
            for attr in names:
                key_code = getattr(p, attr, None)
                if key_code is not None:
                    target_set.add(key_code)

        mouse_wheel_aliases = (
            ("B3G_MOUSE_WHEEL_UP", self._MOUSE_WHEEL_UP),
            ("B3G_MOUSE_WHEEL_DOWN", self._MOUSE_WHEEL_DOWN),
        )
        for attr, target_set in mouse_wheel_aliases:
            key_code = getattr(p, attr, None)
            if key_code is not None:
                target_set.add(key_code)

    @staticmethod
    def _any_key_pressed(keys_down: Set[int], keycodes: Set[int]) -> bool:
        return any(code in keys_down for code in keycodes)

    def update(self, robot: HuskyRobot, dt: float) -> None:
        if dt <= 0.0:
            return
        events = p.getKeyboardEvents()
        mouse_events = p.getMouseEvents()
        self._apply_mouse_wheel_zoom(robot, mouse_events)

        if not events:
            return
        keys_down = {key for key, state in events.items() if state & p.KEY_IS_DOWN}
        if not keys_down:
            return

        yaw_rad = math.radians(robot.camera_yaw)
        forward = 0.0
        strafe = 0.0
        if self._any_key_pressed(keys_down, self._KEY_FORWARD):
            forward += 1.0
        if self._any_key_pressed(keys_down, self._KEY_BACK):
            forward -= 1.0
        if self._any_key_pressed(keys_down, self._KEY_LEFT):
            strafe -= 1.0
        if self._any_key_pressed(keys_down, self._KEY_RIGHT):
            strafe += 1.0

        if forward or strafe:
            # PyBullet's camera yaw rotates counter-clockwise around the +Z axis
            # with 0° pointing along the +X axis. Moving the camera "forward"
            # therefore aligns with the heading perpendicular to the strafe axis
            # rather than with the raw (cos, sin) pair we would normally use for a
            # robot pose. Swapping the basis vectors keeps forward/back tied to
            # the visual heading while left/right remains a lateral strafe.
            forward_vec = (-math.sin(yaw_rad), math.cos(yaw_rad))
            strafe_vec = (math.cos(yaw_rad), math.sin(yaw_rad))
            move_x = forward * forward_vec[0] + strafe * strafe_vec[0]
            move_y = forward * forward_vec[1] + strafe * strafe_vec[1]
            norm = math.hypot(move_x, move_y)
            if norm > 0.0:
                move_x /= norm
                move_y /= norm
            step = self.MOVE_SPEED * dt
            self._pan_x += move_x * step
            self._pan_y += move_y * step

        if self._any_key_pressed(keys_down, self._KEY_RAISE_TARGET):
            self._height_offset += self.HEIGHT_SPEED * dt
        if self._any_key_pressed(keys_down, self._KEY_LOWER_TARGET):
            self._height_offset -= self.HEIGHT_SPEED * dt
        self._height_offset = clamp(self._height_offset, self.MIN_HEIGHT_OFFSET, self.MAX_HEIGHT_OFFSET)

        yaw_changed = False
        if self._any_key_pressed(keys_down, self._KEY_YAW_LEFT):
            robot.camera_yaw = (robot.camera_yaw + self.YAW_SPEED * dt) % 360.0
            yaw_changed = True
        if self._any_key_pressed(keys_down, self._KEY_YAW_RIGHT):
            robot.camera_yaw = (robot.camera_yaw - self.YAW_SPEED * dt) % 360.0
            yaw_changed = True

        pitch_changed = False
        if self._any_key_pressed(keys_down, self._KEY_PITCH_UP):
            robot.camera_pitch = clamp(
                robot.camera_pitch + self.PITCH_SPEED * dt,
                self.MIN_PITCH,
                self.MAX_PITCH,
            )
            pitch_changed = True
        if self._any_key_pressed(keys_down, self._KEY_PITCH_DOWN):
            robot.camera_pitch = clamp(
                robot.camera_pitch - self.PITCH_SPEED * dt,
                self.MIN_PITCH,
                self.MAX_PITCH,
            )
            pitch_changed = True

        if yaw_changed and robot.camera_yaw_slider_id is not None:
            reset_user_debug_parameter(robot.camera_yaw_slider_id, robot.camera_yaw)
        if pitch_changed and robot.camera_pitch_slider_id is not None:
            reset_user_debug_parameter(robot.camera_pitch_slider_id, robot.camera_pitch)

        if self._any_key_pressed(keys_down, self._KEY_RESET):
            self._pan_x = 0.0
            self._pan_y = 0.0
            self._height_offset = 0.0

    def _apply_mouse_wheel_zoom(
        self, robot: HuskyRobot, mouse_events: Sequence[Tuple[int, float, float, int, int]]
    ) -> None:
        if not mouse_events:
            return

        distance_change = 0.0
        trigger_mask = getattr(p, "KEY_WAS_TRIGGERED", 0) | getattr(p, "KEY_IS_DOWN", 0)
        for event_type, _x, _y, button_index, button_state in mouse_events:
            if event_type != getattr(p, "MOUSE_BUTTON_EVENT", 2):
                continue
            if trigger_mask and not (button_state & trigger_mask):
                continue
            if button_index in self._MOUSE_WHEEL_UP:
                distance_change -= self.ZOOM_STEP
            elif button_index in self._MOUSE_WHEEL_DOWN:
                distance_change += self.ZOOM_STEP

        if distance_change:
            robot.camera_distance = clamp(
                robot.camera_distance + distance_change,
                self.MIN_DISTANCE,
                self.MAX_DISTANCE,
            )

    def target_position(self, base_x: float, base_y: float, base_z: float) -> Tuple[float, float, float]:
        return base_x + self._pan_x, base_y + self._pan_y, base_z + self._height_offset


# ---------------- Fleet manager ----------------
class HuskyFleet:
    """Owns the PyBullet connection and manages Husky robots."""

    def __init__(self) -> None:
        self.robots: Dict[str, HuskyRobot] = {}
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._ready = threading.Event()
        self._lock = threading.Lock()
        self._command_queue: queue.Queue[FleetCommand] = queue.Queue()
        self._viewport_controller = ViewportKeyboardController()

    def robot_ids(self) -> Tuple[str, ...]:
        with self._lock:
            return tuple(sorted(self.robots.keys()))

    def robot_exists(self, name: str) -> bool:
        with self._lock:
            return name in self.robots

    def describe_robot(self, name: str) -> Dict[str, object]:
        with self._lock:
            robot = self.robots.get(name)
        if robot is None:
            raise KeyError(name)
        return robot.describe()

    def describe_robots(self) -> List[Dict[str, object]]:
        with self._lock:
            robots = list(self.robots.values())
        return [robot.describe() for robot in robots]

    def queue_drive(
        self, name: str, vx: float, wz: float, duration: Optional[float]
    ) -> None:
        self._command_queue.put(
            FleetCommand(action="drive", robot=name, payload={"vx": vx, "wz": wz, "duration": duration})
        )

    def queue_stop(self, name: str) -> None:
        self._command_queue.put(FleetCommand(action="stop", robot=name, payload={}))

    def queue_reset(self, name: str) -> None:
        self._command_queue.put(FleetCommand(action="reset", robot=name, payload={}))

    def queue_fleet_reset(self) -> None:
        for robot_id in self.robot_ids():
            self.queue_reset(robot_id)

    def start(self) -> None:
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._running.set()
            self._thread = threading.Thread(target=self._physics_loop, daemon=True)
            self._thread.start()

    def stop(self, timeout: Optional[float] = None) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=timeout)
            self._thread = None
        self._ready.clear()
        with self._lock:
            self.robots.clear()

    def wait_until_ready(self, timeout: Optional[float] = None) -> bool:
        return self._ready.wait(timeout=timeout)

    def get_robot(self, name: str) -> HuskyRobot:
        if not self._ready.is_set():
            raise RuntimeError("Fleet is not initialised")
        return self.robots[name]

    def _physics_loop(self) -> None:
        connection_id = p.connect(p.GUI)
        try:
            self.start_simulation()

            while self._running.is_set():
                self._process_command_queue()
                robots = self._get_robots_in_order()
                for current_robot in robots:
                    current_robot.pre_step(DT)
                p.stepSimulation()
                for current_robot in robots:
                    current_robot.post_step()
                self._update_fleet_camera(robots)
                time.sleep(DT)
        finally:
            try:
                p.disconnect(connection_id)
            except Exception:
                pass

    def start_simulation(self) -> None:
        robots = self.setup_world()
        with self._lock:
            self.robots = robots
        self._ready.set()

    def setup_world(self) -> Dict[str, HuskyRobot]:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        p.resetDebugVisualizerCamera(
            cameraDistance=6.0,
            cameraYaw=35.0,
            cameraPitch=-30.0,
            cameraTargetPosition=[0.0, 0.0, 0],
        )

        p.loadURDF("plane.urdf")

        box_pos = [2.0, 0.0, 0.5 * BOX_SCALE * 0.05]
        box_orn = p.getQuaternionFromEuler([0, 0, 0])
        box_id = p.loadURDF("cube_small.urdf", box_pos, box_orn, globalScaling=BOX_SCALE)
        p.changeDynamics(
            box_id,
            -1,
            mass=BOX_MASS,
            lateralFriction=0.9,
            rollingFriction=0.01,
            spinningFriction=0.01,
        )

        robots: Dict[str, HuskyRobot] = {}
        formation_spacing = 1.0
        formation_count = 5
        lateral_offset_origin = (formation_count - 1) / 2.0
        for index in range(formation_count):
            name = f"husky_{index}"
            lateral_offset = (index - lateral_offset_origin) * formation_spacing
            start_pos = (0.0, lateral_offset, 0.1)
            enable_camera_controls = index == 0
            robot = self._spawn_husky(name, start_pos, enable_camera_controls)
            robots[name] = robot
        return robots

    def _spawn_husky(
        self, name: str, start_pos: Tuple[float, float, float], enable_camera_controls: bool
    ) -> HuskyRobot:
        start_orn = p.getQuaternionFromEuler([0, 0, 0])
        body_id = p.loadURDF("husky/husky.urdf", start_pos, start_orn, useFixedBase=False)

        wheel_joint_names = {
            "front_left_wheel": None,
            "rear_left_wheel": None,
            "front_right_wheel": None,
            "rear_right_wheel": None,
        }
        for joint_index in range(p.getNumJoints(body_id)):
            name_bytes = p.getJointInfo(body_id, joint_index)[1]
            joint_name = name_bytes.decode()
            if joint_name in wheel_joint_names:
                wheel_joint_names[joint_name] = joint_index

        wheels_left = (
            wheel_joint_names.get("front_left_wheel"),
            wheel_joint_names.get("rear_left_wheel"),
        )
        wheels_right = (
            wheel_joint_names.get("front_right_wheel"),
            wheel_joint_names.get("rear_right_wheel"),
        )

        if any(j is None for j in wheels_left + wheels_right):
            wheels_left = (2, 4)
            wheels_right = (3, 5)

        robot = HuskyRobot(
            name=name,
            body_id=body_id,
            wheels_left=wheels_left,  # type: ignore[arg-type]
            wheels_right=wheels_right,  # type: ignore[arg-type]
            start_pos=start_pos,
            start_orn=start_orn,
        )
        robot.configure_debug_elements(enable_camera_controls=enable_camera_controls)
        robot._perform_reset()
        return robot

    def _get_robots_in_order(self) -> Tuple[HuskyRobot, ...]:
        with self._lock:
            ordered_names = sorted(self.robots.keys())
            return tuple(self.robots[name] for name in ordered_names)

    def _process_command_queue(self) -> None:
        while True:
            try:
                command = self._command_queue.get_nowait()
            except queue.Empty:
                break
            try:
                if command.robot is None:
                    continue
                try:
                    robot = self.get_robot(command.robot)
                except KeyError:
                    continue
                if command.action == "drive":
                    duration = command.payload.get("duration")
                    duration_value: Optional[float] = None
                    if duration is not None:
                        duration_value = max(0.0, float(duration))
                    robot.set_command(
                        float(command.payload.get("vx", 0.0)),
                        float(command.payload.get("wz", 0.0)),
                        duration_value,
                    )
                elif command.action == "stop":
                    robot.stop()
                elif command.action == "reset":
                    robot.request_reset()
                    robot.stop()
            finally:
                self._command_queue.task_done()

    def _update_fleet_camera(self, robots: Tuple[HuskyRobot, ...]) -> None:
        if not robots:
            return
        positions = [p.getBasePositionAndOrientation(robot.body_id)[0] for robot in robots]
        avg_x = sum(pos[0] for pos in positions) / len(positions)
        avg_y = sum(pos[1] for pos in positions) / len(positions)
        radius = max(math.hypot(pos[0] - avg_x, pos[1] - avg_y) for pos in positions)
        primary_robot = robots[0]
        self._viewport_controller.update(primary_robot, DT)
        camera_distance = max(primary_robot.camera_distance, radius * 3.0 + 3.0)
        yaw, pitch = primary_robot.read_camera_orientation()
        target_x, target_y, target_z = self._viewport_controller.target_position(avg_x, avg_y, 0.4)
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=[target_x, target_y, target_z],
        )


@dataclass
class FleetCommand:
    action: str
    robot: Optional[str]
    payload: Dict[str, object]


fleet = HuskyFleet()


# ---------------- Flask API ----------------
app = Flask(__name__)


@app.get("/health")
def health() -> Dict[str, bool]:
    return {"ok": True}


def _fleet_not_ready_error() -> Tuple[Dict[str, str], int]:
    return {"error": "Husky fleet is not initialised"}, 503


def _unknown_robot(robot_id: str) -> Tuple[Dict[str, str], int]:
    return {"error": f"Robot '{robot_id}' not found"}, 404


def _parse_json() -> Dict[str, object]:
    data = request.get_json(silent=True)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError("Request payload must be an object")
    return data


def _ensure_robot(robot_id: str) -> Optional[Tuple[Dict[str, str], int]]:
    if not fleet.wait_until_ready(timeout=0.0):
        return _fleet_not_ready_error()
    if not fleet.robot_exists(robot_id):
        return _unknown_robot(robot_id)
    return None


@app.get("/robots")
def list_robots() -> Dict[str, object]:
    if not fleet.wait_until_ready(timeout=0.0):
        return {"robots": []}
    return {"robots": list(fleet.describe_robots())}


@app.get("/robots/<robot_id>/pose")
def robot_pose(robot_id: str) -> Tuple[Dict[str, object], int]:
    error = _ensure_robot(robot_id)
    if error:
        return error
    description = fleet.describe_robot(robot_id)
    return {
        "robot": description["id"],
        "pose": description["pose"],
        "command": description["command"],
    }, 200


@app.post("/robots/<robot_id>/drive")
def drive(robot_id: str) -> Tuple[Dict[str, object], int]:
    error = _ensure_robot(robot_id)
    if error:
        return error
    try:
        data = _parse_json()
    except ValueError as exc:
        return {"error": str(exc)}, 400

    try:
        vx = float(data.get("vx", 0.0))
        wz = float(data.get("wz", 0.0))
    except (TypeError, ValueError):
        return {"error": "Fields 'vx' and 'wz' must be numeric"}, 400

    duration_value: Optional[float] = None
    if "duration" in data and data["duration"] is not None:
        try:
            duration_value = max(0.0, float(data["duration"]))
        except (TypeError, ValueError):
            return {"error": "Field 'duration' must be numeric"}, 400

    fleet.queue_drive(robot_id, vx, wz, duration_value)
    return {
        "accepted": True,
        "robot": robot_id,
        "command": {
            "vx": clamp(vx, -MAX_VX, MAX_VX),
            "wz": clamp(wz, -MAX_WZ, MAX_WZ),
            "duration": duration_value,
        },
    }, 202


@app.post("/robots/<robot_id>/stop")
def stop(robot_id: str) -> Tuple[Dict[str, object], int]:
    error = _ensure_robot(robot_id)
    if error:
        return error
    fleet.queue_stop(robot_id)
    return {"robot": robot_id, "stopped": True}, 202


@app.post("/robots/<robot_id>/reset")
def api_reset(robot_id: str) -> Tuple[Dict[str, object], int]:
    error = _ensure_robot(robot_id)
    if error:
        return error
    fleet.queue_reset(robot_id)
    return {"robot": robot_id, "reset": True}, 202


def _queue_drive_passthrough(
    robot_id: str, vx: float, wz: float, duration: float
) -> Tuple[Dict[str, object], int]:
    error = _ensure_robot(robot_id)
    if error:
        return error
    duration_value = max(0.0, duration)
    fleet.queue_drive(robot_id, vx, wz, duration_value)
    return {
        "accepted": True,
        "robot": robot_id,
        "command": {"vx": vx, "wz": wz, "duration": duration_value},
    }, 202


@app.post("/robots/<robot_id>/forward")
def forward(robot_id: str) -> Tuple[Dict[str, object], int]:
    try:
        data = _parse_json()
    except ValueError as exc:
        return {"error": str(exc)}, 400
    try:
        speed = float(data.get("speed", 0.6))
        duration = float(data.get("duration", 1.0))
    except (TypeError, ValueError):
        return {"error": "Fields 'speed' and 'duration' must be numeric"}, 400
    return _queue_drive_passthrough(robot_id, speed, 0.0, duration)


@app.post("/robots/<robot_id>/backward")
def backward(robot_id: str) -> Tuple[Dict[str, object], int]:
    try:
        data = _parse_json()
    except ValueError as exc:
        return {"error": str(exc)}, 400
    try:
        speed = float(data.get("speed", 0.6))
        duration = float(data.get("duration", 1.0))
    except (TypeError, ValueError):
        return {"error": "Fields 'speed' and 'duration' must be numeric"}, 400
    return _queue_drive_passthrough(robot_id, -speed, 0.0, duration)


@app.post("/robots/<robot_id>/turn_left")
def turn_left(robot_id: str) -> Tuple[Dict[str, object], int]:
    try:
        data = _parse_json()
    except ValueError as exc:
        return {"error": str(exc)}, 400
    try:
        rate = float(data.get("rate", 1.0))
        duration = float(data.get("duration", 0.8))
    except (TypeError, ValueError):
        return {"error": "Fields 'rate' and 'duration' must be numeric"}, 400
    return _queue_drive_passthrough(robot_id, 0.0, rate, duration)


@app.post("/robots/<robot_id>/turn_right")
def turn_right(robot_id: str) -> Tuple[Dict[str, object], int]:
    try:
        data = _parse_json()
    except ValueError as exc:
        return {"error": str(exc)}, 400
    try:
        rate = float(data.get("rate", 1.0))
        duration = float(data.get("duration", 0.8))
    except (TypeError, ValueError):
        return {"error": "Fields 'rate' and 'duration' must be numeric"}, 400
    return _queue_drive_passthrough(robot_id, 0.0, -rate, duration)


@app.post("/fleet/reset")
def fleet_reset() -> Tuple[Dict[str, object], int]:
    if not fleet.wait_until_ready(timeout=0.0):
        return _fleet_not_ready_error()
    fleet.queue_fleet_reset()
    return {"reset": True, "robots": list(fleet.robot_ids())}, 202


# ---------------- Convenience helpers ----------------
def start_fleet() -> Optional[threading.Thread]:
    fleet.start()
    fleet.wait_until_ready(timeout=10.0)
    return fleet._thread


def shutdown_fleet(timeout: Optional[float] = None) -> None:
    fleet.stop(timeout)


if __name__ == "__main__":
    start_fleet()
    base_url = "http://127.0.0.1:5000"
    robot_ids = list(fleet.robot_ids())

    print("\nREST controls ready:")
    print(f"  GET  {base_url}/robots")

    if robot_ids:
        print("\nPer-robot examples:")
        for robot_id in robot_ids:
            print(f"  GET  {base_url}/robots/{robot_id}/pose")
            print(
                f"  POST {base_url}/robots/{robot_id}/drive      {{\"vx\":0.6,\"wz\":0.0,\"duration\":2}}"
            )
            print(f"  POST {base_url}/robots/{robot_id}/stop")

    print("\nAdditional helpers: /forward, /backward, /turn_left, /turn_right per robot ID.")
    print()
    app.run(host="127.0.0.1", port=5000, debug=False, threaded=True)
