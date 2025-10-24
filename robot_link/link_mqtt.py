# link.py — load templates from file + MQTT bridge
"""MQTT bridge that wires OmniLink commands to the robot API helpers."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List

from omnilink import (
    OmniLinkEngine,
    OmniLinkMQTTBridge,
    TypeRegistry,
    load_patterns_from_file,
)
from robot_api import backward, forward, list_robots, stop, turn_left, turn_right

HERE = Path(__file__).resolve().parent
PATTERNS_FILE = HERE / "fleet_commands.txt"

types = TypeRegistry()
TEMPLATES = load_patterns_from_file(PATTERNS_FILE, types)
engine = OmniLinkEngine(TEMPLATES, types=types)

_NUMBER_RE = re.compile(r"-?\d+(?:\.\d+)?")


def _extract_numbers(command: str) -> List[float]:
    """Return all numeric values embedded in ``command`` as floats."""

    return [float(match) for match in _NUMBER_RE.findall(command)]


def _format_robot_id(raw_id: float) -> str:
    """Convert the first numeric token in a command to a Husky robot ID."""

    index = int(raw_id)
    if abs(raw_id - index) > 1e-6:
        raise ValueError(f"Robot index must be an integer, got {raw_id}")
    if index < 0:
        raise ValueError(f"Robot index must be non-negative, got {index}")
    return f"husky_{index}"


def _call_robot_motion(
    evt: Dict[str, Any],
    handler: Callable[..., Dict[str, Any]],
    *,
    expected: int = 2,
) -> Dict[str, Any]:
    """Run ``handler`` with ``expected`` numeric arguments after the robot index."""

    numbers = _extract_numbers(evt.get("command", ""))
    required = expected + 1  # account for the leading robot index
    if len(numbers) < required:
        print(
            "[link_mqtt] Not enough numeric arguments for template",
            evt.get("template"),
        )
        return {"ack": False}

    try:
        robot_id = _format_robot_id(numbers[0])
    except ValueError as exc:
        print(f"[link_mqtt] {exc}")
        return {"ack": False, "error": str(exc)}

    args: Iterable[float] = numbers[1:required]
    try:
        result = handler(*args, robot_id=robot_id)
    except Exception as exc:  # pragma: no cover - defensive logging
        print(f"[link_mqtt] Robot command failed: {exc}")
        return {"ack": False, "error": str(exc)}

    return {"ack": True, "result": result}


def _handle_stop(_evt: Dict[str, Any]) -> Dict[str, Any]:
    """Stop every robot reported by the fleet manifest."""

    try:
        manifest = list_robots()
    except Exception as exc:  # pragma: no cover - defensive logging
        print(f"[link_mqtt] Failed to fetch robot manifest: {exc}")
        return {"ack": False, "error": str(exc)}

    robots = manifest.get("robots") if isinstance(manifest, dict) else None
    robot_ids = []
    if isinstance(robots, list):
        for entry in robots:
            if isinstance(entry, dict) and "id" in entry:
                robot_ids.append(str(entry["id"]))
    if not robot_ids:
        # Fallback to the default robot if the manifest is empty.
        robot_ids = [None]

    results: Dict[str, Any] = {}
    errors: Dict[str, str] = {}
    for robot_id in robot_ids:
        try:
            response = stop(robot_id=robot_id)
            key = robot_id if robot_id is not None else "default"
            results[key] = response
        except Exception as exc:  # pragma: no cover - defensive logging
            key = robot_id if robot_id is not None else "default"
            errors[str(key)] = str(exc)

    if errors and not results:
        return {"ack": False, "error": errors}

    payload: Dict[str, Any] = {"stopped": results}
    if errors:
        payload["errors"] = errors
    return {"ack": not errors, "result": payload}


_DISPATCH: Dict[str, Callable[[Dict[str, Any]], Dict[str, Any]]] = {
    "move_robot_[number]_forward_at_[number]_m/s_for_[number]_seconds": lambda evt: _call_robot_motion(
        evt, forward
    ),
    "move_robot_[number]_backward_at_[number]_m/s_for_[number]_seconds": lambda evt: _call_robot_motion(
        evt, backward
    ),
    "turn_robot_[number]_right_at_[number]_rad/s_for_[number]_seconds": lambda evt: _call_robot_motion(
        evt, turn_right
    ),
    "turn_robot_[number]_left_at_[number]_rad/s_for_[number]_seconds": lambda evt: _call_robot_motion(
        evt, turn_left
    ),
    "stop": _handle_stop,
}


def handle_any(evt: Dict[str, Any]) -> Dict[str, Any]:
    """Dispatch recognised templates to the robot control helpers."""

    template = evt.get("template")
    if not template:
        print("[link_mqtt] Event did not match a known template")
        return {"ack": False}

    handler = _DISPATCH.get(template)
    if handler is None:
        print(f"[link_mqtt] No handler for template: {template}")
        return {"ack": False}

    return handler(evt)


engine.on(lambda _evt: True, handle_any)

bridge = OmniLinkMQTTBridge(engine)
bridge.loop_forever()
