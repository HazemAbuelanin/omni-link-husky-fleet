#!/usr/bin/env python3
"""Remote polling bridge that drives Husky robot commands via OmniLink."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List

from omnilink import (
    OmniLinkEngine,
    OmniLinkRemoteCommandBridge,
    TypeRegistry,
    give_context,
    load_patterns_from_file,
    start_periodic_context,
)
from robot_api import backward, forward, get_pose, stop, turn_left, turn_right

HERE = Path(__file__).resolve().parent
PATTERNS_FILE = HERE / "robot_commands.txt"

types = TypeRegistry()
templates = load_patterns_from_file(PATTERNS_FILE, types)
engine = OmniLinkEngine(templates, types=types)

_NUMBER_RE = re.compile(r"-?\d+(?:\.\d+)?")


def _extract_numbers(command: str) -> List[float]:
    """Return all numeric values embedded in ``command`` as floats."""

    return [float(match) for match in _NUMBER_RE.findall(command)]


def _call_motion(
    evt: Dict[str, Any],
    handler: Callable[[float, float], Dict[str, Any]],
    *,
    expected: int = 2,
) -> Dict[str, Any]:
    """Run ``handler`` with ``expected`` numeric arguments parsed from the event."""

    numbers = _extract_numbers(evt.get("command", ""))
    if len(numbers) < expected:
        print(
            "[link_remote] Not enough numeric arguments for template",
            evt.get("template"),
        )
        return {"ack": False}

    args: Iterable[float] = numbers[:expected]
    try:
        result = handler(*args)
    except Exception as exc:  # pragma: no cover - defensive logging
        print(f"[link_remote] Robot command failed: {exc}")
        return {"ack": False, "error": str(exc)}

    return {"ack": True, "result": result}


def _handle_stop(_evt: Dict[str, Any]) -> Dict[str, Any]:
    try:
        result = stop()
    except Exception as exc:  # pragma: no cover - defensive logging
        print(f"[link_remote] Failed to stop robot: {exc}")
        return {"ack": False, "error": str(exc)}
    return {"ack": True, "result": result}


_DISPATCH: Dict[str, Callable[[Dict[str, Any]], Dict[str, Any]]] = {
    "move_forward_at_[number]_m/s_for_[number]_seconds": lambda evt: _call_motion(
        evt, forward
    ),
    "move_backward_at_[number]_m/s_for_[number]_seconds": lambda evt: _call_motion(
        evt, backward
    ),
    "turn_right_at_[number]_ras/s_for_[number]_seconds": lambda evt: _call_motion(
        evt, turn_right
    ),
    "turn_left_at_[number]_ras/s_for_[number]_seconds": lambda evt: _call_motion(
        evt, turn_left
    ),
    "stop": _handle_stop,
}


def _context_payload() -> str:
    """Build a compact JSON payload describing the latest Husky pose."""

    try:
        pose = get_pose()
    except Exception as exc:  # pragma: no cover - telemetry helper
        return json.dumps({"pose_error": str(exc)})

    if not isinstance(pose, dict):
        return json.dumps({"pose": pose})
    return json.dumps({"pose": pose})


def _send_full_context(*_args: Any) -> None:
    """Push a snapshot of the simulator pose to the MQTT context topic."""

    try:
        give_context(_context_payload())
    except RuntimeError:
        # No MQTT bridge has been initialised; running the remote bridge alone is fine.
        pass


def _handle_any(event: Dict[str, Any]) -> Dict[str, Any]:
    """Execute any recognised command and report acknowledgement."""

    template = event.get("template")
    if not template:
        print("[link_remote] Event did not match a known template")
        return {"ack": False}

    handler = _DISPATCH.get(template)
    if handler is None:
        print(f"[link_remote] No handler for template: {template}")
        return {"ack": False}

    return handler(event)


engine.on(lambda _event: True, _handle_any)


def main() -> None:
    """Start polling Supabase for remote Husky commands."""

    try:
        _send_full_context()
        start_periodic_context(5, _context_payload)
    except RuntimeError:
        # It is valid to run the remote bridge without an MQTT bridge.
        pass

    bridge = OmniLinkRemoteCommandBridge(engine)
    bridge.loop_forever()


if __name__ == "__main__":
    main()
