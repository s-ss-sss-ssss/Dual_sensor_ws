"""Strict, offline resolution of multi-board deployment manifests."""
import json
import re
from pathlib import Path
from typing import Any, Dict

import yaml

from .loader import CONFIG_ROOT, get_hardware_config_from_directory


class _DeploymentLoader(yaml.SafeLoader):
    pass


def _unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str):
            raise ValueError("YAML mapping keys must be strings")
        if key in result:
            raise ValueError(f"Duplicate YAML key: {key}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_DeploymentLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping
)

_REQUIRED = {"device_id", "namespace", "port", "hardware_config"}
_OPTIONAL = {
    "baud_rate", "config_dir", "topics", "frame_queue_size", "publish_batch_size",
    "expected_sensors_file", "core_board_id", "firmware_device_id",
}
_TOPIC_SUFFIXES = {
    "raw_frames": "serial_frame_bridge/raw_frames",
    "tx_bytes": "serial_frame_bridge/tx_bytes",
    "sample": "maggrad/sample",
    "diagnostics": "diagnostics",
}


def _mapping(value, allowed, context):
    if not isinstance(value, dict):
        raise ValueError(f"{context} must be a mapping")
    unknown = set(value) - allowed
    if unknown:
        raise ValueError(f"{context}: unknown fields {sorted(unknown, key=str)}")
    return value


def _string(value, context):
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{context} must be a nonempty string without surrounding whitespace")
    return value


def _integer(value, context, minimum=1, maximum=None):
    if type(value) is not int or value < minimum or (maximum is not None and value > maximum):
        raise ValueError(f"{context} must be an integer in [{minimum}, {maximum or 'unbounded'}]")
    return value


def _ros_name(value, context, namespace=None):
    value = _string(value, context)
    if not re.fullmatch(r"[A-Za-z/][A-Za-z0-9_/]*", value):
        raise ValueError(f"{context}: invalid ROS name {value!r}")
    if not value.startswith("/"):
        value = (namespace or "") + "/" + value
    value = "/" + "/".join(part for part in value.split("/") if part)
    if value == "/":
        raise ValueError(f"{context}: root name is not allowed")
    return value


def _relative_path(value, base, context):
    path = Path(_string(value, context)).expanduser()
    return (path if path.is_absolute() else base / path).resolve()


def load_deployment(path) -> Dict[str, Any]:
    """Resolve a schema-v1 YAML manifest; no serial I/O or calibration is applied.

    Invalid manifests raise ValueError; missing configuration files raise
    FileNotFoundError. PyYAML is required, with no fallback parser.
    """
    manifest = Path(path).expanduser().absolute()
    with manifest.open(encoding="utf-8") as stream:
        raw = yaml.load(stream, Loader=_DeploymentLoader)
    raw = _mapping(raw, {"schema_version", "devices"}, "deployment")
    if type(raw.get("schema_version")) is not int or raw["schema_version"] != 1:
        raise ValueError("deployment.schema_version must be integer 1")
    if not isinstance(raw.get("devices"), list) or not raw["devices"]:
        raise ValueError("deployment.devices must be a nonempty list")

    devices = []
    seen = {"device_id": set(), "namespace": set(), "port": set()}
    topic_owners = {}
    for index, entry in enumerate(raw["devices"]):
        context = f"devices[{index}]"
        entry = _mapping(entry, _REQUIRED | _OPTIONAL, context)
        missing = _REQUIRED - set(entry)
        if missing:
            raise ValueError(f"{context}: missing fields {sorted(missing)}")
        device_id = _string(entry["device_id"], context + ".device_id")
        namespace = _ros_name(entry["namespace"], context + ".namespace")
        port = _string(entry["port"], context + ".port")
        if not Path(port).is_absolute():
            raise ValueError(f"{context}.port must be an absolute, non-auto serial path")
        # Preserve stable by-id/udev aliases for future reconnects; resolve only
        # for detecting two names that currently point to the same device.
        port_target = str(Path(port).resolve())
        for key, value in (("device_id", device_id), ("namespace", namespace), ("port", port_target)):
            if value in seen[key]:
                raise ValueError(f"{context}: duplicate {key} {value!r}")
            seen[key].add(value)

        name = _string(entry["hardware_config"], context + ".hardware_config")
        if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]*", name):
            raise ValueError(f"{context}.hardware_config must be a configuration name, not a path")
        directory = (
            _relative_path(entry["config_dir"], manifest.parent, context + ".config_dir")
            if "config_dir" in entry else (CONFIG_ROOT / name.lower()).resolve()
        )
        hardware = get_hardware_config_from_directory(directory)
        # Reuse the board's file references, rather than assume subfile names.
        board = directory / "board.json"
        files = json.loads(board.read_text(encoding="utf-8"))["files"]
        configuration_files = {"board": str(board.resolve())}
        for key in ("magnetometer", "array", "affine", "imu"):
            configuration_files[key] = str((directory / files[key]).resolve())

        overrides = _mapping(entry.get("topics", {}), set(_TOPIC_SUFFIXES), context + ".topics")
        topics = {
            key: _ros_name(overrides.get(key, suffix), context + ".topics." + key, namespace)
            for key, suffix in _TOPIC_SUFFIXES.items()
        }
        for key, topic in topics.items():
            previous = topic_owners.get(topic)
            if previous is not None and not (previous == key == "diagnostics"):
                raise ValueError(f"{context}: topic collision at {topic}")
            topic_owners[topic] = key

        resolved = {
            "device_id": device_id, "namespace": namespace, "port": port,
            "baud_rate": _integer(entry.get("baud_rate", 115200), context + ".baud_rate"),
            "hardware_config": name, "config_dir": str(directory),
            "configuration_files": configuration_files, "topics": topics,
            "frame_queue_size": _integer(entry.get("frame_queue_size", 512), context + ".frame_queue_size"),
            "publish_batch_size": _integer(entry.get("publish_batch_size", 512), context + ".publish_batch_size"),
            "firmware_protocol": hardware.firmware_protocol,
            "calibration_status": hardware.calibration_status,
            "calibration_applied": False, "identity_verified": False,
            "configuration_note": "Referenced configuration only; calibration is not applied and hardware identity is not verified online.",
        }
        if "expected_sensors_file" in entry:
            expected = _relative_path(entry["expected_sensors_file"], manifest.parent, context + ".expected_sensors_file")
            if not expected.is_file():
                raise FileNotFoundError(expected)
            resolved["expected_sensors_file"] = str(expected)
        if "core_board_id" in entry:
            resolved["core_board_id"] = _string(entry["core_board_id"], context + ".core_board_id")
        if "firmware_device_id" in entry:
            resolved["firmware_device_id"] = _integer(
                entry["firmware_device_id"], context + ".firmware_device_id", 0, 0xFFFFFFFF
            )
        devices.append(resolved)
    return {"schema_version": 1, "devices": devices}
