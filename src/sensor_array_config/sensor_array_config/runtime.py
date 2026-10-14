from pathlib import Path
from typing import Any, Dict, Optional


DEFAULT_RUNTIME_CONFIG = {
    "hardware_config": "qmc6309",
    "startup_strategy": "cont",
    "startup_sensors": "auto",
    "trigger_rate_hz": 100,
    "icm_rate_hz": 480,
    "profile": "LOW_NOISE",
    "port": "auto",
    "baudrate": 115200,
}

RUNTIME_SENTINELS = {"", "runtime", "default"}


def _load_mapping(path: Path) -> Dict[str, Any]:
    text = path.read_text()
    try:
        import yaml
    except ImportError:
        data = {}
        for lineno, raw_line in enumerate(text.splitlines(), start=1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            if ":" not in line:
                raise ValueError(f"Invalid runtime config line {lineno} in {path}: {raw_line}")
            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip().strip("'\"")
            if value.lower() in ("true", "false"):
                parsed_value: Any = value.lower() == "true"
            else:
                try:
                    parsed_value = int(value)
                except ValueError:
                    parsed_value = value
            data[key] = parsed_value
        return data

    data = yaml.safe_load(text)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping in runtime config: {path}")
    return data


def get_workspace_root() -> Path:
    try:
        import rospkg

        package_root = Path(rospkg.RosPack().get_path("sensor_array_config"))
    except Exception:
        package_root = Path(__file__).resolve().parents[1]
    return package_root.parents[1]


def get_runtime_config_path(path: Optional[str] = None) -> Path:
    if path:
        return Path(path).expanduser()

    import os

    env_path = os.environ.get("MIGELS_RUNTIME_CONFIG")
    if env_path:
        return Path(env_path).expanduser()
    return get_workspace_root() / "migels_runtime.yaml"


def get_runtime_config(path: Optional[str] = None) -> Dict[str, Any]:
    config = dict(DEFAULT_RUNTIME_CONFIG)
    config_path = get_runtime_config_path(path)
    if config_path.exists():
        config.update(_load_mapping(config_path))
    return config


def is_runtime_value(value: Any) -> bool:
    return isinstance(value, str) and value.strip().lower() in RUNTIME_SENTINELS


def resolve_runtime_value(value: Any, key: str, fallback: Any = None, path: Optional[str] = None) -> Any:
    if is_runtime_value(value):
        return get_runtime_config(path).get(key, fallback)
    return value
