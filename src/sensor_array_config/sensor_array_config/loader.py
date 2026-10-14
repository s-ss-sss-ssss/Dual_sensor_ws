import json
from pathlib import Path
from typing import Any, Dict, List

from .schemas import (
    AffineModelParams,
    AffineModelParamsSet,
    HardwareConfig,
    ImuConfig,
    MagnetometerConfig,
    RCorrEntry,
)
from .validation import validate_hardware_config


def _resolve_config_root() -> Path:
    source_root = Path(__file__).resolve().parents[1] / "config"
    if source_root.exists():
        return source_root

    try:
        import rospkg
    except ImportError as exc:
        raise FileNotFoundError(
            f"sensor_array_config config directory not found at {source_root}"
        ) from exc

    ros_root = Path(rospkg.RosPack().get_path("sensor_array_config")) / "config"
    if ros_root.exists():
        return ros_root
    raise FileNotFoundError(f"sensor_array_config config directory not found at {ros_root}")


CONFIG_ROOT = _resolve_config_root()


def _load_mapping(path: Path) -> Dict[str, Any]:
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping in {path}")
    return data


def _require_file(path: Path) -> Path:
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def _hardware_dir(name: str) -> Path:
    root = CONFIG_ROOT / name.lower()
    _require_file(root / "board.json")
    return root


def _load_hardware_file(root: Path, files: Dict[str, str], key: str) -> Dict[str, Any]:
    if key not in files:
        raise ValueError(f"{root.name}: board.json missing files.{key}")
    path = _require_file(root / files[key])
    return _load_mapping(path)


def _load_affine(raw: Dict[str, Any]) -> AffineModelParamsSet:
    return AffineModelParamsSet(
        params={
            int(entry["sensor_id"]): AffineModelParams(D_i=entry["D_i"], e_i=entry["e_i"])
            for entry in raw["sensors"]
        }
    )


def get_hardware_config(name: str) -> HardwareConfig:
    return get_hardware_config_from_directory(_hardware_dir(name))


def get_hardware_config_from_directory(directory: str) -> HardwareConfig:
    """Load and validate an existing board.json + files configuration directory."""
    hardware_dir = Path(directory).expanduser().resolve()
    raw = _load_mapping(_require_file(hardware_dir / "board.json"))
    files = raw.get("files")
    if not isinstance(files, dict):
        raise ValueError(f"{raw.get('name', hardware_dir.name)}: board.json files mapping is required")
    mag_raw = _load_hardware_file(hardware_dir, files, "magnetometer")
    array_raw = _load_hardware_file(hardware_dir, files, "array")
    affine_raw = _load_hardware_file(hardware_dir, files, "affine")
    mag_raw["d_list"] = array_raw["d_list"]
    mag_raw["R_CORR"] = [RCorrEntry(**entry) for entry in array_raw["R_CORR"]]
    mag_raw["affine_model"] = _load_affine(affine_raw)
    config = HardwareConfig(
        name=raw["name"],
        firmware_protocol=raw["firmware_protocol"],
        calibration_status=raw.get("calibration_status", ""),
        notes=raw.get("notes", ""),
        magnetometer=MagnetometerConfig(**mag_raw),
        imu=ImuConfig(**_load_hardware_file(hardware_dir, files, "imu")),
    )
    validate_hardware_config(config)
    return config


def list_hardware_configs() -> List[str]:
    return sorted(path.parent.name for path in CONFIG_ROOT.glob("*/board.json"))
