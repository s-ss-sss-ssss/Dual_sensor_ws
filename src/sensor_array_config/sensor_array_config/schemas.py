from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np


@dataclass(frozen=True)
class AffineModelParams:
    D_i: List[List[float]]
    e_i: List[float]


@dataclass(frozen=True)
class AffineModelParamsSet:
    params: Dict[int, AffineModelParams]


@dataclass(frozen=True)
class RCorrEntry:
    sensor_ids: List[int]
    matrix: List[float]

    def to_numpy(self) -> np.ndarray:
        return np.array(self.matrix, dtype=float).reshape(3, 3, order="F")


@dataclass(frozen=True)
class MagnetometerConfig:
    name: str
    manufacturer: str
    bit_width: int
    adu_to_gs: float
    gs_to_tesla: float
    n_sensors: int
    n_groups: int
    sensors_per_group: int
    frame_id: str
    d_list: List[List[float]]
    R_CORR: List[RCorrEntry]
    affine_model: AffineModelParamsSet
    range_gs: Optional[float] = None
    calibration_status: str = ""
    notes: str = ""

    @property
    def gs_to_si(self) -> float:
        return self.gs_to_tesla

    def get_sensor_ids(self) -> List[int]:
        return list(range(1, self.n_sensors + 1))

    def get_group_for_sensor(self, sensor_id: int) -> int:
        return (sensor_id - 1) // self.sensors_per_group

    def get_sensors_in_group(self, group: int) -> List[int]:
        start = group * self.sensors_per_group + 1
        return list(range(start, start + self.sensors_per_group))


@dataclass(frozen=True)
class ImuConfig:
    name: str
    imu_type: str
    imu_frame_id: str
    publish_scaled_imu: bool
    accel_range_g: float
    gyro_range_dps: float
    accel_odr_hz: float
    gyro_odr_hz: float
    accel_lsb_per_g: float
    gyro_lsb_per_dps: float
    temp_lsb_per_c: float
    temp_offset_c: float
    position_m: List[float]
    rotation_deg: float
    axis_transform_matrix: List[List[float]]

    def axis_transform_numpy(self) -> np.ndarray:
        return np.array(self.axis_transform_matrix, dtype=float).reshape(3, 3)


@dataclass(frozen=True)
class HardwareConfig:
    name: str
    firmware_protocol: str
    magnetometer: MagnetometerConfig
    imu: ImuConfig
    calibration_status: str = ""
    notes: str = ""

    @property
    def adu_to_gs(self) -> float:
        return self.magnetometer.adu_to_gs

    @property
    def gs_to_si(self) -> float:
        return self.magnetometer.gs_to_si
