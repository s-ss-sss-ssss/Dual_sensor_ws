import numpy as np

from .schemas import HardwareConfig, ImuConfig, MagnetometerConfig


def _require(condition: bool, message: str):
    if not condition:
        raise ValueError(message)


def validate_magnetometer_config(config: MagnetometerConfig):
    sensor_ids = set(range(1, config.n_sensors + 1))
    _require(bool(config.name), "magnetometer name is required")
    _require(config.bit_width > 0, f"{config.name}: bit_width must be positive")
    _require(config.adu_to_gs > 0.0, f"{config.name}: adu_to_gs must be positive")
    _require(config.gs_to_tesla > 0.0, f"{config.name}: gs_to_tesla must be positive")
    _require(config.n_sensors > 0, f"{config.name}: n_sensors must be positive")
    _require(config.n_groups > 0, f"{config.name}: n_groups must be positive")
    _require(config.sensors_per_group > 0, f"{config.name}: sensors_per_group must be positive")
    _require(len(config.d_list) == config.n_sensors, f"{config.name}: n_sensors != len(d_list)")

    r_corr_ids = set()
    for entry in config.R_CORR:
        _require(len(entry.matrix) == 9, f"{config.name}: R_CORR matrix must have 9 values")
        r_corr_ids.update(int(sid) for sid in entry.sensor_ids)
    _require(r_corr_ids == sensor_ids, f"{config.name}: R_CORR must cover all sensors")

    affine_ids = set(config.affine_model.params)
    _require(affine_ids == sensor_ids, f"{config.name}: affine params must cover all sensors")
    for sid, params in config.affine_model.params.items():
        _require(np.array(params.D_i).shape == (3, 3), f"{config.name}: sensor {sid} D_i must be 3x3")
        _require(np.array(params.e_i).reshape(-1).shape == (3,), f"{config.name}: sensor {sid} e_i must have length 3")


def validate_imu_config(config: ImuConfig):
    _require(bool(config.name), "IMU config name is required")
    _require(np.array(config.axis_transform_matrix).shape == (3, 3), f"{config.name}: axis_transform_matrix must be 3x3")
    _require(len(config.position_m) == 3, f"{config.name}: position_m must have length 3")
    _require(config.accel_lsb_per_g > 0.0, f"{config.name}: accel_lsb_per_g must be positive")
    _require(config.gyro_lsb_per_dps > 0.0, f"{config.name}: gyro_lsb_per_dps must be positive")


def validate_hardware_config(config: HardwareConfig):
    _require(bool(config.name), "hardware config name is required")
    _require(bool(config.firmware_protocol), f"{config.name}: firmware_protocol is required")
    validate_magnetometer_config(config.magnetometer)
    validate_imu_config(config.imu)
