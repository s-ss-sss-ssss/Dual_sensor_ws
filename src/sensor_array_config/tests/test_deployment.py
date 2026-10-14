import copy
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import yaml

from sensor_array_config import (
    get_hardware_config, get_hardware_config_from_directory,
    list_hardware_configs, load_deployment,
)
from sensor_array_config.loader import CONFIG_ROOT


class DeploymentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / "deployment.yaml"
        self.device = {
            "device_id": "left", "namespace": "//arrays//left/",
            "port": "/dev/ttyARRAY_LEFT", "hardware_config": "qmc6309",
        }

    def load(self, devices=None, **top):
        raw = {"schema_version": 1, "devices": devices if devices is not None else [self.device]}
        raw.update(top)
        self.path.write_text(yaml.safe_dump(raw), encoding="utf-8")
        return load_deployment(self.path)

    def second(self):
        return dict(self.device, device_id="right", namespace="arrays/right", port="/dev/ttyARRAY_RIGHT")

    def external(self):
        directory = self.root / "external"
        shutil.copytree(CONFIG_ROOT / "qmc6309", directory)
        self.device["config_dir"] = "external"
        return directory

    def test_defaults_and_historical_protocol(self):
        result = self.load()
        self.assertEqual(set(result), {"schema_version", "devices"})
        device = result["devices"][0]
        self.assertEqual(device["namespace"], "/arrays/left")
        self.assertEqual(device["baud_rate"], 115200)
        self.assertEqual(device["frame_queue_size"], 512)
        self.assertEqual(device["publish_batch_size"], 512)
        self.assertEqual(device["hardware_config"], "qmc6309")
        self.assertEqual(device["firmware_protocol"], get_hardware_config("qmc6309").firmware_protocol)
        self.assertFalse(device["calibration_applied"])
        self.assertFalse(device["identity_verified"])
        self.assertEqual(device["topics"], {
            "raw_frames": "/arrays/left/serial_frame_bridge/raw_frames",
            "tx_bytes": "/arrays/left/serial_frame_bridge/tx_bytes",
            "sample": "/arrays/left/maggrad/sample",
            "diagnostics": "/arrays/left/diagnostics",
        })
        self.assertEqual(set(device["configuration_files"]), {"board", "magnetometer", "array", "affine", "imu"})
        for path in device["configuration_files"].values():
            self.assertTrue(Path(path).is_absolute())
            self.assertTrue(Path(path).is_file())

    def test_all_existing_configs_and_legacy_interface(self):
        for name in list_hardware_configs():
            with self.subTest(name=name):
                self.device["hardware_config"] = name
                device = self.load()["devices"][0]
                self.assertEqual(get_hardware_config(name), get_hardware_config_from_directory(device["config_dir"]))

    def test_external_references_and_optional_metadata(self):
        directory = self.external()
        board_path = directory / "board.json"
        board = json.loads(board_path.read_text())
        (directory / board["files"]["array"]).rename(directory / "custom_array.json")
        board["files"]["array"] = "custom_array.json"
        board_path.write_text(json.dumps(board))
        (self.root / "sensors.yaml").write_text("sensors: []\n")
        self.device.update(expected_sensors_file="sensors.yaml", core_board_id="00123", firmware_device_id=0xFFFFFFFF,
                           baud_rate=921600, frame_queue_size=64, publish_batch_size=32)
        device = self.load()["devices"][0]
        self.assertEqual(device["config_dir"], str(directory.resolve()))
        self.assertEqual(device["configuration_files"]["array"], str(directory / "custom_array.json"))
        self.assertEqual(device["expected_sensors_file"], str(self.root / "sensors.yaml"))
        self.assertEqual(device["core_board_id"], "00123")
        self.assertEqual(device["firmware_device_id"], 0xFFFFFFFF)
        self.assertEqual((device["baud_rate"], device["frame_queue_size"], device["publish_batch_size"]), (921600, 64, 32))

    def test_shared_diagnostics_and_topic_overrides(self):
        self.device["topics"] = {"sample": "custom//sample/", "diagnostics": "/shared/diagnostics"}
        second = self.second()
        second["topics"] = {"diagnostics": "/shared//diagnostics/"}
        devices = self.load([self.device, second])["devices"]
        self.assertEqual(devices[0]["topics"]["sample"], "/arrays/left/custom/sample")
        self.assertEqual(devices[0]["topics"]["diagnostics"], devices[1]["topics"]["diagnostics"])

    def test_duplicate_identity_namespace_and_port(self):
        for key, value in (("device_id", "left"), ("namespace", "/arrays/left"), ("port", self.device["port"])):
            with self.subTest(key=key):
                second = self.second()
                second[key] = value
                with self.assertRaisesRegex(ValueError, "duplicate " + key):
                    self.load([self.device, second])

    def test_serial_symlink_collision(self):
        target = self.root / "serial"
        target.touch()
        alias = self.root / "by-id"
        alias.symlink_to(target)
        self.device["port"] = str(alias)
        self.assertEqual(self.load([self.device])["devices"][0]["port"], str(alias))
        self.device["port"] = str(target)
        second = self.second()
        second["port"] = str(alias)
        with self.assertRaisesRegex(ValueError, "duplicate port"):
            self.load([self.device, second])

    def test_all_topic_collision_combinations(self):
        keys = ("raw_frames", "tx_bytes", "sample", "diagnostics")
        for left in keys:
            for right in keys:
                if left == right == "diagnostics":
                    continue
                with self.subTest(left=left, right=right):
                    first, second = copy.deepcopy(self.device), self.second()
                    first["topics"] = {left: "/collision"}
                    second["topics"] = {right: "/collision"}
                    with self.assertRaisesRegex(ValueError, "topic collision"):
                        self.load([first, second])
        self.device["topics"] = {"sample": "same", "diagnostics": "same"}
        with self.assertRaisesRegex(ValueError, "topic collision"):
            self.load()

    def test_required_and_unknown_fields(self):
        for key in ("device_id", "namespace", "port", "hardware_config"):
            entry = dict(self.device)
            del entry[key]
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "missing fields"):
                self.load([entry])
        with self.assertRaisesRegex(ValueError, "unknown fields"):
            self.load(typo=True)
        with self.assertRaisesRegex(ValueError, "unknown fields"):
            self.load([dict(self.device, baudrate=115200)])
        self.device["topics"] = {"samples": "sample"}
        with self.assertRaisesRegex(ValueError, "unknown fields"):
            self.load()

    def test_invalid_types_values_and_names(self):
        invalid = {
            "device_id": [True, 1, "", " left"],
            "namespace": [True, "/", "///", "~left", "bad-name", "left space", "123"],
            "port": [True, "auto", "ttyUSB0", ""],
            "hardware_config": [True, "../qmc6309", "/qmc6309", ""],
            "baud_rate": [True, 0, -1, 115200.0, "115200"],
            "frame_queue_size": [False, 0, 1.5, "512"],
            "publish_batch_size": [True, -1, None],
            "config_dir": [True, "", 1],
            "expected_sensors_file": [False, "", 1],
            "core_board_id": [True, 123, ""],
            "firmware_device_id": [True, -1, 0x100000000, 1.0, "1"],
            "topics": [True, [], None],
        }
        for key, values in invalid.items():
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    self.load([dict(self.device, **{key: value})])
        for value in (True, "/", "~private", "bad-name", "bad.name", "", "space name"):
            with self.subTest(topic=value), self.assertRaises(ValueError):
                self.load([dict(self.device, topics={"sample": value})])
        for version in (True, "1", 1.0, 2, None):
            with self.subTest(version=version), self.assertRaises(ValueError):
                self.load(schema_version=version)
        for devices in ([], {}, [True], [None]):
            with self.subTest(devices=devices), self.assertRaises(ValueError):
                self.load(devices)

    def test_uint32_zero(self):
        self.device["firmware_device_id"] = 0
        self.assertEqual(self.load()["devices"][0]["firmware_device_id"], 0)

    def test_strict_yaml(self):
        for text in ("schema_version: 1\nschema_version: 1\ndevices: []\n", "[]", "null", "schema_version: [", "!!python/object:builtins.object {}", "schema_version: 1\ndevices:\n- device_id: a\n  device_id: b\n"):
            self.path.write_text(text)
            with self.subTest(text=text), self.assertRaises((ValueError, yaml.YAMLError)):
                load_deployment(self.path)

    def test_missing_files(self):
        with self.assertRaises(FileNotFoundError):
            load_deployment(self.path)
        self.device["hardware_config"] = "not_existing"
        with self.assertRaises(FileNotFoundError):
            self.load()
        self.device["hardware_config"] = "qmc6309"
        self.device["expected_sensors_file"] = "missing.yaml"
        with self.assertRaises(FileNotFoundError):
            self.load()

    def test_reuses_hardware_validation(self):
        directory = self.external()
        board = json.loads((directory / "board.json").read_text())
        array_path = directory / board["files"]["array"]
        array = json.loads(array_path.read_text())
        array["d_list"] = []
        array_path.write_text(json.dumps(array))
        with self.assertRaisesRegex(ValueError, "n_sensors != len"):
            self.load()

    def test_hardware_validator_is_called(self):
        with patch("sensor_array_config.loader.validate_hardware_config") as validate:
            self.load()
            validate.assert_called_once()


if __name__ == "__main__":
    unittest.main()
