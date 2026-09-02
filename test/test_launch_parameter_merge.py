"""Regression tests for multi-file launch parameter composition."""

import importlib.util
import tempfile
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "launch" / "include" / "module_loam.launch.py"
SPEC = importlib.util.spec_from_file_location("module_loam_launch", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

ZMQ_MODULE_PATH = REPO / "launch" / "run_zmq_bridge.launch.py"
ZMQ_SPEC = importlib.util.spec_from_file_location(
    "run_zmq_bridge_launch", ZMQ_MODULE_PATH)
ZMQ_MODULE = importlib.util.module_from_spec(ZMQ_SPEC)
assert ZMQ_SPEC.loader is not None
ZMQ_SPEC.loader.exec_module(ZMQ_MODULE)


def test_robot_frame_override_replaces_nested_yaml_value():
    shared = {
        "liorf": {
            "mapFrame": "map",
            "mapFusionFrame": "",
            "sensor": "livox",
        },
        "robot_id": "jackal0",
    }

    result = MODULE._with_overrides(shared, {
        "robot_id": "jackal1",
        "liorf.mapFrame": "jackal1/map",
        "liorf.mapFusionFrame": "map",
    })

    assert result["robot_id"] == "jackal1"
    assert result["liorf"]["mapFrame"] == "jackal1/map"
    assert result["liorf"]["mapFusionFrame"] == "map"
    assert result["liorf"]["sensor"] == "livox"
    assert "liorf.mapFrame" not in result
    assert shared["liorf"]["mapFrame"] == "map"


def test_parameter_files_deep_merge_in_declared_order(tmp_path):
    first = tmp_path / "first.yaml"
    second = tmp_path / "second.yaml"
    first.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    liorf:\n"
        "      mapFrame: map\n"
        "      sensor: livox\n",
        encoding="utf-8")
    second.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    liorf:\n"
        "      mapFrame: site/map\n"
        "      N_SCAN: 4\n",
        encoding="utf-8")

    result = MODULE._load_parameter_files([str(first), str(second)])

    assert result == {
        "liorf": {
            "mapFrame": "site/map",
            "sensor": "livox",
            "N_SCAN": 4,
        },
    }


def test_parameter_file_requires_wildcard_mapping(tmp_path):
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(
        "/**:\n  ros__parameters: not-a-mapping\n", encoding="utf-8")

    with pytest.raises(ValueError, match="must be a mapping"):
        MODULE._load_parameter_files([str(invalid)])


def test_default_pcm_directory_is_runtime_storage():
    default = MODULE._default_pcm_directory()

    assert Path(default).parent == Path(tempfile.gettempdir())
    assert default.endswith("skid_slam_pcm")
    assert str(REPO / "config") != default


@pytest.mark.parametrize("value", ["true", "TRUE", "1"])
def test_boolean_launch_values_accept_true_forms(value):
    assert MODULE._as_bool(value) is True


@pytest.mark.parametrize("value", ["false", "FALSE", "0", ""])
def test_boolean_launch_values_reject_other_forms(value):
    assert MODULE._as_bool(value) is False


@pytest.mark.parametrize("value", ["true", "TRUE", "1"])
def test_zmq_respawn_accepts_true_forms(value):
    assert ZMQ_MODULE._as_bool(value) is True


@pytest.mark.parametrize("value", ["false", "FALSE", "0", ""])
def test_zmq_respawn_rejects_other_forms(value):
    assert ZMQ_MODULE._as_bool(value) is False
