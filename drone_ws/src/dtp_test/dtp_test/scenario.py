"""Scenario definitions for the test harness.

A scenario is a YAML file describing one test run: which PX4 vehicle model and
Gazebo world to use, where the marker is, what disturbances to apply, and the
pass/fail thresholds. Scenarios live in ``dtp_test/scenarios/``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

# PX4 v1.17 Gazebo vehicles and worlds we use (see PX4 docs: sim_gazebo_gz).
KNOWN_VEHICLES = {
    "x500", "x500_depth", "x500_vision", "x500_mono_cam", "x500_mono_cam_down",
    "x500_lidar_down", "x500_lidar_2d", "x500_lidar_front", "x500_gimbal",
}
KNOWN_WORLDS = {
    "default", "aruco", "baylands", "lawn", "rover", "ridge", "walls", "windy",
    "moving_platform",
}

Vec3 = Tuple[float, float, float]


class ScenarioError(ValueError):
    """Raised when a scenario file is invalid."""


@dataclass(frozen=True)
class Marker:
    position: Vec3 = (0.0, 0.0, 0.0)  # world ENU, metres
    length_m: float = 0.5
    marker_id: int = 0
    dictionary: str = "DICT_4X4_50"


@dataclass(frozen=True)
class Thresholds:
    max_landing_error_m: float = 0.05  # paper: within 5 cm indoors
    timeout_s: float = 120.0
    min_detection_rate: float = 0.0  # 0 disables the check
    max_latency_ms: Optional[float] = None


@dataclass(frozen=True)
class Scenario:
    name: str
    vehicle: str = "x500_mono_cam_down"
    world: str = "aruco"
    marker: Marker = field(default_factory=Marker)
    takeoff_altitude_m: float = 5.0
    wind_mps: float = 0.0
    camera_noise_std: float = 0.0
    tags: Tuple[str, ...] = ()
    thresholds: Thresholds = field(default_factory=Thresholds)
    custom_world: bool = False  # True for worlds we generate rather than PX4's built-ins

    @property
    def px4_make_target(self) -> str:
        """The `make px4_sitl <target>` name, e.g. gz_x500_mono_cam_down_aruco."""
        if self.world == "default":
            return f"gz_{self.vehicle}"
        return f"gz_{self.vehicle}_{self.world}"

    @property
    def gz_model_name(self) -> str:
        """Name PX4 gives the spawned model (first instance)."""
        return f"{self.vehicle}_0"


def _vec3(value: Any, what: str) -> Vec3:
    if not (isinstance(value, (list, tuple)) and len(value) == 3):
        raise ScenarioError(f"{what} must be a list of 3 numbers, got {value!r}")
    try:
        return tuple(float(v) for v in value)  # type: ignore[return-value]
    except (TypeError, ValueError) as exc:
        raise ScenarioError(f"{what} must be numeric, got {value!r}") from exc


def _positive(value: Any, what: str, allow_zero: bool = False) -> float:
    v = float(value)
    if v < 0 or (v == 0 and not allow_zero):
        raise ScenarioError(f"{what} must be {'>= 0' if allow_zero else '> 0'}, got {v}")
    return v


def scenario_from_dict(data: Dict[str, Any]) -> Scenario:
    if not isinstance(data, dict):
        raise ScenarioError("scenario must be a mapping")
    allowed = {"name", "vehicle", "world", "marker", "takeoff_altitude_m", "wind_mps",
               "camera_noise_std", "tags", "thresholds", "custom_world"}
    unknown = set(data) - allowed
    if unknown:
        raise ScenarioError(f"unknown keys: {sorted(unknown)}")
    if not data.get("name"):
        raise ScenarioError("scenario needs a name")

    vehicle = data.get("vehicle", "x500_mono_cam_down")
    if vehicle not in KNOWN_VEHICLES:
        raise ScenarioError(f"unknown vehicle {vehicle!r}; known: {sorted(KNOWN_VEHICLES)}")
    custom_world = bool(data.get("custom_world", False))
    world = data.get("world", "aruco")
    if not custom_world and world not in KNOWN_WORLDS:
        raise ScenarioError(f"unknown world {world!r}; set custom_world: true for generated worlds")

    m = data.get("marker", {}) or {}
    marker = Marker(
        position=_vec3(m.get("position", [0, 0, 0]), "marker.position"),
        length_m=_positive(m.get("length_m", 0.5), "marker.length_m"),
        marker_id=int(m.get("marker_id", 0)),
        dictionary=str(m.get("dictionary", "DICT_4X4_50")),
    )
    t = data.get("thresholds", {}) or {}
    latency = t.get("max_latency_ms")
    thresholds = Thresholds(
        max_landing_error_m=_positive(t.get("max_landing_error_m", 0.05), "max_landing_error_m"),
        timeout_s=_positive(t.get("timeout_s", 120.0), "timeout_s"),
        min_detection_rate=float(t.get("min_detection_rate", 0.0)),
        max_latency_ms=None if latency is None else _positive(latency, "max_latency_ms"),
    )
    if not 0.0 <= thresholds.min_detection_rate <= 1.0:
        raise ScenarioError("min_detection_rate must be between 0 and 1")

    return Scenario(
        name=str(data["name"]),
        vehicle=vehicle,
        world=world,
        marker=marker,
        takeoff_altitude_m=_positive(data.get("takeoff_altitude_m", 5.0), "takeoff_altitude_m"),
        wind_mps=_positive(data.get("wind_mps", 0.0), "wind_mps", allow_zero=True),
        camera_noise_std=_positive(data.get("camera_noise_std", 0.0), "camera_noise_std",
                                   allow_zero=True),
        tags=tuple(str(x) for x in data.get("tags", []) or []),
        thresholds=thresholds,
        custom_world=custom_world,
    )


def load_scenario(path: str | Path) -> Scenario:
    with open(path, "r", encoding="utf-8") as f:
        return scenario_from_dict(yaml.safe_load(f))


def load_suite(directory: str | Path, tag: Optional[str] = None) -> List[Scenario]:
    """Load every ``*.yaml`` scenario in a directory, optionally filtered by tag."""
    scenarios = [load_scenario(p) for p in sorted(Path(directory).glob("*.yaml"))]
    names = [s.name for s in scenarios]
    dupes = {n for n in names if names.count(n) > 1}
    if dupes:
        raise ScenarioError(f"duplicate scenario names: {sorted(dupes)}")
    if tag is not None:
        scenarios = [s for s in scenarios if tag in s.tags]
    return scenarios
