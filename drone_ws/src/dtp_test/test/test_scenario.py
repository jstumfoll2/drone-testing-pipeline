from pathlib import Path

import pytest

from dtp_test.scenario import ScenarioError, load_suite, scenario_from_dict

SCENARIO_DIR = Path(__file__).resolve().parents[1] / "scenarios"


def test_bundled_suite_loads():
    suite = load_suite(SCENARIO_DIR)
    assert suite, "expected at least one bundled scenario"
    smoke = [s for s in suite if "smoke" in s.tags]
    assert smoke, "the Gate 0 smoke scenario must be tagged 'smoke'"


def test_defaults_match_px4_aruco_world():
    s = scenario_from_dict({"name": "x"})
    assert s.px4_make_target == "gz_x500_mono_cam_down_aruco"
    assert s.gz_model_name == "x500_mono_cam_down_0"
    assert s.marker.length_m == 0.5 and s.marker.dictionary == "DICT_4X4_50"


def test_default_world_target_has_no_suffix():
    assert scenario_from_dict({"name": "x", "world": "default"}).px4_make_target == "gz_x500_mono_cam_down"


@pytest.mark.parametrize(
    "bad",
    [
        {},
        {"name": "x", "vehicle": "f450"},
        {"name": "x", "world": "my_world"},
        {"name": "x", "marker": {"position": [0, 0]}},
        {"name": "x", "marker": {"length_m": 0}},
        {"name": "x", "wind_mps": -1},
        {"name": "x", "thresholds": {"min_detection_rate": 1.5}},
        {"name": "x", "colour": "red"},
    ],
)
def test_invalid_scenarios_rejected(bad):
    with pytest.raises(ScenarioError):
        scenario_from_dict(bad)


def test_custom_world_allowed_when_flagged():
    s = scenario_from_dict({"name": "x", "world": "warehouse", "custom_world": True})
    assert s.world == "warehouse"


def test_duplicate_names_rejected(tmp_path):
    for i in range(2):
        (tmp_path / f"s{i}.yaml").write_text("name: same\n")
    with pytest.raises(ScenarioError):
        load_suite(tmp_path)
