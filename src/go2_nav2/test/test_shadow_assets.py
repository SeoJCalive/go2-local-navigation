from pathlib import Path
from shutil import copy2
from typing import Final
from xml.etree import ElementTree

import yaml


PACKAGE_ROOT: Final = Path(__file__).parents[1]
VALIDATION_ROOT: Final = PACKAGE_ROOT.parent / "go2_validation"
SCENARIOS_PATH: Final = VALIDATION_ROOT / "config" / "shadow_scenarios.yaml"
BT_PATH: Final = PACKAGE_ROOT / "behavior_trees" / "navigate_to_pose_shadow.xml"
EXPECTED_SCENARIOS: Final = frozenset(
    {
        "success",
        "cancel",
        "blocked_goal",
        "outside_map_goal",
        "planner_failure",
        "no_progress",
    }
)
EXPECTED_TERMINAL_STATUSES: Final = {
    "success": "SUCCEEDED",
    "cancel": "CANCELED",
    "blocked_goal": "ABORTED",
    "outside_map_goal": "ABORTED",
    "planner_failure": "ABORTED",
    "no_progress": "ABORTED",
}
EXPECTED_OBSERVABLES: Final = {
    "success": ("present", "present"),
    "cancel": ("present", "present"),
    "blocked_goal": ("absent", "absent"),
    "outside_map_goal": ("absent", "absent"),
    "planner_failure": ("absent", "absent"),
    "no_progress": ("present", "present"),
}


def load_yaml(path: Path) -> dict[str, object]:
    with path.open(encoding="utf-8") as yaml_file:
        document = yaml.safe_load(yaml_file)
    assert isinstance(document, dict)
    return document


def load_pgm(path: Path) -> tuple[int, int, int, tuple[int, ...]]:
    tokens = path.read_text(encoding="ascii").split()
    assert tokens[0] == "P2"
    width, height, maximum = (int(token) for token in tokens[1:4])
    pixels = tuple(int(token) for token in tokens[4:])
    assert len(pixels) == width * height
    assert all(0 <= pixel <= maximum for pixel in pixels)
    return width, height, maximum, pixels


def cell_value(
    pixels: tuple[int, ...], width: int, height: int, x: int, y: int
) -> int:
    assert 0 <= x < width
    assert 0 <= y < height
    return pixels[(height - 1 - y) * width + x]


def test_given_map_manifests_when_images_loaded_then_geometry_matches() -> None:
    scenarios = load_yaml(SCENARIOS_PATH)
    maps = scenarios["maps"]
    assert isinstance(maps, dict)

    for relative_map_path in maps.values():
        assert isinstance(relative_map_path, str)
        map_path = PACKAGE_ROOT / relative_map_path
        map_yaml = load_yaml(map_path)
        image_name = map_yaml["image"]
        assert isinstance(image_name, str)
        width, height, maximum, _ = load_pgm(map_path.parent / image_name)

        assert (width, height, maximum) == (12, 12, 255)
        assert map_yaml["resolution"] == 0.5
        assert map_yaml["origin"] == [-3.0, -3.0, 0.0]


def test_given_scenarios_when_checked_against_maps_then_cells_and_outcomes_match() -> None:
    manifest = load_yaml(SCENARIOS_PATH)
    maps = manifest["maps"]
    scenarios = manifest["scenarios"]
    assert isinstance(maps, dict)
    assert isinstance(scenarios, list)

    scenarios_by_id: dict[str, dict[str, object]] = {}
    for scenario in scenarios:
        assert isinstance(scenario, dict)
        scenario_id = scenario["id"]
        assert isinstance(scenario_id, str)
        assert scenario_id not in scenarios_by_id
        scenarios_by_id[scenario_id] = scenario

    assert scenarios_by_id.keys() == EXPECTED_SCENARIOS
    for scenario_id, scenario in scenarios_by_id.items():
        assert scenario["expected_action_terminal_status"] == (
            EXPECTED_TERMINAL_STATUSES[scenario_id]
        )
        assert isinstance(scenario["timeout_sec"], int)
        assert scenario["timeout_sec"] > 0
        assert (scenario["path_expectation"], scenario["candidate_expectation"]) == (
            EXPECTED_OBSERVABLES[scenario_id]
        )

        map_id = scenario["map_id"]
        assert isinstance(map_id, str)
        relative_map_path = maps[map_id]
        assert isinstance(relative_map_path, str)
        map_path = PACKAGE_ROOT / relative_map_path
        map_yaml = load_yaml(map_path)
        image_name = map_yaml["image"]
        assert isinstance(image_name, str)
        width, height, _, pixels = load_pgm(map_path.parent / image_name)

        start_cell = scenario["start_cell"]
        goal_cell = scenario["goal_cell"]
        assert isinstance(start_cell, dict)
        assert isinstance(goal_cell, dict)
        start_x, start_y = start_cell["x"], start_cell["y"]
        goal_x, goal_y = goal_cell["x"], goal_cell["y"]
        assert all(isinstance(value, int) for value in (start_x, start_y, goal_x, goal_y))
        assert 0 <= start_x < width and 0 <= start_y < height
        assert cell_value(pixels, width, height, start_x, start_y) == 254

        if scenario_id == "outside_map_goal":
            assert not (0 <= goal_x < width and 0 <= goal_y < height)
        else:
            assert 0 <= goal_x < width and 0 <= goal_y < height
            goal_value = cell_value(pixels, width, height, goal_x, goal_y)
            if scenario_id == "blocked_goal":
                assert goal_value == 0
            else:
                assert goal_value == 254


def test_given_shadow_bt_when_parsed_then_it_computes_and_follows_a_path() -> None:
    root = ElementTree.parse(BT_PATH).getroot()

    assert root.tag == "root"
    assert root.attrib["main_tree_to_execute"] == "NavigateToPoseShadow"
    tree = root.find("BehaviorTree")
    assert tree is not None
    assert tree.attrib["ID"] == "NavigateToPoseShadow"
    assert tree.find(".//ComputePathToPose") is not None
    assert tree.find(".//FollowPath") is not None


def test_given_asset_contract_when_placed_in_package_share_then_lookup_resolves(
    tmp_path: Path,
) -> None:
    package_share = tmp_path / "install" / "go2_nav2" / "share" / "go2_nav2"
    asset_paths = (
        Path("maps/shadow_open.yaml"),
        Path("maps/shadow_open.pgm"),
        Path("maps/shadow_blocked.yaml"),
        Path("maps/shadow_blocked.pgm"),
        Path("behavior_trees/navigate_to_pose_shadow.xml"),
    )

    for relative_path in asset_paths:
        target_path = package_share / relative_path
        target_path.parent.mkdir(parents=True, exist_ok=True)
        copy2(PACKAGE_ROOT / relative_path, target_path)

    validation_scenarios = package_share.parent / "go2_validation/config/shadow_scenarios.yaml"
    validation_scenarios.parent.mkdir(parents=True, exist_ok=True)
    copy2(SCENARIOS_PATH, validation_scenarios)

    assert all((package_share / relative_path).is_file() for relative_path in asset_paths)
    assert validation_scenarios.is_file()


def test_given_setup_metadata_when_inspected_then_maps_and_bt_are_installed() -> None:
    setup_text = (PACKAGE_ROOT / "setup.py").read_text()

    assert 'share/{PACKAGE_NAME}/maps' in setup_text
    assert 'share/{PACKAGE_NAME}/behavior_trees' in setup_text
