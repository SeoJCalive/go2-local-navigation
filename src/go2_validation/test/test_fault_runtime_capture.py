import ast
import json
from pathlib import Path

from go2_validation.fault_runtime_capture import (
    StreamStampObservation,
    correlate_fixture_events,
    parse_fixture_event,
)


def test_given_fixture_markers_and_output_stamps_when_correlated_then_counts_are_observed() -> None:
    # Given: three fixture boundaries and outputs missing only at the fault stamp.
    markers = tuple(
        parse_fixture_event(
            json.dumps(
                {
                    "phase": phase,
                    "clock_nanoseconds": stamp,
                    "reason_code": reason,
                    "child_exit_code": None,
                }
            )
        )
        for phase, stamp, reason in (
            ("baseline", 10, None),
            ("suppressed", 20, "EMPTY_CLOUD"),
            ("recovered", 30, None),
        )
    )
    streams = StreamStampObservation(
        validated_cloud=(10, 30),
        scan=(10, 30),
        odom=(10, 20, 30),
        tf=(10, 20, 30),
    )

    # When: runtime output stamps are joined to their fixture boundaries.
    events = correlate_fixture_events(markers, streams)

    # Then: suppression is based on observed output, not fixture-declared counts.
    assert events[1].output_counts.captured_streams() == ("odom", "tf")
    assert events[2].output_counts.captured_streams() == (
        "validated_cloud",
        "scan",
        "odom",
        "tf",
    )


def test_given_recovery_second_sample_when_correlated_then_one_nanosecond_is_same_boundary() -> None:
    # Given: odometry recovery becomes publishable on the second consecutive sample.
    marker = parse_fixture_event(
        '{"phase":"recovered","clock_nanoseconds":30,'
        '"reason_code":null,"child_exit_code":null}'
    )
    streams = StreamStampObservation((), (), (31,), (31,))

    # When: the capture is correlated.
    event = correlate_fixture_events((marker,), streams)[0]

    # Then: the one-nanosecond continuity sample belongs to recovery.
    assert event.output_counts.odom == 1
    assert event.output_counts.tf == 1


def test_given_rclpy_node_subclasses_when_loaded_then_subscription_registry_is_not_overwritten() -> None:
    # Given: both runtime observers inherit rclpy.node.Node, which owns `_subscriptions`.
    package_root = Path(__file__).parents[1]
    observer_paths = (
        package_root / "go2_validation/fault_runtime_observer.py",
        package_root / "go2_validation/mapping_input_observer.py",
    )

    # When: assignments to self attributes are inspected structurally.
    assigned_attributes = {
        path.name: {
            target.attr
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8")))
            if isinstance(node, (ast.Assign, ast.AnnAssign))
            for target in (
                (*node.targets,) if isinstance(node, ast.Assign) else (node.target,)
            )
            if isinstance(target, ast.Attribute)
            and isinstance(target.value, ast.Name)
            and target.value.id == "self"
        }
        for path in observer_paths
    }

    # Then: neither subclass shadows the mutable registry used by destroy_node().
    assert all(
        "_subscriptions" not in attributes
        for attributes in assigned_attributes.values()
    ), assigned_attributes
