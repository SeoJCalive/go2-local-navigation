from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_given_humble_async_slam_when_client_contract_is_read_then_lifecycle_is_not_required() -> None:
    # Given: 실제 Humble async_slam_toolbox_node와 연결되는 service client source
    source = (
        PACKAGE_ROOT / "go2_validation/mapping_slam_services.py"
    ).read_text(encoding="utf-8")

    # When/Then: 저장 service 세 개만 필수이며 lifecycle GetState는 요구하지 않는다.
    assert '"/slam_toolbox/save_map"' in source
    assert '"/slam_toolbox/serialize_map"' in source
    assert '"/slam_toolbox/deserialize_map"' in source
    assert "GetState" not in source
    assert '"/slam_toolbox/get_state"' not in source
