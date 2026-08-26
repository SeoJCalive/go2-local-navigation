"""통합 preflight의 command publisher owner 판정 경계를 검증한다."""

from dataclasses import dataclass

from bringup.preflight_assessments import project_publisher_count
from bringup.preflight_configuration import EXPECTED_NODES


@dataclass(frozen=True, slots=True)
class EndpointFixture:
    node_name: str
    node_namespace: str


def test_command_publisher_count_ignores_existing_bare_dds_endpoints() -> None:
    # Given: 기존 bare DDS publisher와 프로젝트 motion adapter publisher
    endpoints = (
        EndpointFixture(
            node_name="_CREATED_BY_BARE_DDS_APP_",
            node_namespace="_CREATED_BY_BARE_DDS_APP_",
        ),
        EndpointFixture(node_name="go2_motion_adapter", node_namespace="/"),
    )

    # When: 통합 stack이 소유한 publisher만 계산한다.
    count = project_publisher_count(endpoints, EXPECTED_NODES)

    # Then: bare DDS endpoint는 제외하고 프로젝트 owner 하나만 남는다.
    assert count == 1
