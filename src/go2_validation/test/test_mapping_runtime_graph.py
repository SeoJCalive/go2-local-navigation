from types import SimpleNamespace


class _GraphNode:
    def get_node_names_and_namespaces(self) -> list[tuple[str, str]]:
        return [("slam_toolbox", "/"), ("observer", "/mapping")]

    def get_publishers_info_by_topic(self, topic_name: str) -> list[SimpleNamespace]:
        assert topic_name == "/tf"
        return [
            SimpleNamespace(node_name="slam_toolbox", node_namespace="/"),
            SimpleNamespace(node_name="go2_odometry_adapter", node_namespace="/"),
        ]


def test_given_mapping_graph_when_projected_then_local_tf_owner_is_excluded() -> None:
    # Given: the prior observer graph and QoS ownership surface.
    from go2_validation.mapping_runtime_graph import (
        RAW_ODOMETRY_QOS,
        global_tf_owner_nodes,
        node_paths,
    )

    # When: graph helpers inspect the same endpoint identities.
    node = _GraphNode()

    # Then: mapping owner and subscription depth behavior survive the extraction.
    assert node_paths(node) == {"/slam_toolbox", "/mapping/observer"}
    assert global_tf_owner_nodes(node) == {"/slam_toolbox"}
    assert RAW_ODOMETRY_QOS.depth == 1000
