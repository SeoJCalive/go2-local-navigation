"""활성 domain·URI·실제 interface를 공유 parser로 검증한다."""

import pytest

from description.dds_environment import cyclonedds_network_matches


def config(interface="lo", domain="any", peer="127.0.0.1"):
    return (
        f'<CycloneDDS><Domain id="{domain}"><General><Interfaces>'
        f'<NetworkInterface name="{interface}" multicast="false"/>'
        '</Interfaces></General><Discovery><Peers>'
        f'<Peer Address="{peer}"/></Peers></Discovery></Domain></CycloneDDS>'
    )


def test_inline_and_file_use_identical_network_rules(tmp_path):
    path = tmp_path / "settings with spaces.xml"
    path.write_text(config())
    for uri in (config(), path.as_uri()):
        assert cyclonedds_network_matches(uri, "lo", 65, require_multicast_disabled=True)
        assert not cyclonedds_network_matches(uri, "eno1", 65)


@pytest.mark.parametrize("xml", [
    config("eno1"), config(domain="1"), config(peer="192.168.123.1"),
    config() + "<broken", '<NetworkInterface name="lo"/>',
    config().replace("</Interfaces>", '<NetworkInterface name="eno1"/></Interfaces>'),
])
def test_misleading_or_inactive_settings_are_refused(tmp_path, xml):
    path = tmp_path / "lo-only.xml"
    path.write_text(xml)
    assert not cyclonedds_network_matches(path.as_uri(), "lo", 65)


def test_bad_uri_and_multicast_are_refused(tmp_path):
    assert not cyclonedds_network_matches((tmp_path / "missing.xml").as_uri(), "lo", 65)
    assert not cyclonedds_network_matches("https://localhost/config.xml", "lo", 65)
    xml = config().replace('multicast="false"', 'multicast="true"')
    assert cyclonedds_network_matches(xml, "lo", 65)
    assert not cyclonedds_network_matches(xml, "lo", 65, require_multicast_disabled=True)
