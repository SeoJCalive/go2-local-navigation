"""
실행 domain에 적용되는 CycloneDDS XML의 실제 network 경계를 읽는다.

입력은 inline XML 또는 local file URI다. 활성 Domain의 모든 interface와 peer를
검사하며 잘못된 XML·파일·domain은 False다. 파일명이나 XML 문자열 조각은 근거로 쓰지 않는다.
ROS_LOCALHOST_ONLY 판단은 호출자가 맡는다; 이 함수는 명시 DDS 설정만 검사한다.
"""

from pathlib import Path
from urllib.parse import unquote, urlparse
from xml.etree import ElementTree


def cyclonedds_network_matches(
    uri: str, interface: str, domain_id: int, *, require_multicast_disabled: bool = False,
) -> bool:
    """선택 domain의 interface/peer가 요청한 network에만 속하는지 반환한다."""
    try:
        if uri.lstrip().startswith("<"):
            document = ElementTree.fromstring(uri)
        else:
            location = urlparse(uri)
            if location.scheme != "file" or location.netloc not in ("", "localhost"):
                return False
            document = ElementTree.fromstring(Path(unquote(location.path)).read_text())
    except (OSError, UnicodeError, ElementTree.ParseError):
        return False
    domains = [
        item for item in document.findall(".//{*}Domain")
        if item.get("id", "any") in ("any", str(domain_id))
    ]
    interfaces = [item for domain in domains for item in domain.findall(".//{*}NetworkInterface")]
    if not interfaces or any(item.get("name") != interface for item in interfaces):
        return False
    if interface == "lo" and any(
        item.get("Address") not in ("127.0.0.1", "::1")
        for domain in domains for item in domain.findall(".//{*}Peer")
    ):
        return False
    if require_multicast_disabled and any(
        item.get("multicast") != "false" for item in interfaces
    ):
        return False
    return True
