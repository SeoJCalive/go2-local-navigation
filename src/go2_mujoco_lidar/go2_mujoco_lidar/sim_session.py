"""ROS 실행 환경과 외부 MuJoCo process의 world를 시작 전에 검증한다.

launch는 이 함수들을 publisher 생성 전에 호출한다. world는 /proc의 실제 -s
argument에서 얻으며 상대 경로는 simulator cwd 기준이다. 선택한 model_path는
기대값으로만 쓰고 다른 경로는 거부한다. 반환 hash는 시작 시 파일의 provenance이며
simulator가 이미 읽은 model의 메모리 hash를 뜻하지 않는다.
"""

import hashlib
from pathlib import Path
from urllib.parse import unquote, urlparse
from xml.etree import ElementTree


def simulation_domain(environment, allow_unsafe=False):
    """실물 기본 domain과 loopback 밖 실행을 명시적 override 없이는 거부한다."""
    domain = int(environment.get("ROS_DOMAIN_ID", "0"))
    if not 0 <= domain <= 232:
        raise RuntimeError("simulation requires ROS_DOMAIN_ID in 0..232")
    localhost = environment.get("ROS_LOCALHOST_ONLY") == "1"
    # Preserve the existing Domain-1 runbook: it uses an explicit Cyclone lo XML
    # with ROS_LOCALHOST_ONLY=0. Read that file, never infer isolation from its name.
    if not localhost and environment.get("RMW_IMPLEMENTATION") == "rmw_cyclonedds_cpp":
        uri = urlparse(environment.get("CYCLONEDDS_URI", ""))
        if uri.scheme == "file" and uri.netloc in ("", "localhost"):
            document = ElementTree.parse(unquote(uri.path))
            domains = [
                item for item in document.findall(".//{*}Domain")
                if item.get("id", "any") in ("any", str(domain))
            ]
            interfaces = [
                item for section in domains for item in section.findall(".//{*}NetworkInterface")
            ]
            peers = [item for section in domains for item in section.findall(".//{*}Peer")]
            localhost = (
                bool(interfaces)
                and all(item.get("name") == "lo" for item in interfaces)
                and all(item.get("Address") in ("127.0.0.1", "::1") for item in peers)
            )
    if not allow_unsafe and (domain == 0 or not localhost):
        raise RuntimeError(
            "simulation refused: use non-zero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1 "
            "(or explicit CycloneDDS lo-only XML); allow_unsafe_simulation:=true overrides"
        )
    return domain


def simulator_world(pid, expected_world, domain, allow_unsafe=False, proc_root=Path("/proc")):
    """실행 중인 simulator CLI의 world/domain/interface를 확인해 경로와 hash를 돌려준다."""
    process = proc_root / str(int(pid))
    arguments = (process / "cmdline").read_bytes().rstrip(b"\0").decode().split("\0")

    def option(name):
        if arguments.count(name) != 1:
            raise RuntimeError(f"simulator PID {pid} must have exactly one {name} argument")
        return arguments[arguments.index(name) + 1]

    if int(option("-i")) != domain:
        raise RuntimeError("simulator DDS domain does not match ROS_DOMAIN_ID")
    if not allow_unsafe and option("-n") != "lo":
        raise RuntimeError("simulator must use -n lo")
    world = Path(option("-s"))
    if not world.is_absolute():
        world = (process / "cwd").resolve() / world
    world = world.resolve(strict=True)
    if expected_world and Path(expected_world).resolve(strict=True) != world:
        raise RuntimeError(
            f"world mismatch: simulator={world}, requested={Path(expected_world).resolve()}"
        )
    return world, hashlib.sha256(world.read_bytes()).hexdigest()
