
"""Python 3.10에서 exhaustive match를 표현하는 최소 typing helper다."""
from typing import NoReturn


def assert_never(value: NoReturn) -> NoReturn:
    """정적 도달 불가능 variant가 runtime에 들어오면 즉시 실패한다."""
    raise AssertionError(f"unhandled variant: {value!r}")
