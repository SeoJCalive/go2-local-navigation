"""Verify W04 checker rejects undeclared imports and dependency cycles."""

import importlib.util
from pathlib import Path


def checker(monkeypatch, tmp_path):
    path = Path(__file__).parents[2] / "check_package_dependencies.py"
    spec = importlib.util.spec_from_file_location("package_dependency_check", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "ROOT", tmp_path)
    return module


def package(root, name, dependencies=(), source=""):
    directory = root / "src" / name
    directory.mkdir(parents=True)
    declarations = "".join(f"<exec_depend>{value}</exec_depend>" for value in dependencies)
    (directory / "package.xml").write_text(
        f"<package><name>{name}</name>{declarations}</package>")
    (directory / "node.py").write_text(source)


def test_declared_runtime_reference_passes(monkeypatch, tmp_path):
    package(tmp_path, "producer")
    package(tmp_path, "consumer", ("producer",), "import producer\n")
    assert not checker(monkeypatch, tmp_path).main()


def test_missing_runtime_reference_fails(monkeypatch, tmp_path):
    package(tmp_path, "producer")
    package(tmp_path, "consumer", source="from producer import node\n")
    assert checker(monkeypatch, tmp_path).main()


def test_declared_cycle_fails(monkeypatch, tmp_path):
    package(tmp_path, "producer", ("consumer",))
    package(tmp_path, "consumer", ("producer",))
    assert checker(monkeypatch, tmp_path).main()
