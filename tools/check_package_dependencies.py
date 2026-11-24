"""Check workspace dependency declarations and cycles without importing ROS.

Read package.xml, Python AST and CMake under src/optional/tools. Literal imports,
package lookups, launch Node/include and ros2 command lists must have direct
declarations. Test references may use test_depend. Dynamic package selection is
reported separately; installed launch lookup is also exercised by W04 builds.
Exit nonzero on undeclared workspace references or a dependency cycle.
"""

import ast
from graphlib import TopologicalSorter, CycleError
from pathlib import Path
import re
import sys
from xml.etree import ElementTree


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = {"depend", "exec_depend", "build_depend", "buildtool_depend",
           "build_export_depend", "buildtool_export_depend"}


def main():
    packages = {}
    for base in ("src", "optional", "tools"):
        for manifest in (ROOT / base).rglob("package.xml"):
            document = ElementTree.parse(manifest).getroot()
            name = document.findtext("name")
            assert name not in packages, f"duplicate package: {name}"
            packages[name] = (manifest.parent, document)
    graph = {}
    errors = []
    dynamic = []
    for name, (directory, document) in sorted(packages.items()):
        runtime = {item.text for item in document if item.tag in RUNTIME}
        tests = runtime | {item.text for item in document.findall("test_depend")}
        graph[name] = (tests & packages.keys()) - {name}
        print(f"{name} -> {', '.join(sorted(graph[name])) or '(external only)'}")
        for path in directory.rglob("*.py"):
            if "__pycache__" in path.parts:
                continue
            allowed = tests if "test" in path.relative_to(directory).parts else runtime
            references = set()
            tree = ast.parse(path.read_text(), filename=str(path))
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom) and node.module:
                    references.add((node.module.split(".")[0], node.lineno))
                elif isinstance(node, ast.Import):
                    references.update((alias.name.split(".")[0], node.lineno)
                                      for alias in node.names)
                elif isinstance(node, ast.Call):
                    function = node.func
                    symbol = function.id if isinstance(function, ast.Name) else (
                        function.attr if isinstance(function, ast.Attribute) else "")
                    values = []
                    if symbol in ("get_package_share_directory", "get_package_prefix",
                                  "_include", "include") and node.args:
                        values.append(node.args[0])
                    if symbol == "Node":
                        values.extend(keyword.value for keyword in node.keywords
                                      if keyword.arg == "package")
                    for value in values:
                        if isinstance(value, ast.Constant) and isinstance(value.value, str):
                            references.add((value.value, node.lineno))
                        else:
                            dynamic.append(f"{path.relative_to(ROOT)}:{node.lineno}")
                elif isinstance(node, (ast.List, ast.Tuple)):
                    values = [element.value if isinstance(element, ast.Constant) else None
                              for element in node.elts]
                    for index in range(len(values) - 2):
                        if values[index:index + 2] in (["ros2", "run"], ["ros2", "launch"]):
                            if isinstance(values[index + 2], str):
                                references.add((values[index + 2], node.lineno))
            for target, line in sorted(references):
                if target in packages and target != name and target not in allowed:
                    errors.append(f"{path.relative_to(ROOT)}:{line}: undeclared {target}")
        for path in directory.rglob("CMakeLists.txt"):
            for target in re.findall(r"find_package\(\s*(\w+)", path.read_text()):
                if target in packages and target != name and target not in runtime:
                    errors.append(f"{path.relative_to(ROOT)}: undeclared CMake {target}")
    try:
        order = tuple(TopologicalSorter(graph).static_order())
        print("ORDER " + " ".join(order))
    except CycleError as error:
        errors.append(f"CYCLE {error.args[1]}")
    for error in errors:
        print("ERROR " + error)
    print(f"PACKAGES {len(packages)} ERRORS {len(errors)} "
          f"DYNAMIC_REFERENCES {len(dynamic)}")
    return bool(errors)


if __name__ == "__main__":
    sys.exit(main())
