"""설치 launch를 실행하지 않고 실제 context에서 평가하는 공통 시험 helper.

DeclareLaunchArgument, 조건, OpaqueFunction과 Include를 평가해 Node를 수집한다.
ROS child는 시작하지 않는다. parameters는 Humble launch_ros의 normalize/evaluate
경계를 사용한다. Node의 normalized parameter 저장소만 private API를 읽는다.
"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def load(package, filename):
    path = Path(get_package_share_directory(package)) / "launch" / filename
    spec = importlib.util.spec_from_file_location("tested_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def evaluate(package, filename, overrides=None):
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    nodes, includes = [], []

    def resolve(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    def visit(actions):
        for action in actions:
            if action.condition is not None and not action.condition.evaluate(context):
                continue
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
            elif isinstance(action, OpaqueFunction):
                visit(action.execute(context) or [])
            elif isinstance(action, Node):
                nodes.append({
                    "package": resolve(action.node_package),
                    "executable": resolve(action.node_executable),
                    "parameters": evaluate_parameters(context, action._Node__parameters),
                    "action": action,
                })
            elif isinstance(action, IncludeLaunchDescription):
                action.launch_description_source.get_launch_description(context)
                includes.append({
                    "path": resolve(action.launch_description_source.location),
                    "arguments": {name: resolve(value)
                                  for name, value in action.launch_arguments},
                })
    visit(load(package, filename).generate_launch_description().entities)
    return context, nodes, includes
