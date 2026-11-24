"""Exercise installed core imports and launch assets in a clean dependency prefix.

Argument is the package selected by colcon --packages-up-to. No ROS node is
started: launch descriptions are constructed, and opaque runtime actions remain
unevaluated. Installed URDF/config/launch lookup must resolve in that prefix.
"""

import importlib
import importlib.util
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory


modules = {
    "description": ("description.dds_environment", "description.static_tf_profiles"),
    "go2_inputs": ("go2_inputs.mapping_cloud_gate_node", "go2_inputs.odometry_adapter_node"),
    "go2_nav2": ("go2_nav2.sim_session", "go2_nav2.sim_session_guard"),
    "go2_mujoco_lidar": ("go2_mujoco_lidar.sim_session", "go2_mujoco_lidar.sim_save_map"),
}
for package in list(modules)[:list(modules).index(sys.argv[1]) + 1]:
    share = Path(get_package_share_directory(package))
    for module in modules[package]:
        importlib.import_module(module)
    for path in sorted((share / "launch").glob("*.py")):
        spec = importlib.util.spec_from_file_location(f"installed_{path.stem}", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.generate_launch_description()
    assert (share / "package.xml").is_file()
    print(f"INSTALLED_CORE_OK {package} {share}")
