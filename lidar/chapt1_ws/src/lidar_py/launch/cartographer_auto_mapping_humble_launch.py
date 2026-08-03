"""Ubuntu 22.04 / ROS 2 Humble entry point for the unified stack."""

import importlib.util
import os


_IMPL = os.path.join(
    os.path.dirname(__file__), "cartographer_auto_mapping_jazzy_launch.py")
_SPEC = importlib.util.spec_from_file_location(
    "lidar_py_cartographer_auto_mapping_impl", _IMPL)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


def generate_launch_description():
    return _MODULE.generate_launch_description()
