from importlib import import_module
from typing import Type


def resolve_msg_type(type_str: str) -> Type:
    """Convert e.g. 'sensor_msgs/msg/Image' -> sensor_msgs.msg.Image class."""
    pkg, submod, cls_name = type_str.split('/')
    module = import_module(f'{pkg}.{submod}')  # e.g. sensor_msgs.msg
    return getattr(module, cls_name)
