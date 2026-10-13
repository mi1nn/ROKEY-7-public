"""launch_ros parameter values that keep their type (imported by launch files only).

launch_ros reads a plain ``str`` parameter value as YAML, so text such as '0.1', 'true' or
an all-digit hash would reach a STRING parameter as DOUBLE/BOOL/INTEGER and the node would
refuse to start. Text and text lists are passed with an explicit type instead.
"""
from typing import List

from launch_ros.parameter_descriptions import ParameterValue


def typed(parameters):
    out = {}
    for name, value in parameters.items():
        if isinstance(value, str):
            value = ParameterValue(value, value_type=str)
        elif isinstance(value, list) and value and all(isinstance(item, str) for item in value):
            value = ParameterValue(value, value_type=List[str])
        out[name] = value
    return out
