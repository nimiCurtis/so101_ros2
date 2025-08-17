# MIT License
#
# Copyright (c) 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    so101_description_dir = get_package_share_directory("so101_description")

    model = os.path.join(so101_description_dir, "urdf", "so101_new_calib.urdf.xacro")
    use_joint_states_gui = "true"
    mode = "real"

    display_config_arg = DeclareLaunchArgument(
        "display_config",
        default_value=os.path.join(so101_description_dir, "rviz", "display.rviz"),
        description="RViz config file path",
    )

    # Launch configurations
    display_config = LaunchConfiguration("display_config")

    # Include rsp.launch.py
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(so101_description_dir, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "model": model,
            "mode": mode,
            "joint_states_gui": use_joint_states_gui,
            "type": "follower",
            "use_sim": "false",
        }.items(),
    )

    # Include display.launch.py
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(so101_description_dir, "launch", "display.launch.py")
        ),
        launch_arguments={
            "joint_states_gui": use_joint_states_gui,
            "display_config": display_config,
        }.items(),
    )

    return LaunchDescription(
        [
            display_config_arg,
            rsp_launch,
            display_launch,
        ]
    )
