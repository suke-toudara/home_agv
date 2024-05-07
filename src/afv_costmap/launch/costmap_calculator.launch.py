# Copyright (c) 2020 OUXT Polaris
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    grid_map_demos_dir = get_package_share_directory('afv_costmap')
    
    #cost_map_calc
    filters_config = DeclareLaunchArgument(
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'nav2_costmap.yaml'),
            description='Full path to the filter chain config file to use')

    grid_map_filter_node = Node(
        package='afv_costmap',
        executable='costmap_filter_node',
        name='costmap_filter_node',
        output='screen',
        parameters=[filters_config]
    )

    costmap_calculator_node = Node(
        package='afv_costmap',
        executable='costmap_calculator_node',
        name='costmap_calculator_node',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(grid_map_filter_node)
    ld.add_action(costmap_calculator_node)
    return ld