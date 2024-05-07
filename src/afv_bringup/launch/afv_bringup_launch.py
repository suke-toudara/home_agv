from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

import os
from glob import glob

def generate_launch_description():
    ld = LaunchDescription()
    
    # #
    # behavior_planner = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([
    #                 PathJoinSubstitution([
    #                     FindPackageShare('behavior_planner'),
    #                     'behavior_planner_launch.py'
    #                 ])
    #             ]),
    # )

    #rf2o_laser_odometry
    rf2o_laser_odometry = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('rf2o_laser_odometry'),
                        "launch/",
                        'rf2o_laser_odometry.launch.py'
                    ])
                ]),
    )
    #dwa_cpp_ros2
    dwa_cpp_ros2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('afv_dwa'),
                        "launch/",
                        'afv_dwa.launch.py'
                    ])
                ]),
    )

    afv_simulator = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('afv_simulator'),
                        'afv_simple_simulator_bringup.launch.py'
                    ])
                ]),
    )

    ydlidar_ros2_driver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ydlidar_ros2_driver'),
                        "launch/",
                        'ydlidar_launch.py'
                    ])
                ]),
    )

    #ld.add_action(behavior_planner)
    #ld.add_action(rf2o_laser_odometry)
    ld.add_action(dwa_cpp_ros2)
    ld.add_action(afv_simulator)
    ld.add_action(ydlidar_ros2_driver)
    return ld