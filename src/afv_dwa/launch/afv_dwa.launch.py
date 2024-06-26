import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    container = ComposableNodeContainer(
            name='dwa_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='afv_dwa',
                    plugin='afv_dwa::ObstacleDetector',
                    name='obstacle_detector_node',
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='afv_dwa',
                    plugin='afv_dwa::DWA',
                    name='dwa_node',
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
    )
    # rviz2 = Node(package='rviz2',
    #                 executable='rviz2',
    #                 name='rviz2',
    #                 arguments=[
    #                     '-d',
    #                     get_package_share_directory('afv_dwa')
    #                         + '/dwa.rviz'],
    #                 output='screen'
    #                 )
    return launch.LaunchDescription([container])#, rviz2])
