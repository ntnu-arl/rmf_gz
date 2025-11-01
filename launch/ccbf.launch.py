from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package="rmf_gz",
            executable="pd_controller.py",
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output="screen",
            remappings=[
                ('~/odom', '/rmf/odom'),
                # ('~/cmd', '/rmf/cmd/acc'),
            ],
        ),
        Node(
            package='cbf_pc_selector',
            executable='pc_selector_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('cbf_pc_selector'),
                    'config', 'sim.yaml'
                ),
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        Node(
            package='composite_cbf',
            executable='composite_cbf_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('composite_cbf'),
                    'config', 'sim.yaml'
                ),
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('~/obstacles', '/cbf_pc_selector/output_pc'),
                ('~/cmd_in', '/pd_controller/cmd'),
                ('~/odom', '/rmf/odom'),
                ('~/safe_cmd_twist', '/rmf/cmd/acc'),
            ],
            output='screen'
        ),
    ])
