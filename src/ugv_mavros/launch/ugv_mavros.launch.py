from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')

    mavros_share = get_package_share_directory('mavros')
    apm_config = os.path.join(mavros_share, 'launch', 'apm_config.yaml')
    apm_plugins = os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:57600',
            description='Serial port for Pixhawk'
        ),
        Node(
            package='mavros',
            executable='mavros_node',
            # name='mavros',
            output='screen',
            parameters=[
                apm_config,
                apm_plugins,
                {'fcu_url': fcu_url},
                {'gcs_url': ''},
                {'target_system_id': 1},
                {'target_component_id': 1}
            ]
        )
    ])



# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     fcu_url = LaunchConfiguration('fcu_url')

#     # Get the package directories
#     mavros_share = get_package_share_directory('mavros')
#     ugv_pkg_share = get_package_share_directory('ugv_mavros')  # Replace with your package name if different

#     # YAML config paths
#     apm_plugins = os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml')
#     apm_config = os.path.join(ugv_pkg_share, 'config', 'apm_config.yaml')

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'fcu_url',
#             default_value='/dev/ttyACM0:57600',
#             description='Pixhawk serial port and baudrate'
#         ),

#         Node(
#             package='mavros',
#             executable='mavros_node',
#             namespace='mavros',
#             # name='mavros_node',
#             output='screen',
#             parameters=[
#                 apm_plugins,
#                 apm_config,
#                 {'fcu_url': fcu_url},
#                 {'system_id': 255},
#                 {'component_id': 240},
#                 {'timesync_mode': 'AUTO'}
#             ]
#         ),

#         # IMU config parameters
#         Node(
#             package='rclcpp_components',
#             executable='component_container_mt',
#             name='imu_params_loader',
#             parameters=[
#                 {'imu.enable_filter': True},
#                 {'imu.enable_transform': False}
#             ],
#             output='screen'
#         ),
#     ])
