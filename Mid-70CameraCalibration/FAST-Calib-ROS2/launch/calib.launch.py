import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Filter conflicting runtime library paths that can break PCL/libusb resolution.
    current_ld = os.environ.get('LD_LIBRARY_PATH', '')
    blocked_prefixes = ('/opt/MVS',)
    blocked_exact = {'/usr/local/lib'}
    filtered_entries = []
    for entry in current_ld.split(':'):
        if not entry:
            continue
        if entry in blocked_exact:
            continue
        if any(entry.startswith(prefix) for prefix in blocked_prefixes):
            continue
        filtered_entries.append(entry)

    ld_library_path_sanitized = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        ':'.join(filtered_entries)
    )

    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # Get package share directory
    pkg_share = FindPackageShare('fast_calib')
    
    # Load parameters
    params_file = PathJoinSubstitution([pkg_share, 'config', 'qr_params.yaml'])
    
    # Fast calibration node
    fast_calib_node = Node(
        package='fast_calib',
        executable='fast_calib',
        name='fast_calib',
        output='screen',
        parameters=[params_file]
    )
    
    # RViz node
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz_cfg', 'fast_livo2.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        ld_library_path_sanitized,
        rviz_arg,
        fast_calib_node,
        rviz_node
    ]) 	
