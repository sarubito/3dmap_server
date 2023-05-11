from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    # filename = LaunchConfiguration('filename', default = 'map.pcd')
    # filename_argument = DeclareLaunchArgument('filename', default_value="map.pcd")
    mapserver_path = get_package_share_directory('3dmap_server')
    pcdfile_path = os.path.join(mapserver_path, 'map/'+'map.pcd')
    
    return LaunchDescription([
        Node(
            package='3dmap_server',
            executable='map_server',
            output='screen',
            parameters=[{
                'pcdfile_path' : pcdfile_path,
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output = 'screen',
        )
    ])