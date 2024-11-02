import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Enable software rendering
    software_rendering = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    
    pkg_share = get_package_share_directory('krever')
    urdf_file = os.path.join(pkg_share, 'urdf', 'krever.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments={'ign_args': world_file}.items()
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': True
        }]
    )

    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', 'krever',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/krever/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/krever/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    return LaunchDescription([
        software_rendering,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge
    ])
