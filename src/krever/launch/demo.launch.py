import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
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
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments={'ign_args': f'-r {world_file}'}.items()
    )
    
    # Robot state publisher
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

    # Spawn robot in Gazebo
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

    # Bridge topics for cmd_vel, odometry, and LiDAR
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/krever/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/krever/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    # SLAM node
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Teleop twist keyboard node for manual control
    teleop_twist_keyboard = ExecuteProcess(
        cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen',
        prefix='xterm -e'  # Opens in a new terminal for interaction
    )

    return LaunchDescription([
        software_rendering,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        slam_toolbox,
        teleop_twist_keyboard
    ])
