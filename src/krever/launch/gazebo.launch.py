import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('krever')
    
    # Set paths to URDF and world file
    urdf_file = os.path.join(pkg_share, 'urdf', 'krever.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    # Launch Ignition Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments={'ign_args': world_file}.items()
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

    # Spawn the robot
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

    # Bridge to convert Ignition topics to ROS topics
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
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge
    ])