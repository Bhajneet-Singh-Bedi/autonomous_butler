from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    SetEnvironmentVariable, 
    IncludeLaunchDescription, 
    RegisterEventHandler,
    ExecuteProcess
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    PathJoinSubstitution, 
    LaunchConfiguration, 
    TextSubstitution
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_omni_wheel_bot = get_package_share_directory('autonomous_butler')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # urdf_file = os.path.join(get_package_share_directory('autonomous_butler'), 'urdf', 'butler.urdf')  
    urdf_file = os.path.join(get_package_share_directory('autonomous_butler'), 'model', 'model.sdf')
    assert os.path.exists(urdf_file), "The butler.urdf doesnt exist in "+str(urdf_file)  
    urdf = open(urdf_file).read()




    DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),


    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ' -v4  /home/bhajneet/ros2_ws/src/autonomous_butler/worlds/gazebo.sdf'
        }.items(),
    )


    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_sdf_entity",
        output="screen",
        arguments=[
            "-file", urdf_file,   # Path to the SDF file
            "-name", "robot_name",    # Name of the robot to be spawned
            "-x", "0.0",                # X position
            "-y", "0.0",                # Y position
            "-z", "0.5"                 # Z position
        ],
    )
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   'lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        parameters=[{'qos_overrides./model/vehicle_blue.subscriber.reliability': 'reliable'}],
        output='screen'
    )                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    return LaunchDescription([
        gz_sim,
        spawn_entity,
        bridge
    ])