import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('uav_ball_drop')
    
    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'ball_drop_test_world.sdf')
    models_path = os.path.join(pkg_share, 'models')
    
    # Environment variable to discover custom Gazebo models
    current_gz_res = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = models_path if not current_gz_res else f"{models_path}:{current_gz_res}"
    
    set_gz_resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )
    
    # Include official ros_gz_sim launch description to launch Gazebo Harmonic
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f"-r {world_file}"
        }.items()
    )
    
    # ros_gz_bridge to bridge carousel command topic and simulation clock
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/ball_drop/carousel_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )
    
    # Ball Drop Controller Node
    controller_node = Node(
        package='uav_ball_drop',
        executable='ball_drop_controller',
        name='ball_drop_controller',
        output='screen',
        parameters=[{
            'world_name': 'ball_drop_world',
            'total_chambers': 6,
            'mechanism_pose_z': 2.5
        }]
    )
    
    return LaunchDescription([
        set_gz_resource_env,
        gz_sim,
        gz_bridge,
        controller_node
    ])
