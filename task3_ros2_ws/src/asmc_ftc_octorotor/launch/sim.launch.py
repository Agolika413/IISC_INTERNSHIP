# pyrefly: ignore-all-errors
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_asmc = get_package_share_directory('asmc_ftc_octorotor')
    
    world_path = os.path.join(pkg_asmc, 'models', 'octorotor', 'octorotor.world')
    models_path = os.path.join(pkg_asmc, 'models')
    
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + models_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path
        
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )
    
    # Bridge: Odometry (GZ->ROS), Clock (GZ->ROS), Motor commands (ROS->GZ)
    # The motor plugin expects gz.msgs.Actuators on topic
    # /model/octorotor/command/motor_speed (namespaced by the model include)
    # The actual GZ topic name when included via <include> is:
    #   /{model_name}/command/motor_speed -> /octorotor/command/motor_speed
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/octorotor/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # The GZ motor plugins subscribe to /octorotor/command/motor_speed
            '/octorotor/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
        ],
        remappings=[
            ('/model/octorotor/odometry', '/odometry'),
            ('/octorotor/command/motor_speed', '/motor_speed_cmd'),
        ],
        output='screen'
    )
    
    # ASMC High-Level Controller Node
    asmc_node = Node(
        package='asmc_ftc_octorotor',
        executable='asmc_node',
        name='asmc_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Control Allocation Node
    ca_node = Node(
        package='asmc_ftc_octorotor',
        executable='ca_node',
        name='ca_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Fault Injector Node
    fault_injector = Node(
        package='asmc_ftc_octorotor',
        executable='fault_injector',
        name='fault_injector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo,
        bridge,
        asmc_node,
        ca_node,
        fault_injector
    ])
