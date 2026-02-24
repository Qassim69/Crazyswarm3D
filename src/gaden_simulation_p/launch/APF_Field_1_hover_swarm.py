import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    gaden_simulation_dir = get_package_share_directory('gaden_simulation_p')
    
    # Load YAML files
    cfreal_yaml = os.path.join(gaden_simulation_dir, 'config', 'cfreal.yaml')
    
    # Load parameters from YAML files
    with open(cfreal_yaml, 'r') as f:
        cfreal_params = yaml.safe_load(f)
    
    # Combine all parameters
    real_params = {
        'robots': cfreal_params['robots'],
        'robot_types': cfreal_params['robot_types'],
        # Tracking parameters
        'motion_capture_type': 'none',  # one of none,vicon,optitrack,optitrack_closed_source,qualisys,vrpn
        'object_tracking_type': 'libobjecttracker',  # one of motionCapture,libobjecttracker
        'send_position_only': False,  # set to False to send position+orientation; set to True to send position only
        'motion_capture_host_name': 'vicon',
        #'motion_capture_interface_ip': ""  # optional for optitrack with multiple interfaces
        'print_latency': False,
        'write_csvs': False,
        'forceNoCache': False,
        'enable_parameters': True,
        'enable_logging': True,
        'enable_logging_pose': True
    }
    
    # Launch arguments
    sim_arg = DeclareLaunchArgument('sim', default_value='false', description='Whether to run in simulation mode')
    backend = DeclareLaunchArgument('backend', default_value='cflib', description='Backend to use')

    # Run crazyflie_server.cpp[crazyflie/src] for Real, if sim is set to False
    real_node = Node(
        package='crazyflie',
        executable='crazyflie_server',
        name='crazyflie_server',
        output='screen',
        parameters=[real_params],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('backend'), "' == 'cflib' and '",LaunchConfiguration('sim'), "' == 'false'"]))
    )

    return LaunchDescription([
        sim_arg,
        backend,
        real_node,
        #Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', os.path.join(gaden_simulation_dir, 'launch', 'GSL.rviz')]
        #)
    ])