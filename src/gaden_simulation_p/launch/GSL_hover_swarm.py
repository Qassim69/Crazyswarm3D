from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
import yaml
from ament_index_python.packages import get_package_share_directory
import os
 
def generate_launch_description():
    # Get the package share directory
    gaden_simulation_dir = get_package_share_directory('gaden_simulation_p')
    
    # Load YAML files
    cfrobot_yaml = os.path.join(gaden_simulation_dir, 'config', 'cfrobot.yaml')

    with open(os.path.join(get_package_share_directory('crazyflie'), 'urdf', 'crazyflie_description.urdf'), 'r') as urdf_file:
        urdf_content = urdf_file.read()

    # Load parameters from YAML files
    with open(cfrobot_yaml, 'r') as f:
        cfrobot_params = yaml.safe_load(f)
    
    sim_params = {
        # Directly use the sections from cfrobot.yaml
        'robots': cfrobot_params['robots'],
        'robot_types': cfrobot_params['robot_types'],

        # Simulation-specific settings (not in cfrobot.yaml)
        'sim': {
            'backend': 'np',
            'controller': 'pid',  # or 'mellinger'
            'visualizations': {'rviz': {'enabled': True}
            },
            'max_dt': 0.0005  # simulation timestep
        },
        'world_tf_name': 'world',  # default frame

        # Firmware params are nested under robot_types.default in your YAML
        'firmwareParams': cfrobot_params['robot_types']['default']['firmware_params'],
        
        'robot_description': urdf_content
    }

    # Launch arguments
    sim_arg = DeclareLaunchArgument('sim', default_value='true', description='Whether to run in simulation mode')

    # Run crazyflie_server.py[crazyflie_sim] for Simulation, if sim is set to True
    sim_node = Node(
        package='crazyflie_sim',
        executable='crazyflie_server',
        name='crazyflie_server',
        output='screen',
        parameters=[sim_params],
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    return LaunchDescription([
        sim_arg,
        sim_node,
    ])
