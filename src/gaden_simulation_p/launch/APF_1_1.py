from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Paths to included launch files
    gaden_simulation_p = get_package_share_directory('gaden_simulation_p')
    test_env = get_package_share_directory('test_env')
    gsl_hover_launch = os.path.join(gaden_simulation_p, 'launch', 'GSL_hover_swarm.py')
    gaden_player_launch = os.path.join(test_env, 'launch', 'GSL_GADEN_player.py')

    # Load YAML (correct path without brackets)
    cfrobot_yaml = os.path.join(gaden_simulation_p, 'config', 'cfrobot.yaml')
    with open(cfrobot_yaml, 'r') as f:
        data = yaml.safe_load(f)
        crazyflies_data = data['robots']

    crazyflies_ids = []
    crazyflies_positions = []
    for key in sorted(crazyflies_data.keys()):  # Sort keys to maintain consistent order (e.g., cf1, cf2, etc.)
        cfid = int(key[2:])  # Extract ID from 'cf1' -> 1, etc.
        crazyflies_ids.append(cfid)
        crazyflies_positions.append(crazyflies_data[key]['initial_position'])

    # Function to get flattened params for a given cfid
    def get_agent_params(cfid):
        key = f'cf{cfid}'
        if key not in crazyflies_data:
            raise ValueError(f"No entry for {key} in cfrobot.yaml")
        drone = crazyflies_data[key].copy()
        cf_type = str(drone.pop('type', 'default'))
        x, y, z = drone.pop('initial_position', [0.0, 0.0, 0.0])
        return {
            'cfid': str(cfid),
            'channel': str(drone.pop('channel', 0)),
            'initial_x': str(x),
            'initial_y': str(y),
            'initial_z': str(z),
            'cf_type': cf_type,
        }
        
    # GSL environment node
    env_node = Node(
        package='gaden_simulation_p',
        executable='APF_1_1_env',
        name='GSLenvironment'
        )
    
    # Group for sensor nodes
    sensor_group = GroupAction([
        Node(
            package='simulated_gas_sensor',
            executable='simulated_gas_sensor',
            namespace='mox1',
            name='mox1',
            output='screen',
            parameters=[
                {'sensor_model': 0},
                {'sensor_frame': 'cf1'},
                {'fixed_frame': 'map'}
            ],

        ),
        Node(
            package='simulated_gas_sensor',
            executable='simulated_gas_sensor',
            namespace='mox2',
            name='mox2',
            output='screen',
            parameters=[
                {'sensor_model': 0},
                {'sensor_frame': 'cf2'},
                {'fixed_frame': 'map'}
            ]
        ),
        Node(
            package='simulated_gas_sensor',
            executable='simulated_gas_sensor',
            namespace='mox3',
            name='mox3',
            output='screen',
            parameters=[
                {'sensor_model': 0},
                {'sensor_frame': 'cf3'},
                {'fixed_frame': 'map'}
            ]
        ),
        Node(
            package='simulated_gas_sensor',
            executable='simulated_gas_sensor',
            namespace='mox4',
            name='mox4',
            output='screen',
            parameters=[
                {'sensor_model': 0},
                {'sensor_frame': 'cf4'},
                {'fixed_frame': 'map'}
            ]
        ),
    ])

    # Handler to launch sensors after env node starts
    sensor_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=env_node,
            on_start=[sensor_group]
        )
    )

    return LaunchDescription([
        # Use simulation time parameter
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # Clock node
        Node(
            package='gaden_simulation_p',
            executable='clock',
            name='Clock',
            parameters=[{'factor': 18.0}]
        ),

        # Include GSL Hover Swarm Launch
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gsl_hover_launch),),

        # Include GSL GADEN Player Launch
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gaden_player_launch),),

        # GSL environment node
        env_node,

        # Handler for sensors
        sensor_handler,
        
        # Agent nodes with flattened parameters
        Node(
            package='gaden_simulation_p',
            executable='APF_1_1_agent',
            name='cf1',
            parameters=[get_agent_params(1), {
                "crazyflies_ids": str(crazyflies_ids),
                "crazyflies_positions": str(crazyflies_positions)
            }]
        ),
        Node(
            package='gaden_simulation_p',
            executable='APF_1_1_agent',
            name='cf2',
            parameters=[get_agent_params(2), {
                "crazyflies_ids": str(crazyflies_ids),
                "crazyflies_positions": str(crazyflies_positions)
            }]
        ),
        Node(
            package='gaden_simulation_p',
            executable='APF_1_1_agent',
            name='cf3',
            parameters=[get_agent_params(3), {
                "crazyflies_ids": str(crazyflies_ids),
                "crazyflies_positions": str(crazyflies_positions)
            }]
        ),
        Node(
            package='gaden_simulation_p',
            executable='APF_1_1_agent',
            name='cf4',
            parameters=[get_agent_params(4), {
                "crazyflies_ids": str(crazyflies_ids),
                "crazyflies_positions": str(crazyflies_positions)
            }]
        ),
    ])
