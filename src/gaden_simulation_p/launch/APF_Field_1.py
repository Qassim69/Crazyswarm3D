import os
import yaml
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction

def generate_launch_description():
# Paths to Launch Files
    gaden_simulation_p = get_package_share_directory('gaden_simulation_p')
    field_hover_launch = os.path.join(gaden_simulation_p, 'launch', 'APF_Field_1_hover_swarm.py')
    
# Load YAML
    cfreal_yaml = os.path.join(gaden_simulation_p, 'config', 'cfreal.yaml')
    with open(cfreal_yaml, 'r') as f:
        data = yaml.safe_load(f)
    crazyflies_data = data['robots']
    
# Use Wall Time. Keep it False. 
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='This means Wall Clock will be used.'
    )

    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

# Get an array of IDs and Positions
    crazyflies_ids = []
    crazyflies_positions = []
    for key in sorted(crazyflies_data.keys()):
        # Only add drones that are marked as enabled
        if crazyflies_data[key].get('enabled', True):
            # Extract ID from cf[1,2,3,4]
            cfid = int(key[2:])
            crazyflies_ids.append(cfid)
            crazyflies_positions.append(crazyflies_data[key].get('initial_position', [0.0, 0.0, 0.0]))

# GSL Environment Node
    env_node = Node(
        package='gaden_simulation_p',
        executable='APF_Field_1_env',
        name='GSLenvironment',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'drone_ids': crazyflies_ids}
        ],
        output='screen'
    )

# Drone Nodes [CF1; CF2; CF3; CF4]
    agent_actions = []
    base_period = 17.0

    for i, cfid in enumerate(crazyflies_ids):
        # Calculate exactly a 2s stagger delay between each Drone
        delay = base_period + (i * 2.0)
        
        agent_node = Node(
            package='gaden_simulation_p',
            executable='APF_Field_1_agent',
            name= f'cf{cfid}',
            parameters=[
                {
                    'cfid': cfid,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'crazyflies_ids': str(crazyflies_ids),
                    'crazyflies_positions': str(crazyflies_positions)
                }
            ],
            output='screen'
        )
        
        agent_actions.append(TimerAction(
            period=delay,
            actions=[
                LogInfo(msg=f"Starting CF{cfid} agent..."),
                agent_node
            ]
        ))

    return LaunchDescription([
        # Use Wall Time
        use_sim_time_arg,
        
        # Static Transform from world to map
        world_to_map_tf,
        
        # Start APF_Field_1_hover_swarm.py
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(field_hover_launch),
                    launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
                ),
            ]
        ),
        
        # Start Environment 6s after APF_Field_1_hover_swarm.py[crazyflie_server.cpp]
        TimerAction(
            period=6.0,
            actions=[env_node]
        ),
        
        # Add all dynamically generated Drones[CF1; CF2; CF3; CF4]
        *agent_actions
    ])