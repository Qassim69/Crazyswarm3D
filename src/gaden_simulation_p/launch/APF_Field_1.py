from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.actions import GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Paths to included launch files
    gaden_simulation_p = get_package_share_directory('gaden_simulation_p')
    field_hover_launch = os.path.join(gaden_simulation_p, 'launch', 'APF_Field_1_hover_swarm.py')
    
    # Load YAML
    cfreal_yaml = os.path.join(gaden_simulation_p, 'config', 'cfreal.yaml')
    with open(cfreal_yaml, 'r') as f:
        data = yaml.safe_load(f)
    crazyflies_data = data['robots']
    
    # Do not use ROS Time for Real. Keep it False. 
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
    
    crazyflies_ids = []
    crazyflies_positions = []
    for key in sorted(crazyflies_data.keys()):
        cfid = int(key[2:])  # Extract ID from 'cf1' -> 1, etc.
        crazyflies_ids.append(cfid)
        crazyflies_positions.append(crazyflies_data[key]['initial_position'])
    
    # Function to get flattened params for a given cfid
    def get_agent_params(cfid):
        key = f'cf{cfid}'
        if key not in crazyflies_data:
            raise ValueError(f"No entry for {key} in cfreal.yaml")
        drone = crazyflies_data[key].copy()
        cf_type = str(drone.pop('type', 'default'))
        x, y, z = drone.pop('initial_position', [0.0, 0.0, 0.0])
        return {
            'cfid': cfid,
            'channel': str(drone.pop('channel', 0)),
            'initial_x': str(x),
            'initial_y': str(y),
            'initial_z': str(z),
            'cf_type': cf_type,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }
    
# Declare Environment Node (GSL)
    # GSL environment node
    env_node = Node(
        package='gaden_simulation_p',
        executable='APF_Field_1_env',
        name='GSLenvironment',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

# Declare Individual Agent Nodes
    # Agent CF1
    agent_cf1 = Node(
        package='gaden_simulation_p',
        executable='APF_Field_1_agent',
        name='cf6',
        parameters=[
            get_agent_params(6),
            {
                'crazyflies_ids': str(crazyflies_ids),
                'crazyflies_positions': str(crazyflies_positions)
            }
        ],
        output='screen'
    )

    # Agent CF2
    agent_cf2 = Node(
        package='gaden_simulation_p',
        executable='APF_Field_1_agent',
        name='cf8',
        parameters=[
            get_agent_params(8),
            {
                'crazyflies_ids': str(crazyflies_ids),
                'crazyflies_positions': str(crazyflies_positions)
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        # Use ROS Time parameter
        use_sim_time_arg,
        
        # Static Transform from world to map
        world_to_map_tf,
        
        # Start APF_Field_1_hover_swarm.py after 2s
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(field_hover_launch),
                    launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
                ),
            ]
        ),
        
        TimerAction(
            period=6.0,	# Start Environment Node after 6s
            actions=[env_node]
        ),
        
        # Start CF1
        TimerAction(
            period=8.0,  # CF1 starts 2s after Environment Node
            actions=[
                LogInfo(msg="Starting CF1 agent..."),
                agent_cf1
            ]
        ),
        # Start CF2
        TimerAction(
            period=9.0,  # CF2 starts 1s after CF1
            actions=[
                LogInfo(msg="Starting CF2 agent..."),
                agent_cf2
            ]
        ),
    ])