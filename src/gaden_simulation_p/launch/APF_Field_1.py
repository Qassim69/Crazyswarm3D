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
    
    # Use simulation time argument - CRITICAL: Set consistent time policy
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Use wall clock time to avoid timing conflicts
        description='Use wall clock time for all nodes'
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
    
    # GSL environment node
    env_node = Node(
        package='gaden_simulation_p',
        executable='APF_Field_1_env',
        name='GSLenvironment',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
       
    # Create individual agent nodes
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
        # Use simulation time parameter
        use_sim_time_arg,
        
        # Start TF publisher first
        world_to_map_tf,
        
        # Start included launches with proper use_sim_time propagation
        TimerAction(
            period=1.0,  # 1 second delay after TF
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(field_hover_launch),
                    launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
                ),
            ]
        ),
        
        TimerAction(
            period=2.0,	# Start environment and sensors after 2 seconds
            actions=[env_node]
        ),
        
        # Start CF1 first
        TimerAction(
            period=5.0,  # 5 second after after all services
            actions=[
                LogInfo(msg="Starting CF1 agent..."),
                agent_cf1
            ]
        ),
        # Start CF2 after CF1 has started
        RegisterEventHandler(
            OnProcessStart(
                target_action=agent_cf1,
                on_start=[
                    TimerAction(
                        period=1.0,  # 1 second delay between agents
                        actions=[
                            LogInfo(msg="Starting CF2 agent..."),
                            agent_cf2
                        ]
                    )
                ]
            )
        ),
    ])
