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
    test_env = get_package_share_directory('test_env')
    gsl_hover_launch = os.path.join(gaden_simulation_p, 'launch', 'GSL_hover_swarm.py')
    gaden_player_launch = os.path.join(test_env, 'launch', 'gaden_player_launch.py')
    
    # Load YAML
    cfrobot_yaml = os.path.join(gaden_simulation_p, 'config', 'cfrobot.yaml')
    with open(cfrobot_yaml, 'r') as f:
        data = yaml.safe_load(f)
    crazyflies_data = data['robots']
    
    # Use ROS Time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use ROS Time from /clock'
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
        executable='APF_1_1_env',
        name='GSLenvironment',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Sensor Nodes [mox1; mox2; mox3; mox4]
    sensor_group = GroupAction([
        Node(
            package='simulated_gas_sensor',
            executable='simulated_gas_sensor',
            name='mox1',
            output='screen',
            parameters=[
                {'sensor_model': 0},
                {'sensor_frame': 'cf1'},
                {'fixed_frame': 'map'},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),
        Node(
            package='simulated_gas_sensor',
            executable='simulated_gas_sensor',
            name='mox2',
            output='screen',
            parameters=[
                {'sensor_model': 0},
                {'sensor_frame': 'cf2'},
                {'fixed_frame': 'map'},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),
        Node(
            package='simulated_gas_sensor',
            executable='simulated_gas_sensor',
            name='mox3',
            output='screen',
            parameters=[
                {'sensor_model': 0},
                {'sensor_frame': 'cf3'},
                {'fixed_frame': 'map'},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),
        Node(
            package='simulated_gas_sensor',
            executable='simulated_gas_sensor',
            name='mox4',
            output='screen',
            parameters=[
                {'sensor_model': 0},
                {'sensor_frame': 'cf4'},
                {'fixed_frame': 'map'},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),
    ])

    # Agent CF1
    agent_cf1 = Node(
        package='gaden_simulation_p',
        executable='APF_1_1_agent',
        name='cf1',
        parameters=[
            get_agent_params(1),
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
        executable='APF_1_1_agent',
        name='cf2',
        parameters=[
            get_agent_params(2),
            {
                'crazyflies_ids': str(crazyflies_ids),
                'crazyflies_positions': str(crazyflies_positions)
            }
        ],
        output='screen'
    )

    # Agent CF3
    agent_cf3 = Node(
        package='gaden_simulation_p',
        executable='APF_1_1_agent',
        name='cf3',
        parameters=[
            get_agent_params(3),
            {
                'crazyflies_ids': str(crazyflies_ids),
                'crazyflies_positions': str(crazyflies_positions)
            }
        ],
        output='screen'
    )

    # Agent CF4
    agent_cf4 = Node(
        package='gaden_simulation_p',
        executable='APF_1_1_agent',
        name='cf4',
        parameters=[
            get_agent_params(4),
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

        # Static transform from world to map
        world_to_map_tf,
        
        # Start gaden_player and GSL_hover_swarm
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(gaden_player_launch),
                    launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(gsl_hover_launch),
                    launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
                ),
            ]
        ),

        # Start Environment and Sensor after 8s which is 5s after gaden_player and crazyflie_server[gsl_hover_launch]
        TimerAction(
            period=15.0,
            actions=[env_node, sensor_group]
        ),
        
        # Start CF1
        TimerAction(
            period=17.0,  # CF1 starts 2s after Environment and Sensor
            actions=[
                LogInfo(msg="Starting CF1 agent..."),
                agent_cf1
            ]
        ),
        # Start CF2
        TimerAction(
            period=19.0,  # CF2 starts 3s after CF1
            actions=[
                LogInfo(msg="Starting CF2 agent..."),
                agent_cf2
            ]
        ),
        # Start CF3
        TimerAction(
            period=21.0,  # CF3 starts 3s after CF2
            actions=[
                LogInfo(msg="Starting CF3 agents..."),
                agent_cf3
            ]
        ),
        # Start CF4
        TimerAction(
            period=23.0,  # CF4 starts 3s after CF3
            actions=[
                LogInfo(msg="Starting CF4 agent..."),
                agent_cf4
            ]
        ),
    ])