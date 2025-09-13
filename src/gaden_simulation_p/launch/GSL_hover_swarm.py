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
    crazyflies_yaml = os.path.join(gaden_simulation_dir, 'config', 'crazyflies.yaml')
    crazyflietypes_yaml = os.path.join(gaden_simulation_dir, 'config', 'crazyflieTypes.yaml')
    cfrobot_yaml = os.path.join(gaden_simulation_dir, 'config', 'cfrobot.yaml')
    
    with open(os.path.join(get_package_share_directory('crazyflie'), 'urdf', 'crazyflie_description.urdf'), 'r') as urdf_file:
        urdf_content = urdf_file.read()
    
    # Load parameters from YAML files for REAL
    with open(crazyflies_yaml, 'r') as f:
        crazyflies_params = yaml.safe_load(f)
    
    with open(crazyflietypes_yaml, 'r') as f:
        crazyflietypes_params = yaml.safe_load(f)
    
    # Load parameters from YAML files for SIMULATION
    with open(cfrobot_yaml, 'r') as f:
        cfrobot_params = yaml.safe_load(f)
        
    crazyflies_list = crazyflies_params['crazyflies']
    robots = {}
    for cf in crazyflies_list:
        cf_id = cf['id']
        name = f'cf{cf_id}'
        address = f'E7E7E7E7{cf_id:02X}'  # Standard address pattern
        uri = f"radio://0/{cf['channel']}/2M/{address}"
        robots[name] = {
            'enabled': True,
            'type': cf['type'],
            'uri': uri,
            'initial_position': cf['initialPosition']
        }
    
    sim_params = {
        # Directly use the sections from cfrobot.yaml
        'robots': cfrobot_params['robots'],
        'robot_types': cfrobot_params['robot_types'],

        # Simulation-specific settings (not in cfrobot.yaml)
        'sim': {
            'backend': 'neuralswarm',
            'controller': 'mellinger',  # or 'mellinger'
            'visualizations': {'rviz': {'enabled': True}
            },
            'max_dt': 0.01  # simulation timestep
        },
        'world_tf_name': 'world',  # default frame

        # Firmware params are nested under robot_types.default in your YAML
        'firmwareParams': cfrobot_params['robot_types']['default']['firmware_params'],

        'robot_description': urdf_content
    }
    
    # Combine all parameters
    real_params = {
        'robots': robots,
        'robot_types': crazyflietypes_params['crazyflieTypes'],
        # Logging configuration
        'genericLogTopics': ['log1'],
        'genericLogTopicFrequencies': [10],
        'genericLogTopic_log1_Variables': ['stateEstimate.x', 'ctrltarget.x'],
        # Firmware parameters
        'firmware_params': {
            'commander': {'enHighLevel': 1},
            'stabilizer': {
                'estimator': 2,  # 1: complementary, 2: kalman
                'controller': 2  # 1: PID, 2: mellinger
            },
            'ring': {
                'effect': 16,  # 6: double spinner, 7: solid color, 16: packetRate
                'solidBlue': 255,
                'solidGreen': 0,
                'solidRed': 0,
                'headlightEnable': 0
            },
            'locSrv': {
                'extPosStdDev': 1e-3,
                'extQuatStdDev': 0.5e-1
            },
            'kalman': {'resetEstimation': 1}
        },
        # Tracking parameters
        'motion_capture_type': 'vicon',  # one of none,vicon,optitrack,optitrack_closed_source,qualisys,vrpn
        'object_tracking_type': 'libobjecttracker',  # one of motionCapture,libobjecttracker
        'send_position_only': False,  # set to False to send position+orientation; set to True to send position only
        'motion_capture_host_name': 'vicon',
        #'motion_capture_interface_ip': ""  # optional for optitrack with multiple interfaces
        'save_point_clouds': '/dev/null',  # set to a valid path to log mocap point cloud binary file.
        'print_latency': False,
        'write_csvs': False,
        'force_no_cache': False,
        'enable_parameters': True,
        'enable_logging': True,
        'enable_logging_pose': True
    }
    
    # Launch arguments
    sim_arg = DeclareLaunchArgument('sim', default_value='true', description='Whether to run in simulation mode')

    # Run crazyswarm_server for Real, if sim is set to False
    crazyflie_server_node = Node(
        package='crazyflie',
        executable='crazyflie_server',
        name='crazyflie_server',
        output='screen',
        parameters=[real_params],
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    # Run crazyswarm_server for Simulation, if sim is set to True
    sim_node = Node(
        package='crazyflie_sim',
        executable='crazyflie_server',
        name='sim',
        output='screen',
        parameters=[sim_params],
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    return LaunchDescription([
        sim_arg,
        crazyflie_server_node,
        sim_node,
        # Uncomment to enable RViz
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(gaden_simulation_dir, 'launch', 'GSL.rviz')]
        # )
    ])
