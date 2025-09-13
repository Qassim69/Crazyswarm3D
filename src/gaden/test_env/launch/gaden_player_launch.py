"""
    Launch file to run GADEN_player, a node in charge of playing back 
    the gas dispersal generated with GADEN, offering visualization, and
    simulated sensors to access the gas concentration and wind vector.

    Parameters:
        @param scenario - The scenario where dispersal takes place
        @param simulation - The wind flow actuating in the scenario
        @param source_(xyz) - The 3D position of the release point
"""

"""
    Launch file to run GADEN gas dispersion simulator.
    IMPORTANT: GADEN_preprocessing should be called before!

    Parameters:
        @param scenario - The scenario where dispersal takes place
        @param simulation - The wind flow actuating in the scenario
        @param source_(xyz) - The 3D position of the release point
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

# Internal gaden utilities
import sys
sys.path.append(get_package_share_directory('gaden_common'))
from gaden_internal_py.utils import read_sim_yaml
#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument(
            "scenario",
            default_value=["10x6_central_obstacle"],
            description="scenario to simulate",
        ),
        DeclareLaunchArgument(
            "simulation",
            default_value=["sim1"],
            description="name of the simulation yaml file",
        ),
        DeclareLaunchArgument(
            "use_rqt_plot",
            default_value=["True"],
            description="Open rqt_plot with MOX/PID/Anemometer topics",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value=["True"],
            description="",
        ),
    	DeclareLaunchArgument(
            "source_x",
            default_value=["0.50"],
            description="X position of the gas source",
        ),
        DeclareLaunchArgument(
            "source_y",
            default_value=["0.50"],
            description="Y position of the gas source",
        ),
        DeclareLaunchArgument(
            "source_z",
            default_value=["0.75"],
            description="Z position of the gas source",
        ),
    ]
#==========================
def launch_setup(context, *args, **kwargs):
    scenario = LaunchConfiguration("scenario").perform(context)
    pkg_dir = LaunchConfiguration("pkg_dir").perform(context)

    params_yaml_file = os.path.join(pkg_dir, "scenarios", scenario, "params", "gaden_params.yaml")
    read_sim_yaml(context)
    
    # Static TF transforms for sensors
    tf_mox0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mox0_broadcaster',
        arguments=['3.0', '3.0', '0.4', '0', '0', '0', 'map', 'mox0_frame']
    )

    tf_mox1 = Node(
    	package='tf2_ros',
    	executable='static_transform_publisher',
    	name='mox1_broadcaster',
    	arguments=['3.0', '3.0', '0.4', '0', '0', '0', 'map', 'mox1_frame']
    )

    tf_mox2 = Node(
    	package='tf2_ros',
    	executable='static_transform_publisher',
    	name='mox2_broadcaster',
    	arguments=['3.0', '3.0', '0.4', '0', '0', '0', 'map', 'mox2_frame']
    )

    tf_pid = Node(
    	package='tf2_ros',
    	executable='static_transform_publisher',
    	name='pid_broadcaster',
    	arguments=['3.0', '3.0', '0.4', '0', '0', '0', 'map', 'pid_frame']
    )

    tf_anemometer = Node(
    	package='tf2_ros',
    	executable='static_transform_publisher',
    	name='anemometer_broadcaster',
    	arguments=['3.0', '3.0', '0.4', '0', '0', '0', 'map', 'anemometer_frame']
    )

    # Mox00 namespace and node
    mox00_namespace = PushRosNamespace('Mox00')
    mox00_node = Node(
        package='simulated_gas_sensor',
        executable='simulated_gas_sensor',
        name='fake_mox',
        output='screen',
        parameters=[{
            'sensor_model': 0,
            'sensor_frame': 'mox0_frame',
            'fixed_frame': 'map',
            'rate': 10.0
        }]
    )

    # Mox01 namespace and node
    mox01_namespace = PushRosNamespace('Mox01')
    mox01_node = Node(
        package='simulated_gas_sensor',
        executable='simulated_gas_sensor',
        name='fake_mox',
        output='screen',
        parameters=[{
            'sensor_model': 1,
            'sensor_frame': 'mox1_frame',
            'fixed_frame': 'map',
            'rate': 10.0
        }]
    )

    # Mox02 namespace and node
    mox02_namespace = PushRosNamespace('Mox02')
    mox02_node = Node(
        package='simulated_gas_sensor',
        executable='simulated_gas_sensor',
        name='fake_mox',
        output='screen',
        parameters=[{
            'sensor_model': 2,
            'sensor_frame': 'mox2_frame',
            'fixed_frame': 'map',
            'rate': 10.0
        }]
    )

    # PID namespace and node
    pid_namespace = PushRosNamespace('PID')
    pid_node = Node(
        package='simulated_gas_sensor',
        executable='simulated_gas_sensor',
        name='fake_pid',
        output='screen',
        parameters=[{
            'sensor_model': 30,
            'sensor_frame': 'pid_frame',
            'fixed_frame': 'map',
            'rate': 10.0
        }]
    )

    # Anemometer namespace and node
    anemometer_namespace = PushRosNamespace('Anemometer01')
    anemometer_node = Node(
        package='simulated_anemometer',
        executable='simulated_anemometer',
        name='fake_anemometer',
        output='screen',
        parameters=[{
            'sensor_frame': 'anemometer_frame',
            'fixed_frame': 'map',
            'frequency': 20,
            'noise_std': 0.1,
            'use_map_ref_system': False
        }]
    )
    
    # RQT plots for sensors
    rqt_plot1 = Node(
        condition=IfCondition(LaunchConfiguration("use_rqt_plot")),
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot1",
        output="screen",
        arguments=[
            "/Mox00/Sensor_reading/raw",
            "/Mox01/Sensor_reading/raw",
            "/Mox02/Sensor_reading/raw",
        ],
    )

    rqt_plot2 = Node(
        condition=IfCondition(LaunchConfiguration("use_rqt_plot")),
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot2",
        output="screen",
        arguments=["/PID/Sensor_reading/raw"],
    )

    rqt_plot3 = Node(
        condition=IfCondition(LaunchConfiguration("use_rqt_plot")),
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot3",
        output="screen",
        arguments=["/Anemometer01/WindSensor_reading"],
    )

    return [
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d" + os.path.join(pkg_dir, "launch", "gaden.rviz")],
        ),
        
        # New additions: RQT plots
        rqt_plot1, rqt_plot2, rqt_plot3,
        
        # gaden_environment (for RVIZ visualization)
        Node(
            package='gaden_environment',
            executable='environment',
            name='gaden_environment',
            output='screen',
            parameters=[ParameterFile(params_yaml_file, allow_substs=True)]
            ),

        # gaden_player
        Node(
            package="gaden_player",
            executable="player",
            name="gaden_player",
            output="screen",
            parameters=[ParameterFile(params_yaml_file, allow_substs=True)],
        ),
        
        # New additions: TF transforms
        tf_mox0, tf_mox1, tf_mox2, tf_pid, tf_anemometer,
        # New additions: Gas sensors
        mox00_namespace, mox00_node,
        mox01_namespace, mox01_node,
        mox02_namespace, mox02_node,
        pid_namespace, pid_node,
        # New additions: Wind sensor
        anemometer_namespace, anemometer_node,
    ]

def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),

        SetLaunchConfiguration(
            name="pkg_dir",
            value=[get_package_share_directory("test_env")],
        ),
    ]
    
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    
    return  LaunchDescription(launch_description)
