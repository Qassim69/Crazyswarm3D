import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


# Internal gaden utilities
import sys
sys.path.append(get_package_share_directory('gaden_common'))
from gaden_internal_py.utils import read_sim_yaml

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("scenario", default_value="10x6_empty_room", description="scenario to simulate",),
        
        DeclareLaunchArgument("use_rqt_plot", default_value="True", description="Launch rqt_plot with MOX topics",),

        DeclareLaunchArgument("use_rviz", default_value="True", description="",),
        
    	DeclareLaunchArgument("source_x", default_value="1.5", description="X position of the gas source",),
    	DeclareLaunchArgument("source_y", default_value="3.00", description="Y position of the gas source",),
    	DeclareLaunchArgument("source_z", default_value="0.75", description="Z position of the gas source",),
    ]
#==========================
def launch_setup(context, *args, **kwargs):
    scenario = LaunchConfiguration("scenario").perform(context)
    pkg_dir = LaunchConfiguration("pkg_dir").perform(context)
    source_x = LaunchConfiguration("source_x").perform(context)
    source_y = LaunchConfiguration("source_y").perform(context)
    source_z = LaunchConfiguration("source_z").perform(context)

    params_yaml_file = os.path.join(pkg_dir, "scenarios", scenario, "params", "GSL_gaden_params.yaml")
    
    # read_sim_yaml(context)
    
    return [
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d' + os.path.join(pkg_dir, 'launch', 'gaden.rviz')]
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("use_rqt_plot")),
            package="rqt_plot",
            executable="rqt_plot",
            name="rqt_plot4",
            output="screen",
            arguments=[
                "/mox1/Sensor_reading/raw",
                "/mox2/Sensor_reading/raw",
                "/mox3/Sensor_reading/raw",
                "/mox4/Sensor_reading/raw",
            ],
        ),

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
