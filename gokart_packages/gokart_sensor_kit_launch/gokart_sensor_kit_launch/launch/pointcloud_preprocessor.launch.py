# Single LiDAR passthrough preprocessor for gokart (Ouster OS0)
import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    # Single LiDAR — use passthrough instead of concatenate
    passthrough_component = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::PassThroughFilterComponent",
        name="passthrough_filter",
        remappings=[
            ("input",  "/sensing/lidar/top/pointcloud_raw"),
            ("output", "/sensing/lidar/top/pointcloud_raw_passthrough"),
        ],
        parameters=[{
            "output_frame": "base_link",
            "min_z": -1.5,
            "max_z": 3.0,
            
        }],
        extra_arguments=[
         {"use_intra_process_comms": LaunchConfiguration("use_intra_process")},
         {"use_sensor_data_qos":True},
         ],
    )

    passthrough_loader = LoadComposableNodes(
        composable_node_descriptions=[passthrough_component],
        target_container=LaunchConfiguration("pointcloud_container_name"),
    )

    return [passthrough_loader]

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_concat_filter", "false")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )
    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
