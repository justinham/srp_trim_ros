from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    world_file = "/home/connau/ConnAu/Data/empty.world"


    # # Include Gazebo server and client
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('gazebo_ros').find('gazebo_ros'),
            'launch',
            'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('gazebo_ros').find('gazebo_ros'),
            'launch',
            'gzclient.launch.py')),
    )

    mode_arg = DeclareLaunchArgument(
        "mode_arg",
        default_value="sim",
        description='From where you wanna source the data?'
    )

    mode_arg_value = LaunchConfiguration("mode_arg")

    carla_sync_enable_node = Node(
        package='ground_truth_sim_pkg',
        executable='carla_sync_enable_node',
        name='carla_sync_enable_node',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    s1_node = Node(
        package='ground_truth_sim_pkg',
        executable='s1_node',
        name='s1_node',
        parameters=[{"mode_arg": mode_arg_value}]
    )

    ground_truth_node = Node(
        package='ground_truth_sim_pkg',
        executable='ground_truth_node',
        name='ground_truth_node',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    data_coll_scenarios = Node(
        package='ground_truth_sim_pkg',
        executable='data_coll_scenarios',
        name='data_coll_scenarios',
        parameters=[{'mode_arg': mode_arg_value}]
    )
    
    ego_data_source = Node(
        package='ground_truth_sim_pkg',
        executable='ego_data_source',
        name='ego_data_source',
        parameters=[{'mode_arg': mode_arg_value}]
    )
    
    infra_data_source = Node(
        package='ground_truth_sim_pkg',
        executable='infra_data_source',
        name='infra_data_source',
        parameters=[{'mode_arg': mode_arg_value}]
    )
    
    estimate_fused_node = Node(
        package='ground_truth_sim_pkg',
        executable='estimate_fused_node',
        name='estimate_fused_node',
        parameters=[{'mode_arg': mode_arg_value}]
    )
    
    infra_to_global = Node(
        package='ground_truth_sim_pkg',
        executable='infra_to_global',
        name='infra_to_global',
        parameters=[{'mode_arg': mode_arg_value}]
    )
    
    ego_to_global = Node(
        package='ground_truth_sim_pkg',
        executable='ego_to_global',
        name='ego_to_global',
        parameters=[{'mode_arg': mode_arg_value}]
    )
    
    estimate_ego_node = Node(
        package='ground_truth_sim_pkg',
        executable='estimate_ego_node',
        name='estimate_ego_node',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    estimate_infra_node = Node(
        package='ground_truth_sim_pkg',
        executable='estimate_infra_node',
        name='estimate_infra_node',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    objects_visualizer_node = Node(
        package='ground_truth_sim_pkg',
        executable='objects_visualizer_node',
        name='objects_visualizer_node',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    intersection_finder_node = Node(
        package='ground_truth_sim_pkg',
        executable='intersection_finder_node',
        name='intersection_finder_node',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    ego_msg_to_json = Node(
        package='ground_truth_sim_pkg',
        executable='ego_msg_to_json',
        name='ego_msg_to_json',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    real_ego_gps_to_xy = Node(
        package='ground_truth_sim_pkg',
        executable='real_ego_gps_to_xy',
        name='real_ego_gps_to_xy',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    object_matcher_node = Node(
        package='ground_truth_sim_pkg',
        executable='object_matcher_node',
        name='object_matcher_node',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    real_infra_source_from_gt = Node(
        package='ground_truth_sim_pkg',
        executable='real_infra_source_from_gt',
        name='real_infra_source_from_gt',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    infra_playback_node = Node(
        package='ground_truth_sim_pkg',
        executable='infra_playback_node',
        name='infra_playback_node',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    global_gt_publisher_node = Node(
        package='ground_truth_sim_pkg',
        executable='global_gt_publisher_node',
        name='global_gt_publisher_node',
        parameters=[{'mode_arg': mode_arg_value}]
    )

    ros2_to_app_bridge_node = Node(
        package='ground_truth_sim_pkg',
        executable='ros2_to_app_bridge_node',
        name='ros2_to_app_bridge_node',
        parameters=[{'mode_arg': mode_arg_value}]
    )


    return LaunchDescription([
        mode_arg,
        # gazebo_server,
        # gazebo_client,
        # carla_sync_enable_node,
        # s1_node,
        # ground_truth_node,
        # data_coll_scenarios,
        # ego_data_source,
        # infra_data_source,
        #estimate_fused_node,
        infra_to_global,
        ego_to_global,
        #estimate_ego_node,
        #estimate_infra_node,
        # intersection_finder_node,
        # objects_visualizer_node,
        ego_msg_to_json,
        real_ego_gps_to_xy,
        object_matcher_node,
        # infra_playback_node,

        # global_gt_publisher_node
        # real_infra_source_from_gt
    ])