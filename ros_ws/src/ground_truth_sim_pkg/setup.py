from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ground_truth_sim_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yz4d3h',
    maintainer_email='yz4d3h@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # "ground_truth_node = ground_truth_sim_pkg.ground_truth_node:main",
            # "ego_data_source = ground_truth_sim_pkg.ego_data_source:main",
            # "infra_data_source = ground_truth_sim_pkg.infra_data_source:main",
            # "carla_sync_enable_node = ground_truth_sim_pkg.carla_sync_enable_node:main",
            # "s1_node = ground_truth_sim_pkg.s1_node:main",
            "estimate_fused_node = ground_truth_sim_pkg.estimate_fused_node:main",
            "infra_to_global = ground_truth_sim_pkg.infra_to_global:main",
            "ego_to_global = ground_truth_sim_pkg.ego_to_global:main",
            "estimate_ego_node = ground_truth_sim_pkg.estimate_ego_node:main",
            "estimate_infra_node = ground_truth_sim_pkg.estimate_infra_node:main",
            "objects_visualizer_node = ground_truth_sim_pkg.objects_visualizer_node:main",
            # "intersection_finder_node = ground_truth_sim_pkg.intersection_finder_node:main",
            "ego_msg_to_json = ground_truth_sim_pkg.ego_msg_to_json:main",
            "real_ego_gps_to_xy = ground_truth_sim_pkg.real_ego_gps_to_xy:main",
            "object_matcher_node = ground_truth_sim_pkg.object_matcher_node:main",
            # "data_coll_scenarios = ground_truth_sim_pkg.data_coll_scenarios:main",
            # "real_infra_source_from_gt = ground_truth_sim_pkg.real_infra_source_from_gt:main",
            "infra_playback_node = ground_truth_sim_pkg.infra_playback_node:main",
            "infra_playback_node2 = ground_truth_sim_pkg.infra_playback_node2:main",
            # "global_gt_publisher_node = ground_truth_sim_pkg.global_gt_publisher_node:main",
            "ego_speed_test_node = ground_truth_sim_pkg.ego_speed_test_node:main",
            "ego_speed_test_node_ff2 = ground_truth_sim_pkg.ego_speed_test_node_ff2:main",
            "ego_speed_test_node_ff3 = ground_truth_sim_pkg.ego_speed_test_node_ff3:main",
            "ego_speed_test_node_ff3_new = ground_truth_sim_pkg.ego_speed_test_node_ff3_new:main",
            "raw_infra_playback_node = ground_truth_sim_pkg.raw_infra_playback_node:main",
            "speed_test_gps_sim_node = ground_truth_sim_pkg.speed_test_gps_sim_node:main",
            "ego_speed_test_node_ff = ground_truth_sim_pkg.ego_speed_test_node_ff:main",
            "gui_mover_camera = ground_truth_sim_pkg.gui_mover_camera:main",
            "obu_interface_node = ground_truth_sim_pkg.obu_interface_node:main",
            "obu_interface_node2 = ground_truth_sim_pkg.obu_interface_node2:main",
            "obu_interface_node2_logger = ground_truth_sim_pkg.obu_interface_node2_logger:main",
            "ros2_to_app_bridge_node = ground_truth_sim_pkg.ros2_to_app_bridge_node:main",
            "app_responder_node = ground_truth_sim_pkg.app_responder_node:main",
        ],
    },
)
