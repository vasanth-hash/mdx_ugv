from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command,  LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
  package_name = 'mdx_ugv'
  pkg_share = get_package_share_directory(package_name)
  
  robot_localization_params_file = os.path.join(pkg_share, "config", "dual_ekf_navsat_params.yaml")
  
  rviz_file = os.path.join(pkg_share, 'rviz', 'dual_ekf_localization.rviz')
  
  bring_up = IncludeLaunchDescription(
              PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory(package_name),'launch','display.launch.py'
              )]),
              launch_arguments={'rviz_launch': 'false'}.items() 
  )

  ekf_filter_node_odom = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter_node_odom",
    output="screen",
    parameters=[robot_localization_params_file, {'use_sim_time': True}],
    remappings=[("odometry/filtered", "odometry/filtered/local")],
  ) 

  ekf_filter_node_map = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter_node_map",
    output="screen",
    parameters=[robot_localization_params_file, {'use_sim_time': True}],
    remappings=[("odometry/filtered", "odometry/filtered/global")],
  ) 

  navsat_transform_node = Node(
    package="robot_localization",
    executable="navsat_transform_node",
    name="navsat_transform",
    output="screen",
    parameters=[robot_localization_params_file, {'use_sim_time': True}],
    remappings=[
      ("imu", "imu"),
      ("gps/fix", "navsat/fix"),
      ("odometry/gps", "odometry/gps"),
      ("odometry/filtered", "odometry/filtered/global"),
    ],
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_file],
    output='screen',
    parameters=[{'use_sim_time': True}],
  )

  return LaunchDescription([
    bring_up,
    ekf_filter_node_odom,
    ekf_filter_node_map,
    navsat_transform_node,
    # rviz_node
  ])