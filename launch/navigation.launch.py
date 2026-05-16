import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import ReplaceString

def generate_launch_description():
  package_name = 'mdx_ugv'
  pkg_share = get_package_share_directory(package_name)
  
  nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
  rviz_file = os.path.join(pkg_share, 'rviz', 'navigation.rviz')  
  dyn_obs_params_file = os.path.join(pkg_share, 'config', 'dynamic_obstacle_params.yaml')  
  autostart = LaunchConfiguration('autostart')
  
  remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
  
  lifecycle_nodes_nav2 = [
    'controller_server', 'smoother_server', 'planner_server',
    'behavior_server', 'velocity_smoother', 'bt_navigator', 
    'waypoint_follower'
  ]
  
  # Declare launch arguments
  declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
  declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Auto start nav2 stack')

  dual_ekf_navsat = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','dual_ekf_navsat.launch.py'
            )]),
            launch_arguments={'rviz_launch': 'false'}.items() 
  )

  # Start RViz
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_file],
    output='screen',
    parameters=[{'use_sim_time': True}],
  )
  
  # Start Nav2 Nodes
  nav2_controller = Node(
    package='nav2_controller',
    executable='controller_server',
    name='controller_server',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': True}],
  )
  
  nav2_smoother = Node(
    package='nav2_smoother',
    executable='smoother_server',
    name='smoother_server',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': True}],
    remappings=remappings,
  )
  
  nav2_planner = Node(
    package='nav2_planner',
    executable='planner_server',
    name='planner_server',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': True}],
    remappings=remappings,
  )
  
  nav2_behaviors = Node(
    package='nav2_behaviors',
    executable='behavior_server',
    name='behavior_server',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': True}],
    remappings=remappings,
  )
  
  nav2_bt_navigator = Node(
    package='nav2_bt_navigator',
    executable='bt_navigator',
    name='bt_navigator',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': True}],
    #remappings=remappings,
    remappings=[
      ('/tf', 'tf'),
      ('/tf_static', 'tf_static'),
      #('/goal_pose', '/nav2_blocked_goal_pose'),
    ],
  )
  
  nav2_waypoint_follower = Node(
    package='nav2_waypoint_follower',
    executable='waypoint_follower',
    name='waypoint_follower',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': True}],
    remappings=remappings,
  )
  
  nav2_velocity_smoother = Node(
    package='nav2_velocity_smoother',
    executable='velocity_smoother',
    name='velocity_smoother',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': True}],
    remappings=remappings,
  )
  
  nav2_collision_monitor = Node(
    package='nav2_collision_monitor',
    executable='collision_monitor',
    name='collision_monitor',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': True}],
    remappings=remappings,
  )
  
  nav2_route_server = Node(
    package='nav2_route',
    executable='route_server',
    name='route_server',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': True}],
    remappings=remappings,
  )
  
  # Lifecycle Manager for Nav2
  nav2_lifecycle_nodes_manager = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    output='screen',
    parameters=[{'autostart': autostart}, 
                {'node_names': lifecycle_nodes_nav2}, 
                {'use_sim_time': True}],
  )
  
  interpolator = Node(
    package='mdx_ugv',
    executable='interpolator',
    name='interpolator',
    output='screen',
  )
  
  #python file for dynamic obstacle processing
  dynamic_obstacle_processor_py = Node(
    package='mdx_ugv',
    executable='dynamic_obstacle_processor.py',
    name='dynamic_obstacle_processor',
    output='screen',
  )
  
  #cpp file for dynamic obstacle processing
  dynamic_obstacle_processor_cpp = Node(
    package='mdx_ugv',
    executable='dynamic_obstacle_processor',
    name='dynamic_obstacle_processor',
    output='screen',
    parameters=[dyn_obs_params_file, {'use_sim_time': True}],
  )
  
  #cpp file for dynamic obstacle processing
  waypoint_manager_node = Node(
    package='mdx_ugv',
    executable='waypoint_manager',
    name='waypoint_manager',
    output='screen',
    parameters=[{'use_sim_time': True}],
  )

  ld = LaunchDescription()

  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_autostart_cmd)

  ld.add_action(dual_ekf_navsat)
  ld.add_action(rviz_node)
  ld.add_action(nav2_controller)
  ld.add_action(nav2_smoother)
  ld.add_action(nav2_planner)
  ld.add_action(nav2_behaviors)
  ld.add_action(nav2_bt_navigator)
  ld.add_action(nav2_waypoint_follower)
  ld.add_action(nav2_velocity_smoother)
  #ld.add_action(nav2_collision_monitor)
  # ld.add_action(nav2_route_server)
  ld.add_action(nav2_lifecycle_nodes_manager)
  #ld.add_action(interpolator)
  #ld.add_action(dynamic_obstacle_processor_py)
  #ld.add_action(dynamic_obstacle_processor_cpp)
  #ld.add_action(waypoint_manager_node)

  return ld
