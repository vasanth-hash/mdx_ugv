import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'mdx_ugv'
  pkg_share = get_package_share_directory(package_name)

  xacro_file_path = os.path.join(pkg_share, 'description', 'mdx_ugv.urdf')
  world_file = os.path.join(pkg_share, "worlds", "sand_heightmap_boxes.sdf")
  controllers_file = os.path.join(pkg_share, 'config', 'controllers.yaml')
  bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
  robot_description = Command(['xacro ', xacro_file_path])
  
  rviz_launch = LaunchConfiguration('rviz_launch', default='true') 

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
    output='screen',
    parameters=[{'use_sim_time': True}],
    condition=IfCondition(rviz_launch),
  )

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
  )
  
  # Include the Gazebo launch file, provided by the ros_gz_sim package
  gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': ['-r -v4 ', world_file], 'on_exit_shutdown': 'true', 'use_sim_time': 'true'}.items()
                # launch_arguments={'gz_args': ['-r -v4 empty.sdf', ], 'on_exit_shutdown': 'true', 'use_sim_time': 'true'}.items()
  )

  # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
  spawn_entity = Node(package='ros_gz_sim', executable='create',
                      parameters=[{'use_sim_time': True}],
                      arguments=['-topic', 'robot_description',
                                  '-name', 'mdx_ugv',
                                  # '-x', '-3.42', # sonama
                                  # '-y', '2.64',
                                  # '-z', '0.47',
                                  # '-Y', '-0.65'], 
                                  # '-x', '144.1',   # santorini_scaled
                                  # '-y', '-326.5', 
                                  # '-z', '9.286',
                                  # '-Y', '-2.65'], 
                                  # '-x', '-29.0', #'0',   # sand_heightmap_boxes
                                  # '-y', '-97.9', #'0', 
                                  # '-z', '0.0', #'0',
                                  # '-Y', '3.0'], # '3.12'],
                                   '-x', '0.0', #'0',   # sand_heightmp_boxes dynamic obstacle
                                   '-y', '0.0', #'0', 
                                   '-z', '0.0', #'0',
                                   '-Y', '0.0'], # '3.12'],
                      output='screen'
  )
  
  ros_gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'use_sim_time': True}],
    arguments=[
      '--ros-args',
      '-p',
      f'config_file:={bridge_params}',
    ],
    output='screen'
  )

  return LaunchDescription([
    rviz_node,
    robot_state_publisher,
    gazebo,
    spawn_entity,
    ros_gz_bridge
  ])

