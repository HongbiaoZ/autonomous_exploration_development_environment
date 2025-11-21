import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():
  cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
  vehicleX = LaunchConfiguration('vehicleX')
  vehicleY = LaunchConfiguration('vehicleY')
  checkTerrainConn = LaunchConfiguration('checkTerrainConn')
  
  declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='')
  declare_vehicleX = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
  declare_vehicleY = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
  declare_checkTerrainConn = DeclareLaunchArgument('checkTerrainConn', default_value='true', description='')
  
  start_local_planner = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')
    ),
    launch_arguments={
      'cameraOffsetZ': cameraOffsetZ,
      'goalX': vehicleX,
      'goalY': vehicleY,
    }.items()
  )

  start_terrain_analysis = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
    )
  )

  start_terrain_analysis_ext = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
    ),
    launch_arguments={
      'checkTerrainConn': checkTerrainConn,
    }.items()
  )

  start_sensor_scan_generation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch.py')
    )
  )

  start_loam_interface = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('loam_interface'), 'launch', 'loam_interface_launch.py')
    )
  )

  start_fast_lio = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('fast_lio'),'launch','mapping.launch.py')
    )
  )

  static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
      "--x",
      "0.0",
      "--y",
      "0.0",
      "--z",
      "0.2",
      "--roll",
      "0.0",
      "--pitch",
      "0.0",
      "--yaw",
      "0.0",
      "--frame-id",
      "base_link",
      "--child-frame-id",
      "front_mid360",
    ],
  )

  map_static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
      "--x",
      "0.0",
      "--y",
      "0.0",
      "--z",
      "0.2",
      "--roll",
      "0.0",
      "--pitch",
      "0.0",
      "--yaw",
      "0.0",
      "--frame-id",
      "map",
      "--child-frame-id",
      "odom",
    ],
  )

  vehicle_static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
      "--x",
      "0.0",
      "--y",
      "0.0",
      "--z",
      "0.0",
      "--roll",
      "0.0",
      "--pitch",
      "0.0",
      "--yaw",
      "0.0",
      "--frame-id",
      "base_link",
      "--child-frame-id",
      "vehicle",
    ],
  )
  start_joy = Node(
    package='joy', 
    executable='joy_node',
    name='ps3_joy',
    output='screen',
    parameters=[{
                'dev': "/dev/input/js0",
                'deadzone': 0.12,
                'autorepeat_rate': 0.0,
  		}]
  )

  rviz_config_file = os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'vehicle_simulator.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
  )

  delayed_start_rviz = TimerAction(
    period=2.0,
    actions=[
      start_rviz
    ]
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_cameraOffsetZ)
  ld.add_action(declare_vehicleX)
  ld.add_action(declare_vehicleY)
  ld.add_action(declare_checkTerrainConn)

  ld.add_action(start_local_planner)
  ld.add_action(start_terrain_analysis)
  ld.add_action(start_terrain_analysis_ext)
  ld.add_action(start_sensor_scan_generation)
  ld.add_action(start_loam_interface)
  ld.add_action(start_fast_lio)
  ld.add_action(start_joy)
  ld.add_action(delayed_start_rviz)
  ld.add_action(static_tf)
  ld.add_action(map_static_tf)
  ld.add_action(vehicle_static_tf)
  return ld
