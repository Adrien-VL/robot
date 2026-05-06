import os
import yaml

import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PKG_NAME = "dev"

def pkg_dir():
  return get_package_share_directory(PKG_NAME)

def nav2_dir():
  return get_package_share_directory("nav2_bringup")

def robot_description():
  xacro_file = os.path.join(pkg_dir(), "description/robot.urdf.xacro")
  return xacro.process_file(xacro_file).toxml()

def config(filename):
  return os.path.join(pkg_dir(), "config", filename)

def world(filename):
  return os.path.join(pkg_dir(), "worlds", filename)

def map(filename):
  return os.path.join(pkg_dir(), "map", filename)

def use_sim_time():
  with open(config("common.yaml"), "r") as f:
    common = yaml.safe_load(f)
  return str(common.get("/**", {}).get("ros__parameters", {}).get("use_sim_time", False)).lower()

# ********* Nodes ********* #

def make_rsp_node():
  return Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[
      config("common.yaml"),
      config("rsp.yaml"),
      {"robot_description": robot_description()},
    ]
  )

def make_rviz_node():
  return Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="screen",
    arguments=["-d", os.path.join(pkg_dir(), "rviz/robot.rviz")],
    parameters=[config("common.yaml")],
  )

def make_jsp_gui_node():
  return Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
    name="joint_state_publisher_gui",
    output="screen",
    parameters=[
      {"robot_description": robot_description()},
      {"use_sim_time": False},
    ]
  )

def make_spawn_node(entity_name="robot"):
  return Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    arguments=["-topic", "robot_description", "-entity", entity_name],
    output="screen",
  )

def make_gazebo(world_file=None):
  args = {}
  if world:
    args["world"] = world(world_file)
  return IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch", "gazebo.launch.py"
      )
    ]),
    launch_arguments=args.items(),
  )

def make_slam_toolbox(mapping=True, map_name="main"):
  config_file = "slam_online_async.yaml" if mapping else "localization_online_async.yaml"

  parameters = [
    config("common.yaml"),
    config(config_file),
  ]

  if not mapping:
    parameters.append({"map_file_name": map(map_name)})

  return Node(
    package="slam_toolbox",
    executable="async_slam_toolbox_node",
    name="slam_toolbox",
    output="screen",
    parameters=parameters
  )

def make_periodic_map_saver(filename="latest", period=30):
  save_path = map(filename)
  return ExecuteProcess(
    cmd=["bash", "-c",
      f"while true; do "
      f"ros2 run nav2_map_server map_saver_cli -f {save_path} "
      f"--ros-args "
      f"-p map_subscribe_transient_local:=true "
      f"-p free_thresh_default:=0.25 "
      f"-p occupied_thresh_default:=0.65 "
      f"-p save_map_timeout:=5.0; "
      f"sleep {period}; done"
    ],
    output="screen",
  )

def make_periodic_map_serializer(filename="latest", period=60):
  """
  Saves the massive .posegraph and .data files needed for other localization sources, or to resume mapping later.
  Because serialization is a heavy CPU process, it's recommended to do it less frequently.
  """
  save_path = map(filename)
  return ExecuteProcess(
    cmd=["bash", "-c",
      f"while true; do "
      f"ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{{filename: '{save_path}'}}\"; "
      f"sleep {period}; done"
    ],
    output="screen",
  )

def make_nav2():
  nav2_launch_file = os.path.join(nav2_dir(), "launch", "navigation_launch.py")

  return IncludeLaunchDescription(
    PythonLaunchDescriptionSource(nav2_launch_file),
    launch_arguments={
      "use_sim_time": use_sim_time(),
      "params_file": config("nav2.yaml"),
    }.items(),
  )

def make_lidar_node():
  return Node(
    package="lidar",
    executable="lidar",
    name="lidar",
    output="screen",
    parameters=[ 
      config("lidar.yaml")
    ]
  )

def make_diff_node():
  return Node(
    package="diff",
    executable="diff",
    name="diff",
    output="screen",
    parameters=[
      config("diff.yaml"),
    ],
    remappings=[
      ("cmd_vel", "cmd_vel"),   # make sure it matches Nav2
      ("odom", "/odom"),
    ]
  )