from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from dev.launch import (
  make_gazebo,
  make_rsp_node,
  make_rviz_node,
  make_spawn_node,
  make_slam_toolbox,
  make_nav2
)

def launch_setup(context, *args, **kwargs):
  mapping_str = LaunchConfiguration("mapping").perform(context)
  is_mapping = mapping_str.lower() in ["true", "t", "1", "yes"]
  print(is_mapping)

  return [
    make_gazebo(world_file="main.world"),
    make_rsp_node(),
    make_rviz_node(),
    make_spawn_node(),
    make_slam_toolbox(mapping=is_mapping, map_name="main"),
    make_nav2()
  ]


def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument(
      "mapping",
      default_value="true",
      description="Run SLAM Toolbox in mapping mode (true) or localization mode (false)"
    ),

    OpaqueFunction(function=launch_setup)
  ])