from launch import LaunchDescription
from dev.launch import make_rsp_node, make_diff_node, make_lidar_node, make_nav2, make_slam_toolbox

def generate_launch_description():
  return LaunchDescription([
    make_rsp_node(),
    make_diff_node(),
    make_lidar_node(),
    make_slam_toolbox(),
    make_nav2(),
  ])