from launch import LaunchDescription
from dev.launch import make_rsp_node, make_diff_node, make_lidar_node, make_nav2, make_sync_relay_node, make_slam_toolbox, make_rf2o_node

def generate_launch_description():
  return LaunchDescription([
    make_rsp_node(),
    make_diff_node(),
    make_lidar_node(),
    # make_sync_relay_node(),
    # make_rf2o_node(),
    make_slam_toolbox(),
    make_nav2(),
  ])