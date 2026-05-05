from launch import LaunchDescription
from dev.launch import make_rsp_node, make_diff_node

def generate_launch_description():
  return LaunchDescription([
    make_rsp_node(),
    make_diff_node(),
  ])