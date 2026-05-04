from launch import LaunchDescription
from dev.launch import make_jsp_gui_node, make_rviz_node


def generate_launch_description():
  return LaunchDescription([
    make_jsp_gui_node(),
    make_rviz_node(),
  ])