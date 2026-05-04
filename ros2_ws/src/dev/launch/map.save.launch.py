from launch import LaunchDescription
from dev.launch import make_periodic_map_saver, make_periodic_map_serializer

def generate_launch_description():
  return LaunchDescription([
    make_periodic_map_saver(filename="latest", period=30),
    make_periodic_map_serializer(filename="latest_serial", period=60),
  ])