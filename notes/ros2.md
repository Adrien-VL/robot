You can either use the general `setup.sh` script but some other useful commands in orde are.

```bash
pixi install -e humble
colcon build --symlink-install
source ./install/setup.zsh
pixi shell -e humble
```

The order of parameters dict have priority:
```python
robot_state_publisher_node = Node(
  package="robot_state_publisher",
  executable="robot_state_publisher",
  output="screen",
  parameters=[
    common_yaml,           # 1st (lowest priority)
    rsp_yaml,              # 2nd
    {"robot_description": robot_description_raw}  # 3rd (highest priority)
  ]
)
```