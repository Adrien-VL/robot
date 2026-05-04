# Static Transforms

Static transforms in ROS define unchanging spatial relationships between coordinate frames, such as a camera mounted rigidly to a robot base.

In the transform package (TF) static transforms are published to `/tf_static`.

# Dynamic Transforms

**Used packages:**
- ros-humble-xacro
- ros-humble-joint-state-publisher-gui

Dynamic transform trees are defined using URDF files.

**Tree:** A tree of relative transforms (relative position + orientation from parent) where each node has one incoming reference from a parent transform except for the root.

A tree cannot have cycles. You can transform one transform/frame to the position of the other by accumulating the relative transforms on the path towards it (up and down the tree). Editing a parent transform also applies that relative transform to all children with respect to the parent reference frame.

Although the transform package (TF) uses topics behind the scenes this is abstracted by broadcastening transforms and listening to transforms.

A tree in the transform package (TF) consists out of (reference) frames connected by transforms.

In the transform package (TF) dynamic transforms are published to `/tf`.

# URDF

In an URDF file a robot is made out of tree composed out of links connected by joints (the relations between links).

This is conceptually equivalent to the tree of the transform package (TF). This is also partially why there is a ros node called `robot_state_publisher` that takes in a URDF file and automatically broadcasts all transform equivalents from it. A copy of the URDF file is published to the `/robot_description` topic.

A joint can be either defined as fixed or as one of the movable types. For fixed joints the `robot_state_publisher` will simply publish a static transform.

For dynamic joints the `robot_state_publisher` subscribes to `/joint_states` to update the transforms of the dynamic joints at each point in time.

The `joint_state_publisher_guid` parses an URDF, finds any dynamic variables and shows these in an interactive GUI. On change of the variable values it publishes the changes to `/joint_states`. This package expects the whole URDF file as a parsed string for which `xacro` is used.

Instead of having to publish whole transforms we now only need to publish to `/joint_states`. Real actuator feedback also publishes to `/joint_states`. Durint simulation it is simulated actuator feedback that gets published.

**Note:** Everytime you make a change to an URDF file being used you will need to restart `robot_state_publisher`.

# View_frames

`view_frames` is part of the the `tf2_tools` package. It listens to transforms and outputs the derived transform tree into a pdf.

```bash
ros2 run tf2_tools view_frames
```