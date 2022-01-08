# ROS2 Lifecycle

This repository demonstrates the usage of ROS2 lifecycle nodes. 

![img](resources/life_cycle_sm.png)

This demo is a recreation of this [demo](https://github.com/ros2/demos/tree/master/lifecycle) from ROS2.

Primary States (steady states):

- unconfigured
- inactive
- active
- shutdown

Transition States (intermediate states):

- configuring
- activating
- deactivating
- cleaningup
- shuttingdown

The possible transitions to invoke are:

- configure
- activate
- deactivate
- cleanup
- shutdown

## Running Demo

Use the bash script in tmux directory to run the full demo.

```bash
./start_demo.bash
```

## Reference

- https://index.ros.org/p/lifecycle/
- https://design.ros2.org/articles/node_lifecycle.html
- https://github.com/ros-planning/navigation2/tree/main/nav2_lifecycle_manager
- https://github.com/ros2/demos/tree/master/lifecycle