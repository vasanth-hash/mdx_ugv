# MDX_UGV

This repository contains for outdoor navigation using robot localization package ros2 for the 6x6.

## Requirements
* ROS2 - Humble
* Ubuntu - 22.04
* Ignition gazebo fortress
* teb_local_planner package

## Execution

* Create a workspace, and clone this repo inside the src directly and build it using `colcon` and source the workspace

    ```bash
    cd your_ws/
    colcon build --symlink-install
    source install/setup.bash
    ```

* To launch the robot in gazebo, execute the robot localization and the navigation, run this command

    ```bash
    cd your_ws/
    ros2 launch mdx_uv navigation.launch.py
    ```
