# ros2-system-monitor

## Synopsis

System monitoring tools for ROS2.



## Description

This project provides system monitoring tools for ROS2 in the form of the following monitors:

* CPU monitor
* Disk monitor
* Memory monitor
* Network monitor

All data is published on ROS2 topic `/diagnostics` which can conveniently be visualized
in the runtime monitor.



## Build

Use colcon to build the package.

The main script is system_monitor.py.

- Clone the repository to your `~/ros2_ws/src`.

  ```sh
  git clone https://github.com/lemon19900815/ros2_system_monitor.git
  ```

  

- Change to `~/ros2_ws/`, then run `colcon build --packages-select system_monitor` to build.

  ```sh
  cd ~/ros2_ws/
  colcon build --packages-select system_monitor
  ```

  

- Run system_monitor.

  ```sh
  source install/setup.bash
  ros2 run system_monitor system_monitor
  ```

  

- Echo the topic data.

  ```sh
  # you will see the system_monitor topic
  ros2 topic list
  
  ros2 topic echo /diagnostics
  ```

  

**Note:**

1. The script system_monitor.py must put under the directory system_monitor.