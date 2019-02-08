# flash_robot

ROS integration for the [FLASH MK II](https://www.edinburgh-robotics.org/equipment/robotarium-west-field-systems-humanoid/flash-robot). The stack is structured according to the [TIAGO robot ROS Integration](https://github.com/pal-robotics/tiago_robot).

<img src="https://s3.amazonaws.com/poly-screenshots.angel.co/Project/85/245278/1480f251a33df7bb4f17c40061490969-original.png" width="350"/>

## Table of Contents

- [System Requirements](#system-requirements)
- [Prerequisites](#prerequisites)
  - [Python Version](#python-version)
  - [Odometry](#odometry)
  - [Battery Level in RViz](#battery-level-in-rviz)
- [Building the Package](#building-the-package)
- [Usage](#usage)
  - [flash_2dnav](#flash_2dnav)
  - [flash_behaviors](#flash_behaviors)
  - [flash_bringup](#flash_bringup)
  - [flash_controller](#flash_controller)
  - [flash_description](#flash_description)
  - [flash_maps](#flash_maps)
  - [flash_odom](#flash_odom)
  - [flash_robot](#flash_robot)
- [Developers](#developers)

## System Requirements

The full stack has only been tested on Ubuntu 16.04 and ROS Kinetic!

## Prerequisites

### Python Version

Most of the code should be compatible with Python3. The only exception is the navigation.py script which depends on the tf package. If you want to make it compatible with Python3, then you should also build tf with Python3 (see this [thread](https://github.com/ros/geometry2/issues/259)).

In order to install the Python3 required dependencies, we recommend creating a virtual environment and using the provided `requirements.txt` file (see [Building the Package](#building-the-package)).

### Odometry

The robot odometry is computed based on planar laser scans using the [rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry) package. The official package has been released only for ROS Indigo, hence you will need to clone the repository and build it within your ROS workspace (use provided fork):

```sh
cd <YOUR_PATH>/kinetic_ws/src
git clone https://github.com/joselpart/rf2o_laser_odometry.git
cd ..
catkin_make
```

### Battery Level in RViz

In order to show the battery level in RViz, the [topics_rviz_plugin](https://gitlab.com/InstitutMaupertuis/topics_rviz_plugin) package is used:

```sh
cd <YOUR_PATH>/kinetic_ws/src
git clone https://gitlab.com/InstitutMaupertuis/topics_rviz_plugin.git
cd topics_rviz_plugin
git checkout kinetic
cd ../..
catkin_make
```

## Building the Package

In the directory where you want to create your Python3 virtual environment do:

```sh
python3 -m venv python3_venv
source python3_venv/bin/activate
```

Then, clone the flash_robot stack, install the dependencies and build it:

```sh
cd <YOUR_PATH>/kinetic_ws/src
git clone https://github.com/BrutusTT/flash_robot.git
pip install -r flash_robot/requirements.txt
cd ..
catkin_make
```

## Usage

If you followed the previous instructions, then your ROS workspace should look something like this:

```
kinetic_ws/
  src/
    CMakeLists.txt
    flash_robot/
      README.md
      requirements.txt
      flash_2dnav/
      flash_behaviors/
      flash_bringup/
      flash_controller/
      flash_description/
      flash_maps/
      flash_odom/
      flash_robot/
    rf2o_laser_odometry/
    topics_rviz_plugin/
  build/
  devel/
```

### flash_2dnav

This package contains configuration files and launch files for navigation and mapping. We use [gmapping](http://wiki.ros.org/gmapping?distro=kinetic) for mapping and amcl for localization, available from the [navigation](http://wiki.ros.org/navigation?distro=kinetic) stack. You can change the configuration of the planners and the localization algorithm in the config files provided.

If you need to create a new map, then run:

```sh
roslaunch flash_2dnav mapping.launch
```

Then, you can use a joystick to teleoperate the robot around (see [flash_controller](#flash_controller)). Once you are happy with the map, then you can save it:

```sh
# If you don't provide a directory, it will be saved in the current directory.
rosrun map_server map_saver -f <MAP_NAME>
```

To navigate in the created map, first change the corresponding parameter in `move_base.launch` to point to your map. In this file, you can also change the local planner to be used. Then, you can run it and provide goals using the available RViz plugin:

```
roslaunch flash_2dnav move_base.launch
```

The package also contains a python script `navigate.py` with scripted waypoints (with respect to the provided maps) and behaviors. It should be run after `move_base.launch` has been run:

```sh
# Make sure your Python3 environment has been deactivated unless you have built tf with Python3!
rosrun flash_2dnav navigate.py
```

### flash_behaviors

This package contains various action servers for speech, behaviors, gestures, etc. It also contains a configuration file that defines a series of URBI functions that are loaded onto the robot when the action server starts.

The launch file available here is meant to set up the system to run experiments since it starts all the necessary components.

### flash_bringup

This package contains the joystick configuration file and a few demo launch files.

### flash_controller

This package contains all the interfaces between ROS and URBI and the nodes that publish sensor data and subscribe to commands.

### flash_description

This package contains description files for the flash robot. At the moment, these are dummy files mainly for visualization in RViz.

### flash_maps

Where the maps should be stored.

### flash_odom

This package contains a launch file that configures and runs the odometry node.

### flash_robot

Metapackage.

## Developers

- [Ingo Keller](https://github.com/BrutusTT)
- [Jose L. Part](https://github.com/joselpart)
