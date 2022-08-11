# Introduction
These instruction provide information on how to manually install the software, if you don't use the Docker version.

# Prerequisites
**Operating system:** Tested with Ubuntu 20.04.4 LTS. [This](https://docs.ros.org/en/galactic/Releases/Release-Galactic-Geochelone.html#supported-platforms) lists other platforms that might work, but they have not been tested.

**ROS** ROS2 Galactic Geochelone. [Installation instructions](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages), use the ros-galactic-desktop package.

**Python** Tested with Python 3.8.10, which is the base version of Ubuntu 20. 3.10 does not yet work, there are incompatibilties with ROS2 (tried it out but can't remember what exactly wasn't ported yet).

# Preparations
Create a base folder anywhere you like. Inside this base folder, clone the following repo:

## RMF
For RMF, follow the [instructions](https://github.com/open-rmf/rmf) for building from sources.
Installing it in the mentioned base folder is recommended.

Then, update one file manually: `rmf_ws/src/rmf/rmf_traffic_editor/rmf_building_map_tools/building_map/building.py`

At the end of the function generate_sdf_world(), just before the return statement (`return sdf`), add the following (with the correct indentation):
```
# add custom gazebo plugin for grid creation
grid_creator_ele = SubElement(
    world,
    'plugin',
    {'name': 'grid_creator_plugin',
        'filename': 'libgrid_creator_plugin.so'})
```

Install the free_fleet packages, which are not included by default:
```
cd rmf_ws/src
git clone https://github.com/open-rmf/free_fleet.git
```

## Navigation2 and turtlebot3
`sudo apt install ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-nav2-gazebo-spawner ros-galactic-turtlebot3*`

## libMultiRobotPlanning
Clone and build according to instructions at https://github.com/whoenig/libMultiRobotPlanning

## mrp_bench
 - `git clone https://github.com/boschresearch/mrp_bench`

Enter the newly cloned folder: `cd mrp_bench`. Add a symbolic link to rmf in mrp_bench. Example: 
`ln -s /home/username/bench/rmf_ws/ rmf_ws`

This will cause RMF to be built together with mrp_bench.

Update all paths in `mrp_bench/bench_ws/bench_pkg/bench_pkg/param/config.yaml` according to your local setup.
This includes the external libraries, as well as the place to store generated bags (experiments).
If new parameters need to be added, this will be done in base_config.yaml. config.yaml is ignored by git.

Copy the map pgms from TODO to `launch/maps` if you don't want to generate them yourself. 

Then build everything by running `colcon build` from the mrp_bench folder. There will be some stderr output in the RMF packages at the first build. At least with the ones I got, this does not cause any problems.

## Google OR-Tools
Install with pip:
`python3 -m pip install --upgrade --user ortools`

## Other Python packages
`python3 -m pip install --upgrade --user dotmap networkx pandas`

Other packages as per error messages, if they're not part of your Python installation.
