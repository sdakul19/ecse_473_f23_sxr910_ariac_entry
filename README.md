# How to use ariac_entry package

## How the Package Works
The ariac_entry package is a slightly edited version of the 2019 ARIAC competion. Aside from minor customizations, all components remain the same.
This includes messages, services, order parts, etc. The documentation from the competition can be found [here](https://bitbucket.org/osrf/ariac/wiki/2019/documentation).

### Launch
The launch directory contains the launch file which launches the [ecse_373_ariac](https://github.com/cwru-eecs-373/ecse_373_ariac/tree/noetic-devel/ecse_373_ariac) launch file. <br>
It also launches the ariac_entry node using the ariac_entry source code.

### Msg
The message directory contains two messages, Bin.msg and Bins.msg. 

#### Bin.msg
Bin.msg has three elements.
```
# Bin name
string bin_name

# Material type
string material_type

# Camera Image Information
osrf_gear/LogicalCameraImage image
```
For bins 1 through bin 6, each bin will be identified by their bin number, material_type, and the image data. The current environment has bins 1-3 empty so their material_type is listed as any and there is no image data. Bin 4 has the piston_rod_part material and currently has 9 parts. The camera image data should contain the pose of each part. Bin 5 has the gasket_part and currently has 6 parts. Bin 6 has the gear_part and currently has 12 parts.

#### Bins.msg
The Bins.msg has only one element which is a vector of bins.
```
# Collection of bins
Bin[] bins
```
The Bins.msg is for publishing to the topic /bins_contents. Currently the bin vector is only initialized once and needs to be modified to update everytime a part is taken out of the bins. To see the bin contents: > ```rostopic echo /bin_contents```
### Src

## Setup Before Launching

### Install Simulation Environment
In order to launch "entry.launch", the simulator environment must be first installed.<br>
For the course of ECSE_473, the simulation environment was cloned from a class repository: [cwru_ariac_2019](https://github.com/cwru-eecs-373/cwru_ariac_2019).
```
# Create a catkin workspace for the simulation environment
mkdir -p ~/ariac_ws/src
cd ~/ariac_ws/src
# Clone the repository
git clone https://github.com/cwru-eecs-373/cwru_ariac_2019.git
# Install any missing dependencies
rosdep install --from-paths ariac --ignore-src -r -y
# Build the simulator environment
cd ../
# Install the simulator environment
sudo -- /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make
-DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install"
```
### Install Working Package for Class Assignment
Along with cwru_ariac_2019, another package, [ecse_373_ariac](https://github.com/cwru-eecs-373/ecse_373_ariac/tree/noetic-devel/ecse_373_ariac), needs to be installed.<br>
Currently the ARIAC Simulation needs to be called from another package.<br>
This package also contains other customizations to fit the course material better.<br>
```
# Make a workspace for the ARIAC node.
mkdir -p ~/ecse_373_ariac_ws/src
cd ~/ecse_373_ariac_ws/src
# Clone the GIT repository for this laboratory.
git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git
# Install any missing dependencies.
rosdep install --from-paths ecse_373_ariac --ignore-src -r -y
# Add it to your ROS environment.
cd ../
catkin_make
source devel/setup.bash
```
### empy bug
Ubuntu Focal currently has a bug with the empy module for python3.<br>
There are two solutions for this labratory.<br>

#### Use Static Files in ecse_373_ariac Package
The first method is to use the static files from the ecse_373_ariac package.
This is done by setting the python argument to false.
```
# This is the static file workaround for the Python3 empy module bug in Ubuntu Focal.
roslaunch ecse_373_ariac ecse_373_ariac.launch python:=false
```

#### Direct Patch with Administrative Privileges
The second method is the preferred method to fixing the empy bug.<br>
```
# Path the em.py file of the Python3 empy module.
sudo patch /usr/lib/python3/dist-packages/em.py < `rospack find
ecse_373_ariac`/patches/empy.patch
```
## How to Launch
After all necessary packages are installed and ROS is aware of all packages, the launch file should work properly.<br>
```
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch ariac_entry entry.launch
```
