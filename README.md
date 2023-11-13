# How to use ariac_entry package

## How the Package Works
The ariac_entry package is a slightly edited version of the 2019 ARIAC competion. Aside from minor customizations, all components remain the same.
This includes messages, services, order parts, etc. The documentation from the competition can be found [here](https://bitbucket.org/osrf/ariac/wiki/2019/documentation).

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
