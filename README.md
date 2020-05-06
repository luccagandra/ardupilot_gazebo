# Ardupilot Gazebo Plugin & Models

## Requirements

* Ubuntu 18.04 LTS (Bionic Beaver)
* ROS Melodic
* Gazebo 9

## Installation

Installation instructions are based on [this](https://docs.google.com/document/d/1CIObvL-HHYCosqRLb9j_OIzAPuPPpV2wvajG1nY-UlY/edit?usp=sharing) document. Special thanks to [@AnaBatinovic](https://github.com/AnaBatinovic).

### Various packages

```bash
pip install -U pymavlink
pip install -U mavproxy
sudo apt install libgeographic-dev
sudo apt install ros-melodic-mavlink
sudo apt install ros-melodic-mavros ros-melodic-mavros-msgs
sudo apt install ros-melodic-octomap-ros ros-melodic-joy
sudo apt install python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev
sudo apt install git gitk git-gui
```

### GeographicLib

```bash
sudo apt install geographiclib-doc geographiclib-tools libgeographic17 node-geographiclib
cd /opt/ros/melodic/lib/mavros/
sudo ./install_geographiclib_datasets.sh
```

Note: If building Mavros from source locate the *install_geographiclib_datasets.sh* script inside your local Mavros source folder.

### Ardupilot

```bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
cd Tools/environment_install
./install-prereqs-ubuntu.sh -y
echo "export PATH=$PATH:$HOME/ardupilot/Tools/autotest" >> ~/.bashrc
echo "export PATH=/usr/lib/ccache:$PATH" >> ~/.bashrc
source ~/.bashrc
```

For additional information see: [Setting up the Build Environment(Linux/Ubuntu).](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)

### Catkin workspace setup

Navigate to your catkin workspace or create it as follows.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/melodic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ~/catkin_ws
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Download and install the following catkin packages.

[rotors_simulator](https://github.com/larics/rotors_simulator)

```bash
cd ~/catkin_ws/src
git clone https://github.com/larics/rotors_simulator
cd rotors_simulator
git checkout larics_melodic_master
```

[mav_comm](https://github.com/larics/mav_comm)

```bash
cd ~/catkin_ws/src
git clone https://github.com/larics/mav_comm
cd mav_comm
git checkout larics_master
```

[ardupilot_gazebo](https://github.com/larics/ardupilot_gazebo)

```bash
cd ~/catkin_ws/src
git clone https://github.com/larics/ardupilot_gazebo
cd ardupilot_gazebo
git checkout larics_master
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=~/catkin_ws/build/ardupilot_gazebo:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
source ~/.bashrc
```

Donwload optional packages:

* [storm_gazebo_ros_magnet](https://github.com/larics/storm_gazebo_ros_magnet/tree/melodic_electromagnet_dev) - electromagnet plugin for Gazebo
* [velodyne_simulator](https://github.com/lmark1/velodyne_simulator) - velodyne plugin for Gazebo

Finally build your catkin workspace.

```bash
cd ~/catkin_ws
catkin build
source ~/.bashrc
```

## Simulation Startup

**Note**: Make sure that you start the *sim_vehicle.py* script for each vehicle from their respective folder.

### Bebop

````bash
# 1st terminal
mkdir -p ~/bebop_startup
cd ~/bebop_startup
sim_vehicle.py -v ArduCopter -f gazebo-iris -m --mav10 --console --map -I0 -m --streamrate=50

# 2nd terminal
roslaunch ardupilot_gazebo bebop.launch

# 3rd terminal
roslaunch ardupilot_gazebo mavros.launch
````

### Ardrone

````bash
# 1st terminal
mkdir -p ~/bebop_startup
cd ~/bebop_startup
sim_vehicle.py -v ArduCopter -f gazebo-iris -m --mav10 --console --map -I0 -m --streamrate=50

# 2nd terminal
roslaunch ardupilot_gazebo ardrone.launch

# 3rd terminal
roslaunch ardupilot_gazebo mavros.launch
````

### Rover

````bash
# On 1st Terminal(Launch Ardupilot SITL)
sim_vehicle.py -v APMrover2 -f gazebo-rover  -m --mav10 --map --console -I1

# On 2nd Termianal(Launch Gazebo with differential drive Rover model, Retrieved from Husky Model)
gazebo --verbose rover_ardupilot.world
````

### Kopterworx

````bash
# 1st terminal
mkdir -p ~/kopter_startup
cd ~/kopter_startup
sim_vehicle.py -v ArduCopter --add-param-file=$HOME/catkin_ws/src/ardupilot_gazebo/config/kopterworx_red.parm -f gazebo-iris -m --mav10 --console -I0 -m --streamrate=50

# 2nd terminal
roslaunch ardupilot_gazebo kopterworx.launch

# 3rd terminal
roslaunch ardupilot_gazebo mavros.launch
````

### 3DR IRIS Copter

````bash
# On 1st Terminal(Launch Ardupilot SITL)
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 --map --console -I0

# On 2nd Terminal(Launch Gazebo with demo 3DR Iris model)
gazebo --verbose iris_ardupilot.world
````

### Plane

````bash
# On 1st Terminal(Launch Ardupilot SITL)
sim_vehicle.py -v ArduPlane -f gazebo-zephyr  -m --mav10 --map --console -I0

# On 2nd Terminal(Launch Gazebo with demo Zephyr flying wing model)
gazebo --verbose zephyr_ardupilot_demo.world
````

## Simulation and MAVProxy commands

Various MAVProxy commands can be found in [UAV Configuration - MAVProxy](https://ardupilot.github.io/MAVProxy/html/uav_configuration/index.html).

### Streamrate

In order to enable increased MAVProxy stream rate enter the following in the MavProxy command line.

````bash
set baudrate 921600
set streamrate 50
set streamrate2 50
````

Those commands can be saved to a file called "mavinit.src". They will be automatically executed if you use the following command

````bash
sim_vehicle.py -v ArduCopter -f gazebo-iris -m --aircraft="[absolute path to mavinit.src]" --map --console -I0
````

To increase Mavros topic stream rate enter the following command.

````bash
rosrun mavros mavsys rate --all 50
````

### Takeoff commands

This section concerns UAV simulations.

When messages:

* EKF1 is using GPS
* EKF2 is using GPS

appear then execute the following command in the MAVProxy command line.

```bash
rc 3 1500
mode guided
arm throttle
takeoff 3
```

## GCS

In addition, you can use any GCS that can connect to the Ardupilot locally or remotely(will require connection setup).
If MAVProxy Developer GCS is uncomfortable. Omit --map --console arguments out of SITL launch and use APMPlanner2 or QGroundControl instead.

Local connection with APMPlanner2/QGroundControl is automatic, and easier to use.

You will also need to edit ArduPilot Parameter SYSID_THISMAV to be unique from one another for the GCS connection

### APMPlanner2

Download it from [here](http://firmware.eu.ardupilot.org/Tools/APMPlanner/)
and launch it in terminal or run executable

```bash
apmplanner2
```

### QGroundControl

Download it from [here](https://donlakeflyer.gitbooks.io/qgroundcontrol-user-guide/en/download_and_install.html) and follow the installation guide.

## Multi-Vehicle simulation

This section explains how to connect any combination of multi-vehicles of ArduPilot

For the multi-vehicle connection, port number is increased by 10 per instance(#)
In SITL launch argument -I # of sim_vehicle.py 

-I 0 has FDM in/out ports of 9002/9003 / GCS connection UDP:14550

-I 1 has FDM in/out ports of 9012/9013 / GCS connection UDP:14560

-I 2 has FDM in/out ports of 9022/9023 / GCS connection UDP:14570

and so on...

You will need to edit your world for any combination of Rover, Plane, Copter, etc...

### Example
Look simulation of 3 IRIS quadcopter at once from Jonathan Lopes Florêncio
https://www.youtube.com/watch?v=3c7EhVMaqKY&feature=youtu.be

## Troubleshooting

### Missing libArduPilotPlugin.so... etc

In case you see this message when you launch gazebo with demo worlds, check you have no error after sudo make install.  
If no error use "ls" on the install path given to see if the plugin is really here.  
If this is correct, check with "cat /usr/share/gazebo/setup.sh" the variable GAZEBO_PLUGIN_PATH. It should be the same as the install path. If not use "cp" to copy the lib to right path.

**For Example**

````
sudo cp -a /usr/lib/x86_64-linux-gnu/gazebo-7.0/plugins/ /usr/lib/x86_64-linux-gnu/gazebo-7/
````

Path mismatch is confirmed as ROS's glitch. It only happens with Gazebo 7
