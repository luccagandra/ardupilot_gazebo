# Installation instructions

Installation instructions are based on [this](https://docs.google.com/document/d/1CIObvL-HHYCosqRLb9j_OIzAPuPPpV2wvajG1nY-UlY/edit?usp=sharing) document. Special thanks to [@AnaBatinovic](https://github.com/AnaBatinovic).

## Various packages

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

## GeographicLib

```bash
sudo apt install geographiclib-doc geographiclib-tools libgeographic17 node-geographiclib
cd /opt/ros/melodic/lib/mavros/
sudo ./install_geographiclib_datasets.sh
```

Note: If building Mavros from source locate the *install_geographiclib_datasets.sh* script inside your local Mavros source folder.

## Ardupilot

```bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
cd Tools/environment_install
./install-prereqs-ubuntu.sh -y
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc
pip install mavproxy
source ~/.bashrc
```

For additional information see: [Setting up the Build Environment(Linux/Ubuntu).](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)

## Catkin workspace setup

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
echo "source ~/catkin_ws/src/ardupilot_gazebo/scripts/shell_scripts.sh" >> ~/.bashrc
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
