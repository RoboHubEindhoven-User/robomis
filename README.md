# Robot Mission Station (robomis)

The robomis application stands for Robot Mission Station. This application provides a clean GUI, which can be use to set robot mission configurations and monitor robot mission status. robomis was developed with C++ and QT.
To know more about this package, you can visit the robomis [wiki](https://github.com/RoboHubEindhoven/robomis/wiki).


## Getting Started
For this package, it is assumed you using Ubuntu 16.04 LTS with ROS Kinetic. This package has not been tested on other OS and ROS versions. 

### Prerequisites
To use this pacakge, the following dependencies have to installed.

```
sudo apt-get update
sudo apt-get install protobuf-compiler libprotobuf-dev libprotoc-dev libyaml-cpp-dev libssl-dev 
```
Clone the mission_planner_comm package if it doesn't exist on your catkin workspace

```
cd catkin_ws/src
git clone https://github.com/RoboHubEindhoven/mission_planner_comm.git
```
Clone the robomis package in to your catkin workspace;

```
git clone https://github.com/RoboHubEindhoven/robomis.git
cd ..
catkin_make
```

if you get an error message like this
```
[ 95%] Built target robomis_generate_messages
usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN"
AUTOGEN: error: process for /home/mechlab/catkin_ws/build/robomis/moc_robomisnode.cpp failed:
usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN"

moc failed...
robomis/CMakeFiles/robomis_node_automoc.dir/build.make:57: recipe for target 'robomis/CMakeFiles/robomis_node_automoc' failed
make[2]: * [robomis/CMakeFiles/robomis_node_automoc] Error 1
CMakeFiles/Makefile2:9598: recipe for target 'robomis/CMakeFiles/robomis_node_automoc.dir/all' failed
make[1]: * [robomis/CMakeFiles/robomis_node_automoc.dir/all] Error 2
Makefile:138: recipe for target 'all' failed
make: * [all] Error 2
Invoking "make -j4 -l4" failed
```
Follow these instructions to resolve the error

Step 1: Open the boost as_binary_operator header file with your favorite editor for example.
```
sudo gedit /usr/include/boost/type_traits/detail/has_binary_operator.hpp 
```
Step 2: Modify 

```
namespace BOOST_JOIN(BOOST_TT_TRAIT_NAME,_impl) {
...
}
```
 To
 
```
#ifndef Q_MOC_RUN 
namespace BOOST_JOIN(BOOST_TT_TRAIT_NAME,_impl) { 
#endif

...

#ifndef Q_MOC_RUN 
} 
#endif
```
Step 3:

```
cd catkin_ws/
catkin_make
```

## Usage
To use this package, makesure the pc running this package is on the same network as the robot.
* Set the ROS_MASTER_IP in the bash file to the IP of the robot
* Execute this package with the command
```
rosrun robomis robomis_node
```

#### Configuration files
For this versions, the configuration files can be save locally only and copied to the robot via ssh/sftp (on terminal or via filezilla).

## Authors

* **Thierry Zinkeng** - *in name of RoboHub Eindhoven* - [RoboHub Eindhoven website](https://robohub-eindhoven.nl/)
