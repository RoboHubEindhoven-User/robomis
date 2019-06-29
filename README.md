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
clone this package in to your catkin workspace;

```
cd catkin_ws/src
git clone https://github.com/RoboHubEindhoven/robomis.git
cd ..
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
