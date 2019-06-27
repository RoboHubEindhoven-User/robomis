#include "../include/robomis/configuration.h"

#include "../include/robomis/vector_utils.h"

#include <QtNetwork/QFtp>

Configuration::Configuration(){

}

Configuration::~Configuration(){
}

void Configuration::addRobotArmPose(robot_arm::JointPose pose){
    pose.id = static_cast<int>(robot_arm_poses.size() + 1);
    robot_arm_poses.push_back(pose);
}

void Configuration::addWaypoint(Waypoint waypoint){
    waypoint.id = static_cast<int>(waypoints.size() + 1);
    waypoints.push_back(waypoint);
}

void Configuration::addObject(mission_data::ObjectIdentifier object){
    objects.push_back(object);
}


std::vector<robot_arm::JointPose> Configuration::getRobotArmPoses(void) const {
    return robot_arm_poses;
}

robot_arm::JointPose* Configuration::getRobotArmPose(int id) {
    for (size_t i = 0; i < robot_arm_poses.size(); i++) {
        if(robot_arm_poses[i].id == id)
            return &robot_arm_poses[i];
    }
    return nullptr;
}

std::vector<Waypoint> Configuration::getWaypoints(void) const {
    return waypoints;
}

Waypoint* Configuration::getWaypoint(int id) {
    for (size_t i = 0; i < waypoints.size(); i++) {
        if(waypoints[i].id == id)
            return &waypoints[i];
    }
    return nullptr;
}

std::vector<mission_data::ObjectIdentifier> Configuration::getObjects(void) const {
    return objects;
}


void Configuration::updateRobotArmPose(int id, robot_arm::JointPose pose) {

    for (size_t i = 0; i < robot_arm_poses.size(); i++) {
        if(robot_arm_poses[i].id == id)
            robot_arm_poses[i] = pose;
    }
}

void Configuration::updateWaypoint(int id, Waypoint waypoint) {

    for (size_t i = 0; i < waypoints.size(); i++) {
        if(waypoints[i].id == id)
            waypoints[i] = waypoint;
    }
}

void Configuration::updateObject(int index, mission_data::ObjectIdentifier object) {

    int list_size = static_cast<int>(objects.size());

    if(index > list_size) return;

    objects[static_cast<size_t>(index)] = object;
}

bool Configuration::deleteRobotArmPose(int id) {
    for(size_t i = 0; i < robot_arm_poses.size(); i++){
        if(robot_arm_poses[i].id == id){
            remove(robot_arm_poses, static_cast<size_t>(i));
            return true;
        }
    }
    return false;
}

bool Configuration::deleteWaypoint(int id) {
    for(size_t i = 0; i < waypoints.size(); i++){
        if(waypoints[i].id == id){
            remove(waypoints, static_cast<size_t>(i));
            return true;
        }
    }
    return false;
}

bool Configuration::deleteObject(int id) {
    for(size_t i = 0; i < objects.size(); i++){
        if(objects[i].id == id){
            remove(objects, static_cast<size_t>(i));
            return true;
        }
    }
    return false;
}

int Configuration::getNrOfWaypointsWithType(mission_protobuf::LocationIdentifier::LocationType type) {

    int count = 0;
    if (waypoints.size() > 0) {
        for(size_t i = 0; i < waypoints.size(); i++){
            if(waypoints[i].location.type == WaypointHelper::getLocationTypeNumeral(type)){
                count++;
            }
        }
    }
    return count;
}

void Configuration::resetWaypoints(void) {
    waypoints.clear();
}

void Configuration::resetRobotArmPoses(void) {
    robot_arm_poses.clear();
}

void Configuration::resetObjects(void) {
    objects.clear();
}

bool Configuration::save(std::string filename, Type type) {
    switch(type) {
    case Type::Waypoint:
        return file_handler.saveWaypoints(filename, waypoints);
    case Type::RobotArmPose:
        return file_handler.saveRobotArmPoses(filename, robot_arm_poses);
    case Type::Object:
        return file_handler.saveObjects(filename, objects);
    }
    return false;
}

bool Configuration::load(std::string filename, Type type) {
    switch(type) {
    case Type::Waypoint:
        return file_handler.loadWaypoints(filename, waypoints);
    case Type::RobotArmPose:
        return file_handler.loadRobotArmPoses(filename, robot_arm_poses);
    case Type::Object:
        return file_handler.loadObjects(filename, objects);
    }
    return false;
}
