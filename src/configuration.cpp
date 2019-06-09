#include "../include/robomis/configuration.h"

#include "../include/robomis/vector_utils.h"

Configuration::Configuration(){

}

Configuration::~Configuration(){
}

void Configuration::addRobotArmPose(robot_arm::JointPose pose){
    robot_arm_poses.push_back(pose);
}

void Configuration::addWaypoint(Waypoint waypoint){
    waypoints.push_back(waypoint);
}

void Configuration::addObject(mission_protobuf::ObjectIdentifier object){
    objects.push_back(object);
}


std::vector<robot_arm::JointPose> Configuration::getRobotArmPoses(void) const {
    return robot_arm_poses;
}

std::vector<Waypoint> Configuration::getWaypoints(void) const {
    return waypoints;
}

std::vector<mission_protobuf::ObjectIdentifier> Configuration::getObjects(void) const {
    return objects;
}


void Configuration::updateRobotArmPose(int index, robot_arm::JointPose pose) {

    int list_size = static_cast<int>(robot_arm_poses.size());

    if(index > list_size) return;

    robot_arm_poses[static_cast<size_t>(index)] = pose;
//    robot_arm_config.mutable_arm_pose()->
}

void Configuration::updateWaypoint(int index, Waypoint waypoint) {

    int list_size = static_cast<int>(waypoints.size());

    if(index > list_size) return;

    waypoints[static_cast<size_t>(index)] = waypoint;
}

void Configuration::updateObject(int index, mission_protobuf::ObjectIdentifier object) {

    int list_size = static_cast<int>(objects.size());

    if(index > list_size) return;

    objects[static_cast<size_t>(index)] = object;
}

void Configuration::deleteRobotArmPose(int index) {
    remove(robot_arm_poses, static_cast<size_t>(index));
}

void Configuration::deleteWaypoint(int index) {
    remove(waypoints, static_cast<size_t>(index));
}

void Configuration::deleteObject(int index) {
    remove(objects, static_cast<size_t>(index));
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

bool Configuration::saveWaypoints(std::string filename) {
    return file_handler.saveWaypoints(filename, waypoints);
}

bool Configuration::loadWaypoints(std::string filename) {
   return file_handler.loadWaypoints(filename, waypoints);
}

bool Configuration::saveRobotArmPoses(std::string filename) {
    return file_handler.saveRobotArmPoses(filename, robot_arm_poses);
}

bool Configuration::loadRobotArmPoses(std::string filename) {
   return file_handler.loadRobotArmPoses(filename, robot_arm_poses);
}

