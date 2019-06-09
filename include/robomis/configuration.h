#ifndef ROBOTCONFIGURATION_H
#define ROBOTCONFIGURATION_H

#include "robotarmpose.h"
#include "waypoint_helper.h"
#include "object.hpp"
#include "taskbase.h"
#include "transportationtask.h"
#include <vector>
#include <mission_protobuf/task_action.pb.h>
#include "config_file_handler.h"


class Configuration
{
public:
    Configuration();
    ~Configuration();

    void addRobotArmPose(robot_arm::JointPose pose);
    void addWaypoint(Waypoint waypoint);
    void addObject(mission_protobuf::ObjectIdentifier object);


    std::vector<robot_arm::JointPose> getRobotArmPoses(void) const;
    robot_arm::JointPose getRobotArmPoseAtIndex(int index) { return robot_arm_poses[static_cast<size_t>(index)]; }
    std::vector<Waypoint> getWaypoints(void) const;
    Waypoint getWaypointAtIndex(int index) { return waypoints[static_cast<size_t>(index)]; }
    std::vector<mission_protobuf::ObjectIdentifier> getObjects(void) const;

    void updateRobotArmPose(int index, robot_arm::JointPose pose);
    void updateWaypoint(int index, Waypoint waypoint);
    void updateObject(int index, mission_protobuf::ObjectIdentifier object);

    void deleteRobotArmPose(int index);
    void deleteWaypoint(int index);
    void deleteObject(int index);

    void resetWaypoints(void);
    void resetRobotArmPoses(void);
    void resetObjects(void);

    bool saveWaypoints(std::string filename);
    bool saveRobotArmPoses(std::string filename);
    bool loadWaypoints(std::string filename);
    bool loadRobotArmPoses(std::string filename);

    int getNrOfWaypointsWithType(mission_protobuf::LocationIdentifier::LocationType type = mission_protobuf::LocationIdentifier::NONE);


private:

    std::vector<robot_arm::JointPose> robot_arm_poses;
    std::vector<Waypoint> waypoints;
    std::vector<mission_protobuf::ObjectIdentifier> objects;
    ConfigFileHandler file_handler;

};

#endif // CONFIGURATION_H
