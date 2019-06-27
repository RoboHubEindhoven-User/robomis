#ifndef ROBOTCONFIGURATION_H
#define ROBOTCONFIGURATION_H

#include "robotarmpose.h"
#include "waypoint_helper.h"
#include "object.hpp"
#include "taskbase.h"
#include "transportationtask.h"
#include <vector>
#include <mission_protobuf/task_action.pb.h>
#include "configfilehandler.h"


class Configuration
{
public:
    enum class Type : int {
        Waypoint     = 1,
        RobotArmPose = 2,
        Object       = 3
    };
    Configuration();
    ~Configuration();

    void addRobotArmPose(robot_arm::JointPose pose);
    void addWaypoint(Waypoint waypoint);
    void addObject(mission_data::ObjectIdentifier object);


    std::vector<robot_arm::JointPose> getRobotArmPoses(void) const;
    robot_arm::JointPose getRobotArmPoseAtIndex(int index) { return robot_arm_poses[static_cast<size_t>(index)]; }
    robot_arm::JointPose* getRobotArmPose(int id);
    std::vector<Waypoint> getWaypoints(void) const;
    Waypoint getWaypointAtIndex(int index) { return waypoints[static_cast<size_t>(index)]; }
    Waypoint* getWaypoint(int id);
    std::vector<mission_data::ObjectIdentifier> getObjects(void) const;

    void updateRobotArmPose(int id, robot_arm::JointPose pose);
    void updateWaypoint(int id, Waypoint waypoint);
    void updateObject(int id, mission_data::ObjectIdentifier object);

    bool deleteRobotArmPose(int id);
    bool deleteWaypoint(int id);
    bool deleteObject(int id);

    void resetWaypoints(void);
    void resetRobotArmPoses(void);
    void resetObjects(void);

    bool load(std::string filename, Type type);
    bool save(std::string filename, Type type);

    int getNrOfWaypointsWithType(mission_protobuf::LocationIdentifier::LocationType type = mission_protobuf::LocationIdentifier::NONE);


private:

    std::vector<robot_arm::JointPose> robot_arm_poses;
    std::vector<Waypoint> waypoints;
    std::vector<mission_data::ObjectIdentifier> objects;
    ConfigFileHandler file_handler;

};

#endif // CONFIGURATION_H
