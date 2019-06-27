/*
 * @file /src/RobomisNode.cpp
 *
 * @brief Ros communication central!
 *
 * @date March 2019
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include<cctype>
#include <iomanip>
#include "../include/robomis/robomisnode.hpp"
#include <robomis/systemconfig.h>
#include <mission_protobuf/task_action.pb.h>
#include <robomis/messageaccess.h>
#include <QDebug>
#include <stdio.h>

#define FRAME_ID         "/map"
#define LINK             "/base_link"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ziros {



//auto& MessageKeeper = MessageAccess::Instance();

auto& MessageKeeper = MessageAccess::get();

RobomisNode::RobomisNode(int argc, char** argv) :
    init_argc(argc),
    init_argv(argv)
    {}

RobomisNode::~RobomisNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
        wait();
}

bool RobomisNode::init() {
        ros::init(init_argc,init_argv,"robomis_node");
        if ( ! ros::master::check() ) {
                return false;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;

        waypoint_pub        = n.advertise<robomis::WaypointData>("/robomis/waypoint", 1000);
        feedback_sub        = n.subscribe("/ar_marker", 1000, &RobomisNode::waypointMarkerCallback, this);
        robot_arm_sub       = n.subscribe("/joint_states", 1000, &RobomisNode::jointStateCallback, this);
        waypoint_marker_sub = n.subscribe("/move_base/feedback", 1000, &RobomisNode::feedbackCallback, this);
        listener            = new tf::TransformListener();

        listener->waitForTransform(FRAME_ID, LINK, ros::Time(0), ros::Duration(4));

        return true;
}

bool RobomisNode::init(const std::string &master_url, const std::string &host_url) {
        std::map<std::string,std::string> remappings;
        remappings["__master"] = master_url;
        remappings["__hostname"] = host_url;
        ros::init(remappings,"robomis_node");
        if ( ! ros::master::check() ) {
                return false;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;


        waypoint_pub        = n.advertise<robomis::WaypointData>("/robomis/waypoint", 1000);
        feedback_sub        = n.subscribe("/ar_marker", 1000, &RobomisNode::waypointMarkerCallback, this);
        robot_arm_sub       = n.subscribe("/joint_states", 1000, &RobomisNode::jointStateCallback, this);
        waypoint_marker_sub = n.subscribe("/move_base/feedback", 1000, &RobomisNode::feedbackCallback, this);
        listener            = new tf::TransformListener();

        listener->waitForTransform(FRAME_ID, LINK, ros::Time(0), ros::Duration(4));

        start();
        return true;
}

void RobomisNode::run() {
        ros::Rate loop_rate(10);
        int count = 0;
        while ( ros::ok() ) {

            ros::spinOnce();
            loop_rate.sleep();
            ++count;
        }
        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)

}


void RobomisNode::log( const LogLevel &level, const std::string &msg) {
//        logging_model.insertRows(logging_model.rowCount(),1);
//        std::stringstream logging_model_msg;
//        switch ( level ) {
//                case(Debug) : {
//                                ROS_DEBUG_STREAM(msg);
//                                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
//                                break;
//                }
//                case(Info) : {
//                                ROS_INFO_STREAM(msg);
//                                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
//                                break;
//                }
//                case(Warn) : {
//                                ROS_WARN_STREAM(msg);
//                                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
//                                break;
//                }
//                case(Error) : {
//                                ROS_ERROR_STREAM(msg);
//                                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
//                                break;
//                }
//                case(Fatal) : {
//                                ROS_FATAL_STREAM(msg);
//                                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
//                                break;
//                }
//        }
//        QVariant new_row(QString(logging_model_msg.str().c_str()));
//        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
//        Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void RobomisNode::addWaypoint(int instance_id, std::string name, bool scan, double pos_x, double pos_y, double pos_z, double orient_x, double orient_y, double orient_z, double orient_w) {
    robomis::Waypoint waypoint;

    waypoint.type               = getLocationIdentifierType(name);
    waypoint.instance_id        = instance_id;
    waypoint.description        = createWaypointDescription(name, instance_id);
    waypoint.scan               = scan;
    waypoint.pose.position.x    = pos_x;
    waypoint.pose.position.y    = pos_y;
    waypoint.pose.position.z    = pos_z;
    waypoint.pose.orientation.x = orient_x;
    waypoint.pose.orientation.y = orient_y;
    waypoint.pose.orientation.z = orient_z;
    waypoint.pose.orientation.w = orient_w;
    rob_msg.waypoints.push_back(waypoint);
}

std::string RobomisNode::createWaypointDescription(std::string location, int instance_id) {
    return (location == "Conveyor belt" || location == "Precision Platform" || location == "Entrance" || location == "Exit") ?
                location : location + " " + std::to_string(instance_id);
}

void RobomisNode::updateWaypoint(int index, int instance_id, std::string name, bool scan, double pos_x, double pos_y, double pos_z, double orient_x, double orient_y, double orient_z, double orient_w) {

    int list_size = static_cast<int>(rob_msg.waypoints.size());

    if(index > list_size) return;

    robomis::Waypoint waypoint;

    waypoint.type               = getLocationIdentifierType(name);
    waypoint.instance_id        = instance_id;
    waypoint.description        = name + " " + std::to_string(instance_id);
    waypoint.scan               = scan;
    waypoint.pose.position.x    = pos_x;
    waypoint.pose.position.y    = pos_y;
    waypoint.pose.position.z    = pos_z;
    waypoint.pose.orientation.x = orient_x;
    waypoint.pose.orientation.y = orient_y;
    waypoint.pose.orientation.z = orient_z;
    waypoint.pose.orientation.w = orient_w;

    rob_msg.waypoints[index] = waypoint;
}

int RobomisNode::getLocationIdentifierType(std::string location) {
    if (location == "Entrance")
        return 99;
    if (location == "Exit")
        return 100;
    if (location == "Shelf")
        return 1;
    if (location == "Workstation")
       return 2;
    if (location == "Conveyor belt")
        return 3;
    if (location == "Way Point")
        return 4;
    if (location == "Precision Platform")
        return 5;
    return -1;
}

int RobomisNode::getNextLocationInstanceId(std::string location) {
//    if(getLocationIdentifierType(name);)
    int count = getNrOfLocationWaypoints(location);
    return count + 1;
}

int RobomisNode::getCurrentWaypointMarker() {

}

void RobomisNode::publishWaypoints(void) {
    waypoint_pub.publish(rob_msg);
}

void RobomisNode::reset() {
    rob_msg.frame = "";
    rob_msg.waypoints.clear();
}

void RobomisNode::waypointMarkerCallback(const std_msgs::String::ConstPtr& msg) {
    std::cout << msg->data <<std::endl;
    if(markerFunctPtr != nullptr)
        markerFunctPtr(msg->data);
}

void RobomisNode::feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
    MessageKeeper.lock(&mutex);
    if(MessageKeeper.getShouldGetRobotPosition()){
        MessageKeeper.setRobotPosition(msg->feedback.base_position.pose);
    }
}

void RobomisNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
 joint_state.position.clear();
    sensor_msgs::JointState data;
    robot_arm::JointPose pose;

     pose.joint_1 = msg->position[0];
     pose.joint_2 = msg->position[1];
     pose.joint_3 = msg->position[2];
     pose.joint_4 = msg->position[3];
     pose.joint_5 = msg->position[4];
     pose.joint_6 = msg->position[5];


     auto& message = MessageAccess::get();
     message.lock(&mutex);
     message.setRobotArmPose(pose);
}

mission_ros_msgs::TaskObjective RobomisNode::getTaskObjectFromRef(const mission_ros_msgs::TaskObjective::ConstPtr& msg) {
    mission_ros_msgs::TaskObjective data;
    data.identifier = msg->identifier;

    for (size_t i = 0; i < msg->tasks.size(); i++){
        data.tasks[i] = msg->tasks[i];
    }
    return data;
}

void RobomisNode::todoTasksCallback(const mission_ros_msgs::TaskObjective::ConstPtr& msg) {
    if(todoTaskPtr) {
        todoTaskPtr(getTaskObjectFromRef(msg));
    }
}

void RobomisNode::completedTasksCallback(const mission_ros_msgs::TaskObjective::ConstPtr& msg) {
    if(completedTasksPtr) {
        completedTasksPtr(getTaskObjectFromRef(msg));
    }
}

void RobomisNode::currentTasksCallback(const mission_ros_msgs::TaskObjective::ConstPtr& msg) {
    if(currentTasksPtr) {
        currentTasksPtr(getTaskObjectFromRef(msg));
    }
}

void RobomisNode::objectHolderCallback(const mission_ros_msgs::TaskObjective::ConstPtr& msg) {
    if(objectHolderPtr) {
        objectHolderPtr(getTaskObjectFromRef(msg));
    }
}


int RobomisNode::getNrOfLocationWaypoints(std::string location) {

    int count = 0;
    for(size_t i = 0; i < rob_msg.waypoints.size(); i++){
        if(rob_msg.waypoints[i].type == getLocationIdentifierType(location)){
            count++;
        }
    }
    return count;
}
// get the current transform
tf::StampedTransform RobomisNode::getTransform()
{
//  tf::StampedTransform transform;
//  try
//  {
//    listener->lookupTransform(FRAME_ID, LINK, ros::Time(0), transform);
//  }
//  catch(tf::TransformException &ex)
//  {
//    ROS_ERROR("TransformException Thrown : %s", ex.what());
//  }
//  return transform;
}

// Get the robot's current position from the transform
geometry_msgs::Pose RobomisNode::getRobotPosition() {
  geometry_msgs::Pose pose;
//  tf::StampedTransform transform; = getTransform();

  try
  {
    listener->lookupTransform(FRAME_ID, LINK, ros::Time(0), transform);
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("TransformException Thrown : %s", ex.what());
  }

  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();

  tf::quaternionTFToMsg(transform.getRotation(), pose.orientation);
#if ROBOMIS_DEBUG
  ROS_INFO("Pose X=%.3f, Pose Y: %.3f, Pose Z: %.3f, Orient W: %f, Orient Z: %f",  pose.position.x,  pose.position.y,  pose.position.z, pose.orientation.w, pose.orientation.z);
#endif

  return pose;
}

sensor_msgs::JointState RobomisNode::getRobotArmPose() {
    return joint_state;
}



} /* namespace ziros */
