/*
 * @file /src/robomisnode.cpp
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
#include <mission_protobuf/task_action.pb.h>

#define FRAME_ID         "/map"
#define LINK             "/base_link"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ziros {


robomisNode::robomisNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv),
    shelf_nr(0),
    conveyor_nr(0),
    workstation_nr(0),
    waypoint_nr(0),
    pre_plaform_nr(0)
    {}

robomisNode::~robomisNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
        wait();
}

bool robomisNode::init() {
        ros::init(init_argc,init_argv,"robomis_node");
        if ( ! ros::master::check() ) {
                return false;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;
        // Add your ros communications here.
        waypoint_pub = n.advertise<robomis::WaypointData>("/robomis/waypoint", 1000);
//        feedback_sub = n.subscribe("chatter", 1000, feedbackCallback);
        feedback_sub = n.subscribe("/ar_marker", 1000, &robomisNode::waypointMarkerCallback, this);
        waypoint_marker_sub  = n.subscribe("/move_base/feedback", 1000, &robomisNode::feedbackCallback, this);
//         waypoint_marker_sub  = n.subscribe("/odom", 1000, &robomisNode::feedbackCallback, this);
        listener = new tf::TransformListener();
        listener->waitForTransform(FRAME_ID, LINK, ros::Time(0), ros::Duration(4));

        return true;
}

bool robomisNode::init(const std::string &master_url, const std::string &host_url) {
        std::map<std::string,std::string> remappings;
        remappings["__master"] = master_url;
        remappings["__hostname"] = host_url;
        ros::init(remappings,"robomis_node");
        if ( ! ros::master::check() ) {
                return false;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;
        // Add your ros communications here.
        waypoint_pub = n.advertise<robomis::WaypointData>("/robomis/waypoint", 1000);
        feedback_sub = n.subscribe("/ar_marker", 1000, &robomisNode::waypointMarkerCallback, this);
//        waypoint_marker_sub = n.subscribe("/odom", 1000, &robomisNode::feedbackCallback, this);
        waypoint_marker_sub  = n.subscribe("/move_base/feedback", 1000, &robomisNode::feedbackCallback, this);
        listener = new tf::TransformListener();
        listener->waitForTransform(FRAME_ID, LINK, ros::Time(0), ros::Duration(4));

        start();
        return true;
}

void robomisNode::run() {
        ros::Rate loop_rate(1);
        int count = 0;
        while ( ros::ok() ) {


//                std_msgs::String msg;
//                std::stringstream ss;
//                ss << "hello world " << count;
//                msg.data = ss.str();
//                waypoint_pub.publish(msg);
//                log(Info,std::string("I sent: ")+msg.data);
                ros::spinOnce();
                loop_rate.sleep();
                ++count;
        }
        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void robomisNode::log( const LogLevel &level, const std::string &msg) {
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

void robomisNode::addWaypoint(int instance_id, std::string name, bool scan, double pos_x, double pos_y, double pos_z, double orient_x, double orient_y, double orient_z, double orient_w) {
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

std::string robomisNode::createWaypointDescription(std::string location, int instance_id) {
    return (location == "Conveyor belt" || location == "Precision Platform" || location == "Entrance" || location == "Exit") ?
                location : location + " " + std::to_string(instance_id);
}

void robomisNode::updateWaypoint(int index, int instance_id, std::string name, bool scan, double pos_x, double pos_y, double pos_z, double orient_x, double orient_y, double orient_z, double orient_w) {

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

int robomisNode::getLocationIdentifierType(std::string location) {
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

int robomisNode::getNextLocationInstanceId(std::string location) {
//    if(getLocationIdentifierType(name);)
    int count = getNrOfLocationWaypoints(location);
    return count + 1;
}

int robomisNode::getCurrentWaypointMarker() {

}

void robomisNode::publishWaypoints(void) {
    waypoint_pub.publish(rob_msg);
}

void robomisNode::reset() {
    rob_msg.frame = "";
    rob_msg.waypoints.clear();
}

void robomisNode::waypointMarkerCallback(const std_msgs::String::ConstPtr& msg) {
//    std::cout << msg->data <<std::endl;
    if(markerFunctPtr != nullptr)
        markerFunctPtr(msg->data);
}

//void robomisNode::feedbackCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//    way_msg.pose = msg->pose.pose;
//    ROS_INFO("Pos X=%.3f, Pos Y: %.3f, Pos Z: %.3f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
//    if(feedbackFunctPtr != nullptr)
//        feedbackFunctPtr(way_msg);
//}
void robomisNode::feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
    way_msg.pose = msg->feedback.base_position.pose;
    ROS_INFO("Pos X=%.3f, Pos Y: %.3f, Pos Z: %.3f",  way_msg.pose.position.x,  way_msg.pose.position.y,  way_msg.pose.position.z);
    if(feedbackFunctPtr != nullptr)
        feedbackFunctPtr(way_msg);
}


int robomisNode::getNrOfLocationWaypoints(std::string location) {

    int count = 0;
    for(size_t i = 0; i < rob_msg.waypoints.size(); i++){
        if(rob_msg.waypoints[i].type == getLocationIdentifierType(location)){
            count++;
        }
    }
    return count;
}
// get the current transform
tf::StampedTransform robomisNode::getTransform()
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
geometry_msgs::Pose robomisNode::getRobotPosition() {
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

  ROS_INFO("Pose X=%.3f, Pose Y: %.3f, Pose Z: %.3f, Orient W: %f, Orient Z: %f",  pose.position.x,  pose.position.y,  pose.position.z, pose.orientation.w, pose.orientation.z);


  return pose;
}

void robomisNode::newPosereceived() {
//    Q_EMIT getRobotPosition();
//   Q_EMIT newPoseSignal(getRobotPosition());
}

} /* namespace ziros */
