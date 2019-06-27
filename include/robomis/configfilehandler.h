#ifndef CONFIG_FILE_HANDLER_H_
#define CONFIG_FILE_HANDLER_H_

#include <fstream>
#include <yaml-cpp/yaml.h>

// #include <mission_data/robot_arm.h>
#include <mission_data_types/waypoint.h>
#include <mission_data_types/robot_arm.h>
#include <mission_data_types/objects.h>
#include <string>
#include <vector>

class ConfigFileHandler
{
public:
    enum class Key : int {
        ID,
        Type,
        InstanceId,
        Description,
        None,
        LocationDescription,
        LocationType,
        LocationInstanceId,
        RobotId,
        RobotDescription,
        Joint1,
        Joint2,
        Joint3,
        Joint4,
        Joint5,
        Joint6,
        ServiceAreaId,
        Length,
        Width,
        Height,
        Diameter,
        TiltAngle,
        Position,
        Orientation,
        X,
        Y,
        Z,
        W,
        Scan
    };
    ConfigFileHandler() { }
    ~ConfigFileHandler() = default;

    bool loadWaypoint(std::string filename, Waypoint &waypoint);
    bool saveWaypoint(std::string filename, Waypoint &waypoint);

    bool loadWaypoints(std::string filename, std::vector<Waypoint> &waypoints);
    bool saveWaypoints(std::string filename, std::vector<Waypoint> waypoints);

    bool loadRobotArmPoses(std::string filename, std::vector<robot_arm::JointPose> &poses);
    bool saveRobotArmPoses(std::string filename, std::vector<robot_arm::JointPose> poses);

    bool loadObjects(std::string filename, std::vector<mission_data::ObjectIdentifier> &objects);
    bool saveObjects(std::string filename, std::vector<mission_data::ObjectIdentifier> objects);
private:
    void writeStream(const Waypoint &waypoint, std::ostream &out, const std::string &filename, int id = 1);
    void writeStream(const robot_arm::JointPose &pose, std::ostream &out, const std::string &filename, int id = 1);
    void writeStream(const mission_data::ObjectIdentifier &object, std::ostream &out, const std::string &filename, int id = 1);

    void writeNode(const Waypoint &wp, YAML::Node &node, int id);
    void writeNode(const robot_arm::JointPose &pose, YAML::Node &node, int id);
    void writeNode(const mission_data::ObjectIdentifier &object, YAML::Node &node, int id);

    bool setWaypointScaler(Waypoint &waypoint, const YAML::Node  &node, std::string key);
    bool setPositionScaler(Waypoint &waypoint, const YAML::Node  &node, std::string key, ConfigFileHandler::Key & pose_key);
    bool setOrientationScaler(Waypoint &waypoint, const YAML::Node  &node, std::string key, ConfigFileHandler::Key & pose_key);
    bool setJointPoseScaler(robot_arm::JointPose& pose, const YAML::Node  &node, std::string key);
    bool setObjectScaler(mission_data::ObjectIdentifier& object, const YAML::Node  &node, std::string key);

    bool readYamlNode(Waypoint &waypoint, const YAML::Node &node);
    bool readYamlNode(robot_arm::JointPose& pose, YAML::const_iterator &node_it);
    bool readYamlNodes(Waypoint &waypoint, YAML::const_iterator &node_it);
    bool readYamlNode(mission_data::ObjectIdentifier &object, YAML::const_iterator &node_it);

    Key getKeyEnumWaypoint(std::string map_key);
    Key getKeyEnumRobotPose(std::string map_key);
    Key getKeyEnumWaypointType(std::string map_key);
    Key getKeyEnumObjectType(std::string map_key);

    bool error;
    std::string message;
};

#endif /* CONFIG_FILE_HANDLER_H_ */

namespace YAML {
template<>
struct convert<geometry::Pose::Point> {
  static Node encode(const geometry::Pose::Point& position) {
    Node node;
    node["x"] = position.x;
    node["y"] = position.y;
    node["z"] = position.z;
    return node;
  }

  static bool decode(const Node& node, geometry::Pose::Point& position) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    position.x = node[0].as<double>();
    position.y = node[1].as<double>();
    position.z = node[2].as<double>();

    return true;
  }
};

template<>
struct convert<geometry::Pose::Quaternion> {
  static Node encode(const geometry::Pose::Quaternion& orientation) {
    Node node;
    node["x"] = orientation.x;
    node["y"] = orientation.y;
    node["z"] = orientation.z;
    node["w"] = orientation.w;
    return node;
  }

  static bool decode(const Node& node, geometry::Pose::Quaternion& orientation) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    orientation.x = node[0].as<double>();
    orientation.y = node[1].as<double>();
    orientation.z = node[2].as<double>();
    orientation.w = node[3].as<double>();

    return true;
  }
};

template<>
struct convert<geometry::Pose> {
  static Node encode(const geometry::Pose& pose) {
    Node node;
    node["position"]    = pose.position;
    node["orientation"] = pose.orientation;
    return node;
  }

  static bool decode(const Node& node, geometry::Pose& pose) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    pose.position    = node[0].as<geometry::Pose::Point>();
    pose.orientation = node[1].as<geometry::Pose::Quaternion>();

    return true;
  }
};

template<>
struct convert<ServiceArea> {
  static Node encode(const ServiceArea& service_area) {
    Node node;
    node["id"]          = service_area.id;
    node["length"]      = service_area.length;
    node["width"]       = service_area.width;
    node["height"]      = service_area.height;
    node["diameter"]    = service_area.diameter;
    node["tilt_angle"]  = service_area.tilt_angle;
    return node;
  }

  static bool decode(const Node& node, ServiceArea& service_area) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    service_area.id           = node["id"].as<int>();
    service_area.length       = node["length"].as<double>();
    service_area.width        = node["width"].as<double>();
    service_area.height       = node["height"].as<double>();
    service_area.diameter     = node["diameter"].as<double>();
    service_area.tilt_angle   = node["tilt_angle"].as<double>();

    return true;
  }
};

  template<>
struct convert<Location> {
  static Node encode(const Location& location) {
    Node node;
    // node.SetStyle(YAML::EmitterStyle::Flow);
    node["type"]        = location.type;
    node["instance_id"] = location.instance_id;
    node["description"] = location.description;
    return node;
  }

  static bool decode(const Node& node, Location& location) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    location.type        = node["type"].as<int>();
    location.instance_id = node["instance_id"].as<int>();
    location.description = node["description"].as<std::string>();
    return true;
  }
};

template<>
struct convert<Waypoint> {
  static Node encode(const Waypoint& wp) {
    Node node;
    // node.SetStyle(YAML::EmitterStyle::Flow);
    node["id"]           = wp.id;
    node["location"]     = wp.location;
    node["service_area"] = wp.service_area;
    node["pose"]         = wp.pose;
    node["should_scan"]  = wp.should_scan;
    return node;
  }

  static bool decode(const Node& node, Waypoint& wp) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    wp.id           = node["id"].as<int>();
    wp.location     = node["location"].as<Location>();
    wp.service_area = node["service_area"].as<ServiceArea>();
    wp.pose         = node["pose"].as<geometry::Pose>();
    wp.should_scan  = node["should_scan"].as<bool>();

    return true;
  }
};

template<>
struct convert<robot_arm::JointPose> {
  static Node encode(const robot_arm::JointPose& pose) {
    Node node;
    node["id"]          = pose.id;
    node["description"] = pose.description;
    node["joint_1"]     = pose.joint_1;
    node["joint_2"]     = pose.joint_2;
    node["joint_3"]     = pose.joint_3;
    node["joint_4"]     = pose.joint_4;
    node["joint_5"]     = pose.joint_5;
    node["joint_6"]     = pose.joint_6;
    return node;
  }

  static bool decode(const Node& node, robot_arm::JointPose& pose) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    pose.id           = node["id"].as<int>();
    pose.description  = node["description"].as<std::string>();
    pose.joint_1      = node["joint_1"].as<double>();
    pose.joint_2      = node["joint_2"].as<double>();
    pose.joint_3      = node["joint_3"].as<double>();
    pose.joint_4      = node["joint_4"].as<double>();
    pose.joint_5      = node["joint_5"].as<double>();
    pose.joint_5      = node["joint_6"].as<double>();

    return true;
  }
};

template<>
struct convert<mission_data::ObjectIdentifier> {
  static Node encode(const mission_data::ObjectIdentifier& object) {
    Node node;
    node["id"]          = object.id;
    node["instance_id"] = object.instance_id;
    node["type"]        = object.type;
    node["description"] = object.description;

    return node;
  }

  static bool decode(const Node& node, mission_data::ObjectIdentifier& object) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    object.id           = node["id"].as<int>();
    object.type         = node["type"].as<int>();
    object.instance_id  = node["instance_id"].as<int>();
    object.description  = node["description"].as<std::string>();

    return true;
  }
};

} /* namespace YAML */
