#include "../include/robomis/configuration.h"
#include <iostream>
#include <stddef.h>


bool ConfigFileHandler::loadWaypoint(std::string filename, Waypoint &waypoint)
{
    try
    {
        YAML::Node node = YAML::LoadFile(filename);
        error = false;
        message = "";
        readYamlNode(waypoint, node);
        return true;
    }
    catch (YAML::ParserException &ex)
    {
        message = ex.what();
    }

    return false;
}

bool ConfigFileHandler::saveWaypoint(std::string filename, Waypoint &waypoint)
{
    try
    {
        std::ofstream out(filename);
        if (out)
        {
            writeStream(waypoint, out, filename);
            out.close();
            return true;
        }
        else
        {
            error = true;
            message = "Failed to open " + filename + " for writing.";
        }
    }
    catch (std::exception ex)
    {
        error = true;
        message = ex.what();
    }
    return false;
}

bool ConfigFileHandler::loadWaypoints(std::string filename, std::vector<Waypoint> &waypoints)
{
    try
    {
        YAML::Node node = YAML::LoadFile(filename);
        error = false;
        message = "";
        
        // std::cout << "Size: "<<node.size()<<std::endl;
        
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            // std::cout<<"key ---\n"<<YAML::Dump(key)<<"\nvalue---\n"<<YAML::Dump(value)<<"\n***************************"<<std::endl;
            Waypoint wp;
            readYamlNodes(wp, it);
            // std::cout <<"Description: " <<wp.location.description << "\n";
            waypoints.push_back(wp);
        }
        return true;
    }
    catch (YAML::ParserException &ex)
    {
        message = ex.what();
    }

    return false;
}
bool ConfigFileHandler::saveWaypoints(std::string filename, std::vector<Waypoint> waypoints)
{
    try
    {
        std::ofstream out(filename);
        if (out)
        {
            for (size_t i = 0; i < waypoints.size(); i++) {
                writeStream(waypoints[i], out, filename, static_cast<int>(i + 1));
            }
            out.close();
            return true;
        }
        else
        {
            error = true;
            message = "Failed to open " + filename + " for writing.";
        }
    }
    catch (std::exception ex)
    {
        error = true;
        message = ex.what();
    }
    return false;
}
bool ConfigFileHandler::loadRobotArmPoses(std::string filename, std::vector<robot_arm::JointPose> &poses)
{
      try
    {
        YAML::Node node = YAML::LoadFile(filename);
        error = false;
        message = "";
        
        // std::cout << "Size: "<<node.size()<<std::endl;
        
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            // std::cout<<"key ---\n"<<YAML::Dump(key)<<"\nvalue---\n"<<YAML::Dump(value)<<"\n***************************"<<std::endl;
            robot_arm::JointPose pos;
            readYamlNode(pos, it);
            // std::cout <<"Description: " <<pos.description << "\n";
            poses.push_back(pos);
        }
        return true;
    }
    catch (YAML::ParserException &ex)
    {
        message = ex.what();
    }

    return false;
   
}

bool ConfigFileHandler::saveRobotArmPoses(std::string filename, std::vector<robot_arm::JointPose> poses)
{ 
   try
    {
        std::ofstream out(filename);
        if (out)
        {
            for (size_t i = 0; i < poses.size(); i++) {
                writeStream(poses[i], out, filename, static_cast<int>(i + 1));
            }
            out.close();
            return true;
        }
        else
        {
            error = true;
            message = "Failed to open " + filename + " for writing.";
        }
    }
    catch (std::exception ex)
    {
        error = true;
        message = ex.what();
    }
    return false;
}


bool ConfigFileHandler::loadObjects(std::string filename, std::vector<mission_data::ObjectIdentifier> &objects) {
    try
  {
      YAML::Node node = YAML::LoadFile(filename);
      error = false;
      message = "";

      for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
          mission_data::ObjectIdentifier obj;
          readYamlNode(obj, it);
          objects.push_back(obj);
      }
      return true;
  }
  catch (YAML::ParserException &ex)
  {
      message = ex.what();
  }

  return false;
}

bool ConfigFileHandler::saveObjects(std::string filename, std::vector<mission_data::ObjectIdentifier> objects) {
    try
     {
         std::ofstream out(filename);
         if (out)
         {
             for (size_t i = 0; i < objects.size(); i++) {
                 writeStream(objects[i], out, filename, static_cast<int>(i + 1));
             }
             out.close();
             return true;
         }
         else
         {
             error = true;
             message = "Failed to open " + filename + " for writing.";
         }
     }
     catch (std::exception ex)
     {
         error = true;
         message = ex.what();
     }
     return false;
}

void ConfigFileHandler::writeStream(const Waypoint &waypoint, std::ostream &out, const std::string &filename, int id)
{
    error = false;
    message = "";
    YAML::Node node;
    writeNode(waypoint, node, id);
    if (!error)
    {
        out << node<<'\n';
    }
}

void ConfigFileHandler::writeStream(const robot_arm::JointPose &pose, std::ostream &out, const std::string &filename, int id)
{
    error = false;
    message = "";
    YAML::Node node;
    writeNode(pose, node, id);
    if (!error)
    {
        out << node<<'\n';
    }
}

void ConfigFileHandler::writeStream(const mission_data::ObjectIdentifier &object, std::ostream &out, const std::string &filename, int id)
{
    error = false;
    message = "";
    YAML::Node node;
    writeNode(object, node, id);
    if (!error)
    {
        out << node<<'\n';
    }
}


void ConfigFileHandler::writeNode(const Waypoint &wp, YAML::Node &node, int id)
{
    std::string id_str = "waypoint/waypoint" + std::to_string(id);
    node[id_str] = wp;
}

void ConfigFileHandler::writeNode(const robot_arm::JointPose &pose, YAML::Node &node, int id)
{
    std::string id_str = "jointpose/jointpose" + std::to_string(id);
    node[id_str] = pose;
}

void ConfigFileHandler::writeNode(const mission_data::ObjectIdentifier &object, YAML::Node &node, int id)
{
    std::string id_str = "object/object" + std::to_string(id);
    node[id_str] = object;
}

ConfigFileHandler::Key ConfigFileHandler::getKeyEnumWaypoint(std::string map_key)
{
    if (map_key == "id")
        return ConfigFileHandler::Key::ID;
    if (map_key == "x")
        return ConfigFileHandler::Key::X;
    if (map_key == "y")
        return ConfigFileHandler::Key::Y;
    if (map_key == "z")
        return ConfigFileHandler::Key::Z;
    if (map_key == "w")
        return ConfigFileHandler::Key::W;
    if (map_key == "description") return ConfigFileHandler::Key::LocationDescription;
    if (map_key == "type") return ConfigFileHandler::Key::LocationType;
    if (map_key == "instance_id") return ConfigFileHandler::Key::LocationInstanceId;
    if (map_key == "length") return ConfigFileHandler::Key::Length;
    if (map_key == "width") return ConfigFileHandler::Key::Width;
    if (map_key == "height") return ConfigFileHandler::Key::Height;
    if (map_key == "diameter") return ConfigFileHandler::Key::Diameter;
    if (map_key == "tilt_angle") return ConfigFileHandler::Key::TiltAngle;
    if (map_key == "should_scan") return ConfigFileHandler::Key::Scan;

    return ConfigFileHandler::Key::None;
}

ConfigFileHandler::Key ConfigFileHandler::getKeyEnumRobotPose(std::string map_key)
{
    if (map_key == "id") return ConfigFileHandler::Key::RobotId;
    if (map_key == "description") return ConfigFileHandler::Key::RobotDescription;
    if (map_key == "joint_1") return ConfigFileHandler::Key::Joint1;
    if (map_key == "joint_2") return ConfigFileHandler::Key::Joint2;
    if (map_key == "joint_3") return ConfigFileHandler::Key::Joint3;
    if (map_key == "joint_4") return ConfigFileHandler::Key::Joint4;
    if (map_key == "joint_5") return ConfigFileHandler::Key::Joint5;
    if (map_key == "joint_6") return ConfigFileHandler::Key::Joint6;

    return ConfigFileHandler::Key::None;
}

ConfigFileHandler::Key ConfigFileHandler::getKeyEnumWaypointType(std::string map_key)
{
    if (map_key == "position") return ConfigFileHandler::Key::Position;
    if (map_key == "orientation") return ConfigFileHandler::Key::Orientation;

    return ConfigFileHandler::Key::None;
}

ConfigFileHandler::Key ConfigFileHandler::getKeyEnumObjectType(std::string map_key)
{
    if (map_key == "id") return ConfigFileHandler::Key::ID;
    if (map_key == "instance_id") return ConfigFileHandler::Key::InstanceId;
    if (map_key == "type") return ConfigFileHandler::Key::Type;
    if (map_key == "description") return ConfigFileHandler::Key::Description;

    return ConfigFileHandler::Key::None;
}

bool ConfigFileHandler::setWaypointScaler(Waypoint &waypoint, const YAML::Node &node, std::string key)
{

    switch (getKeyEnumWaypoint(key))
    {
    case ConfigFileHandler::Key::ID:
//        std::cout<<"Waypoint id: "<<node.as<int>()<<std::endl;
//        waypoint.id = node.as<int>();
        break;
    case ConfigFileHandler::Key::LocationDescription:
        waypoint.location.description = node.as<std::string>();
        break;
    case ConfigFileHandler::Key::LocationType:
        waypoint.location.type = node.as<int>();
        break;
    case ConfigFileHandler::Key::LocationInstanceId:
        waypoint.location.instance_id = node.as<int>();
        break;
    case ConfigFileHandler::Key::Length:
        waypoint.service_area.length = node.as<double>();
        break;
    case ConfigFileHandler::Key::Width:
        waypoint.service_area.width = node.as<double>();
        break;
    case ConfigFileHandler::Key::Height:
        waypoint.service_area.height = node.as<double>();
        break;
    case ConfigFileHandler::Key::Diameter:
        waypoint.service_area.diameter = node.as<double>();
        break;
    case ConfigFileHandler::Key::TiltAngle:
        waypoint.service_area.tilt_angle = node.as<double>();
        break;
    case ConfigFileHandler::Key::Scan:
        waypoint.should_scan = node.as<bool>();
    default:
        return false;
    }

    return true;
}

bool ConfigFileHandler::setPositionScaler(Waypoint &waypoint, const YAML::Node  &node, std::string key, ConfigFileHandler::Key & pose_key){

    switch (getKeyEnumWaypoint(key))
    {
    case ConfigFileHandler::Key::X:
        waypoint.pose.position.x = node.as<double>();
        break;
    case ConfigFileHandler::Key::Y:
        waypoint.pose.position.y = node.as<double>();
        break;
    case ConfigFileHandler::Key::Z:
        waypoint.pose.position.z = node.as<double>();
        pose_key = ConfigFileHandler::Key::None;
    default:
        return false;
    }
    return true;
}

bool ConfigFileHandler::setOrientationScaler(Waypoint &waypoint, const YAML::Node  &node, std::string key, ConfigFileHandler::Key & pose_key){

    switch (getKeyEnumWaypoint(key))
    {
    case ConfigFileHandler::Key::X:
        waypoint.pose.orientation.x = node.as<double>();
        break;
    case ConfigFileHandler::Key::Y:
        waypoint.pose.orientation.y = node.as<double>();
        break;
    case ConfigFileHandler::Key::Z:
        waypoint.pose.orientation.z = node.as<double>();
        break;
    case ConfigFileHandler::Key::W:
        waypoint.pose.orientation.w = node.as<double>();
        pose_key = ConfigFileHandler::Key::None;
        break;
    default:
        return false;
    }
    return true;
}

bool ConfigFileHandler::setJointPoseScaler(robot_arm::JointPose& pose, const YAML::Node  &node, std::string key) {

    switch (getKeyEnumRobotPose(key))
    {
    case ConfigFileHandler::Key::RobotId:
        pose.id = node.as<int>();
        break;
    case ConfigFileHandler::Key::RobotDescription:
        pose.description = node.as<std::string>();
        break;
    case ConfigFileHandler::Key::Joint1:
        pose.joint_1 = node.as<double>();
        break;
    case ConfigFileHandler::Key::Joint2:
        pose.joint_2 = node.as<double>();
        break;
    case ConfigFileHandler::Key::Joint3:
        pose.joint_3 = node.as<double>();
        break;
    case ConfigFileHandler::Key::Joint4:
        pose.joint_4 = node.as<double>();
        break;
    case ConfigFileHandler::Key::Joint5:
        pose.joint_5 = node.as<double>();
        break;
    case ConfigFileHandler::Key::Joint6:
        pose.joint_6 = node.as<double>();
        break;
    default:
        return false;
    }
    return true;
}

bool ConfigFileHandler::setObjectScaler(mission_data::ObjectIdentifier& object, const YAML::Node  &node, std::string key) {

    switch (getKeyEnumObjectType(key))
    {
    case ConfigFileHandler::Key::ID:
        object.id = node.as<int>();
        break;
    case ConfigFileHandler::Key::InstanceId:
        object.instance_id = node.as<int>();
        break;
    case ConfigFileHandler::Key::Type:
        object.type = node.as<int>();
        break;
    case ConfigFileHandler::Key::Description:
        object.description = node.as<std::string>();
        break;
    default:
        return false;
    }
    return true;
}

bool ConfigFileHandler::readYamlNode(Waypoint &waypoint, const YAML::Node &node)
{
    YAML::const_iterator tmp_it;
    static std::string map_key = "";
    static ConfigFileHandler::Key pose_key = ConfigFileHandler::Key::None;
    
    switch (node.Type())
    {
    case YAML::NodeType::Map:
    {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
        {
            ConfigFileHandler::Key key = getKeyEnumWaypointType(it->first.as<std::string>());

            if ((pose_key == ConfigFileHandler::Key::None) && (key != ConfigFileHandler::Key::None)){
                pose_key = key;
                // std::cout << "Key in map: " << it->first.as<std::string>()<< std::endl;
            }
            
            map_key = it->first.as<std::string>();
            readYamlNode(waypoint, it->second);
        }
        break;
    }
    case YAML::NodeType::Scalar:
    {
        // std::cout << "Key in scaler: " << map_key << std::endl;
        if (pose_key == ConfigFileHandler::Key::Position) {
            // std::cout << "Position............."<< std::endl;
            setPositionScaler(waypoint, node, map_key, pose_key);
        }
        else if (pose_key == ConfigFileHandler::Key::Orientation){
            // std::cout << "Orientation............."<< std::endl;
            setOrientationScaler(waypoint, node, map_key, pose_key);
        }
        else{        
            setWaypointScaler(waypoint, node, map_key);
        }
        
        break;
    }
    case YAML::NodeType::Sequence:
    {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
        {
            // std::cout << "Sequence: " << it->first.as<std::string>() << ", Value: " << it->second.as<std::string>() << std::endl;
            readYamlNode(waypoint, *it);
        }
        break;
    }
    default:
        break;
    }
    return true;
}


bool ConfigFileHandler::readYamlNodes(Waypoint &waypoint, YAML::const_iterator &node_it)
{
    const YAML::Node &key = node_it->first;
    const YAML::Node &node = node_it->second;
    YAML::const_iterator tmp_it;
    static std::string map_key = "";
    static ConfigFileHandler::Key pose_key = ConfigFileHandler::Key::None;
    
    switch (node.Type())
    {
    case YAML::NodeType::Map:
    {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
        {
            ConfigFileHandler::Key key = getKeyEnumWaypointType(it->first.as<std::string>());
            if(it->first.as<std::string>() == "id"){
                waypoint.id = it->second.as<int>();
            }

            if ((pose_key == ConfigFileHandler::Key::None) && (key != ConfigFileHandler::Key::None)){
                pose_key = key;
                // std::cout << "Key in map: " << it->first.as<std::string>()<< std::endl;
            }
            
            map_key = it->first.as<std::string>();
            readYamlNode(waypoint, it->second);
        }
        break;
    }
    case YAML::NodeType::Scalar:
    {
        // std::cout << "Key in scaler: " << map_key << std::endl;
        if (pose_key == ConfigFileHandler::Key::Position) {
            // std::cout << "Position............."<< std::endl;
            setPositionScaler(waypoint, node, map_key, pose_key);
        }
        else if (pose_key == ConfigFileHandler::Key::Orientation){
            // std::cout << "Orientation............."<< std::endl;
            setOrientationScaler(waypoint, node, map_key, pose_key);
        }
        else{        
            setWaypointScaler(waypoint, node, map_key);
        }
        
        break;
    }
    case YAML::NodeType::Sequence:
    {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
        {
            // std::cout << "Sequence: " << it->first.as<std::string>() << ", Value: " << it->second.as<std::string>() << std::endl;
            readYamlNode(waypoint, *it);
        }
        break;
    }
    default:
        break;
    }
    return true;
}

bool ConfigFileHandler::readYamlNode(robot_arm::JointPose &pose, YAML::const_iterator &node_it)
{
    const YAML::Node &key = node_it->first;
    const YAML::Node &node = node_it->second;

    pose.id          = node["id"].as<int>();
    pose.description = node["description"].as<std::string>();
    pose.joint_1     = node["joint_1"].as<double>();
    pose.joint_2     = node["joint_2"].as<double>();
    pose.joint_3     = node["joint_3"].as<double>();
    pose.joint_4     = node["joint_4"].as<double>();
    pose.joint_5     = node["joint_5"].as<double>();
    pose.joint_6     = node["joint_6"].as<double>();

    return true;
}

bool ConfigFileHandler::readYamlNode(mission_data::ObjectIdentifier &object, YAML::const_iterator &node_it)
{
    const YAML::Node &key = node_it->first;
    const YAML::Node &node = node_it->second;

    object.id          = node["id"].as<int>();
    object.instance_id = node["instance_id"].as<int>();
    object.type        = node["type"].as<int>();
    object.description = node["description"].as<std::string>();

    return true;
}
