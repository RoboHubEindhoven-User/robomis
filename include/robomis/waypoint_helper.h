#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <string>
#include <mission_protobuf/task_action.pb.h>
#include <mission_protobuf/task_specification.pb.h>

class WaypointHelper {
 using LocationType = mission_protobuf::LocationIdentifier;

public:
    WaypointHelper(){ }

    ~WaypointHelper() = default;

    static bool isServiceArea(mission_protobuf::LocationIdentifier::LocationType type){
        return ((type == LocationType::WS) ||
                (type == LocationType::SH) ||
                (type == LocationType::CB) ||
                (type == LocationType::PP)
                ) ? true : false;
    }

    static std::string getDescription(mission_protobuf::LocationIdentifier::LocationType type, int instance_id) {
        return (type == LocationType::CB || type == LocationType::PP || type == LocationType::EN || type == LocationType::EX) ?
                    getLocationStrByType(type) : getLocationStrByType(type) + " " + std::to_string(instance_id);
    }

    static int getNextInstanceID(int nr_of_waypoints) {
        int id = nr_of_waypoints + 1;
        return id;
    }


    static bool isUniqueWaypoint(mission_protobuf::LocationIdentifier::LocationType type){
        if (type == LocationType::CB || type == LocationType::PP ||
            type == LocationType::EN || type == LocationType::EX) {
            return true;
        }
        return false;
    }

    static std::string getLocationStrByIndex(int index) {
        switch(index) {
            case 0: return "Entrance";
            case 1: return "Shelf";
            case 2: return "Workstation";
            case 3: return "Conveyor belt";
            case 4: return "Way Point";
            case 5: return "Precision Platform";
            case 6: return "Exit";
            default: return "Invalid data";
        }
        return "Invalid data";
    }

    static mission_protobuf::Task::TaskType  getTaskType(int type)  {
        if (type == 1) return mission_protobuf::Task::TRANSPORTATION;
        else if (type == 2) if (type == 1) return mission_protobuf::Task::NAVIGATION;

        return mission_protobuf::Task::UNKNOWN;
    }

    static mission_protobuf::LocationIdentifier::LocationType getLocationByDescription(std::string description) {
        if(description == "Entrance") return LocationType::EN;
        if(description == "Shelf") return LocationType::SH;
        if(description == "Workstation") return LocationType::WS;
        if(description == "Conveyor belt") return LocationType::CB;
        if(description == "Way Point") return LocationType::WP;
        if(description == "Precision Platform") return LocationType::PP;
        if(description == "Exit") return LocationType::EX;
        return LocationType::NONE;
    }

    static mission_protobuf::LocationIdentifier::LocationType getLocationTypeByIndex(int index) {
        index = (index == 100) ? 6 : index;
      switch(index) {
        case 0: return LocationType::EN;
        case 1: return LocationType::SH;
        case 2: return LocationType::WS;
        case 3: return LocationType::CB;
        case 4: return LocationType::WP;
        case 5: return LocationType::PP;
        case 6: return LocationType::EX;
        default: return LocationType::NONE;
      }
      return LocationType::NONE;
    }

    static int getLocationTypeNumeral(mission_protobuf::LocationIdentifier::LocationType type) {
      switch(type) {
        case LocationType::EN: return 0;
        case LocationType::SH: return 1;
        case LocationType::WS: return 2;
        case LocationType::CB: return 3;
        case LocationType::WP: return 4;
        case LocationType::PP: return 5;
        case LocationType::ROBOT: return 6;
        case LocationType::EX: return 100;
        default: return -1;
      }
      return -1;
    }

    static std::string getLocationStrByType(mission_protobuf::LocationIdentifier::LocationType type) {
        switch(type) {
            case LocationType::EN: return "Entrance";
            case LocationType::SH: return "Shelf";
            case LocationType::WS: return "Workstation";
            case LocationType::CB: return "Conveyor belt";
            case LocationType::WP: return "Way Point";
            case LocationType::PP: return "Precision Platform";
            case LocationType::EX: return "Exit";
            default: return "Invalid data";
        }
        return "Invalid data";
    }
};


#endif // WAYPOINT_H
