cmake_minimum_required(VERSION 2.8.3)
project(robomis)
set(ROS_NODE "robomis_node")
## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

 ###########################################################################################
 ##                               QT Environment ##
 ###########################################################################################

find_package(catkin REQUIRED COMPONENTS
  qt_build
  roscpp
  rospy
  std_msgs
  sensor_msgs
  genmsg
  move_base_msgs
  geometry_msgs
  tf
  mission_protobuf
  mission_data_types
  mission_ros_msgs
  message_generation
)

find_package(yaml-cpp REQUIRED)

find_package(PkgConfig)
pkg_check_modules(YAML_CPP yaml-cpp)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY msg
   FILES
   Waypoint.msg
   WaypointData.msg
 )


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
#  Waypoint.msg
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS mission_protobuf mission_data_types mission_ros_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#set(CMAKE_CXX_FLAGS "-DGOOGLE_PROTOBUF_ARCH_X64 -DHAVE_THREAD ${CMAKE_CXX_FLAGS}")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${YAML_CPP_INCLUDEDIR}
    ${mission_protobuf_INCLUDE_DIRS}
    ${mission_data_types_INCLUDE_DIRS}
)


include_directories(${mission_protobuf_INCLUDE_DIRS})

if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()
include_directories(${YAML_CPP_INCLUDE_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

###########################################################################################
##                               QT Environment ##
###########################################################################################


find_package(Qt4 REQUIRED)
FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(QT ${QT} network ftp)

file( GLOB_RECURSE HDRS_FILES include/robomis/*.h include/robomis/*.hpp include/protonet/*.h)
file( GLOB_RECURSE SRCS_FILES src/*.cpp src/protonet/*.cpp)
file( GLOB_RECURSE UI_FILES src/*.ui )


qt4_wrap_cpp( MOC_SRCS ${HDRS_FILES} )
qt4_wrap_ui( UI_HDRS ${UI_FILES} )
qt4_wrap_ui(UI_HEADERS mainwindow.ui)

source_group( "UI Files" FILES ${UI_FILES} ${UI_HEADERS} )
source_group( "Generated Files" FILES ${MOC_SRCS} ${UI_HDRS} )

#add_library( ${ROS_NODE} STATIC ${SRCS_FILES} ${UI_FILES} ${HDRS_FILES} ${MOC_SRCS} ${UI_HDRS} )
rosbuild_prepare_qt4(QtCore QtGui)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

#declare a cpp executable
add_executable(
    ${ROS_NODE}
    ${SRCS_FILES}
    ${UI_FILES}
    ${UI_HEADERS}
    ${HDRS_FILES}
    ${UI_HDRS}
    )

add_dependencies(${ROS_NODE} robomis_generate_messages_cpp mission_protobuf_generate_messages_cpp)
add_dependencies(${ROS_NODE} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Add cmake target dependencies of the executable
target_link_libraries(
    ${ROS_NODE}
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
    Qt4::QtGui
    Qt4::QtDBus
    ${Boost_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    mission_planner_proto
    mission_data
    ${PROTOBUF_LIBRARIES}
)

target_link_libraries(${ROS_NODE} ${PROTOBUF_LIBRARIES})
#target_link_libraries(joy_teleop_node ${catkin_LIBRARIES})
## Specify libraries to link a library or executable target against

