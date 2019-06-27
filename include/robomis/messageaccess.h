#ifndef MESSAGEACCESS_H
#define MESSAGEACCESS_H

#include <QMutex>
#include <mission_ros_msgs/TaskObjective.h>
#include <mission_ros_msgs/Time.h>
#include <sensor_msgs/JointState.h>
#include <mission_data_types/robot_arm.h>
#include <vector>
#include <geometry_msgs/Pose.h>

class MessageAccess {
public:
//    ~MessageAccess() {}

//    void lock(QMutex* mutex);

//    void setRobotArmPose(robot_arm::JointPose pose);

//    robot_arm::JointPose robotArmPose(void) const;

//    void setTaskType(int type);

//    int taskType(void) const;

//    void setTodoTask(mission_ros_msgs::Task task);

//    std::vector<mission_ros_msgs::Task> todoTasks(void) const;

//    void setCompletedTask(mission_ros_msgs::Task task);

//    std::vector<mission_ros_msgs::Task> completedTasks(void) const;

//    void setCurrentTask(mission_ros_msgs::Task task);

//    std::vector<mission_ros_msgs::Task> currentTasks(void) const;

//    void addToObjectHolderList(mission_ros_msgs::Task task);

//    std::vector<mission_ros_msgs::Task> objectHolderList(void) const;

//    // It returns a reference, so capture a reference
//    static MessageAccess& get();
    void lock(QMutex* mutex){
        this->mutex = mutex;
    }

    void setRobotPosition(geometry_msgs::Pose pose) {
        QMutexLocker locker(mutex);
        robot_pose = pose;
    }

    void shouldGetRobotPosition(bool value) {
        QMutexLocker locker(mutex);
        should_get_robot_position = value;
    }

    bool getShouldGetRobotPosition() const {
        QMutexLocker locker(mutex);
        return should_get_robot_position;
    }

    geometry_msgs::Pose getRobotPosition() const{
        QMutexLocker locker(mutex);
        return robot_pose;
    }

    void setRobotArmPose(robot_arm::JointPose pose) {
        robot_arm_pose = pose;
    }

    robot_arm::JointPose robotArmPose(void) const{
        QMutexLocker locker(mutex);
        return robot_arm_pose;
    }

    void setTaskType(int type) {
        QMutexLocker locker(mutex);
        this->type = type;
    }

    int taskType(void) const {
         QMutexLocker locker(mutex);
         return type;
    }

    void setTodoTask(mission_ros_msgs::Task task){
        QMutexLocker locker(mutex);
        todo_tasks.push_back(task);
    }

    std::vector<mission_ros_msgs::Task> todoTasks(void) const {
        QMutexLocker locker(mutex);
        return todo_tasks;
    }

    void setCompletedTask(mission_ros_msgs::Task task){
        QMutexLocker locker(mutex);
        completed_task.push_back(task);
    }

    std::vector<mission_ros_msgs::Task> completedTasks(void) const {
        QMutexLocker locker(mutex);
        return completed_task;
    }

    void setCurrentTask(mission_ros_msgs::Task task){
        QMutexLocker locker(mutex);
        current_tasks.push_back(task);
    }

    std::vector<mission_ros_msgs::Task> currentTasks(void) const {
        QMutexLocker locker(mutex);
        return current_tasks;
    }

    void addToObjectHolderList(mission_ros_msgs::Task task){
        QMutexLocker locker(mutex);
        object_holder_list.push_back(task);
    }

    std::vector<mission_ros_msgs::Task> objectHolderList(void) const {
        QMutexLocker locker(mutex);
        return object_holder_list;
    }

    static MessageAccess& get()
    {
        static MessageAccess instance;
        return instance;
    }


        MessageAccess(MessageAccess const&)               = delete;
        void operator=(MessageAccess const&)  = delete;
//        MessageAccess& operator=(MessageAccess const&) = delete;
private:
        MessageAccess() {}
     // Disallow instantiation outside of the class.
    /* To prevent making a copy of the singleton instance (or moving from it!),
     * these 4 special functions should be deleted, like so
    */
//    MessageAccess(const MessageAccess&)            = delete;
//    MessageAccess& operator=(const MessageAccess&) = delete;
//    MessageAccess(MessageAccess&&)                 = delete;
//    MessageAccess& operator=(MessageAccess&&)      = delete;

//    Q_DISABLE_COPY(MessageAccess)
    QMutex*                             mutex;
    int                                 type;
    bool                                should_get_robot_position;
    geometry_msgs::Pose                 robot_pose;
    robot_arm::JointPose                robot_arm_pose;
    std::vector<mission_ros_msgs::Task> todo_tasks;
    std::vector<mission_ros_msgs::Task> completed_task;
    std::vector<mission_ros_msgs::Task> current_tasks;
    std::vector<mission_ros_msgs::Task> object_holder_list;
};



#endif // MESSAGEACCESS_H
