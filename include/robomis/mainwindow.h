#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "robomisnode.hpp"
#include "configuration.h"
#include "robomis/Waypoint.h"
#include "robomis/WaypointData.h"
#include "robomis/LocationIdentifier.h"
#include <QDoubleValidator>
#include <QIntValidator>
#include <QPalette>
#include <QTimer>
#include <QMutex>

namespace Ui {
class MainWindowForm;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = 0);
//    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    /* Not to be called by user */

//    void updateMarker(std::string data);
//    void updateTodoTaskList(mission_ros_msgs::TaskObjective todos);
//    void updateCompletedTaskList(mission_ros_msgs::TaskObjective comp_tasks);
//    void updateCurrentTaskList(mission_ros_msgs::TaskObjective current_tasks);
//    void updateObjectHolderList(mission_ros_msgs::TaskObjective object_holder);

//    void updateFeedback(robomis::Waypoint data);

    void showWaypointsProperties(void);
    void on_tableViewGoalSelection(const QItemSelection &, const QItemSelection &);
    void startNode() { rnode.start(); }

private Q_SLOTS:
//    void tableGoalSelected(const QItemSelection & selected, const QItemSelection & deselected);
//    void on_pBtnPublish_clicked();
    void updateRobotArmPose();
    void updateOverViewLists();
    void updateMarker();
//    void updateFeedback();

    void on_pBtnAdd_clicked();

    void on_pBtnClearAdd_clicked();

    void on_pBtnRemove_clicked();

    void on_rBtnManual_clicked();

    void on_rBtnFeedback_clicked();

    void on_actionSave_triggered();

    void on_actionSave_As_triggered();

    void on_actionOpen_triggered();

    void updateRobotPositionDisplay();

    void on_pBtnUpdate_clicked();

    void on_tabWidgetProperties_currentChanged(int index);

    void on_pBtnBrowse_clicked();

    void on_pBtnLocalFileSave_clicked();

    void on_pBtnLoadFileLoad_clicked();

    void on_pBtnLocalFileCancel_clicked();

    void on_pBtnSshFileLoad_clicked();

    void on_pBtnSshFileSave_clicked();

    void on_pBtnSshFileCancel_clicked();

    void on_pBtnAddPose_clicked();

    void on_pBtnUpdatePose_clicked();

    void on_pBtnClearPose_clicked();

    void on_pBtnRemovePose_clicked();

    void on_pBtnOkWs_clicked();

    void on_pBtnClearWs_clicked();

    void on_pBtnAddOrder_clicked();

    void on_pBtnRemoveOrder_2_clicked();

    void on_pBtnRemoveOrder_clicked();

    void on_pBtnStartRotatingTable_clicked();

    void on_pBtnResetMission_clicked();

    void on_pBtnStartMission_clicked();

    void on_pBtnAddObject_clicked();

    void on_pBtnUpdateObject_clicked();

    void on_pBtnClearObject_clicked();

    void on_tableWidgetGoal_activated(const QModelIndex &index);

//    void on_tableWidgetGoal_currentItemChanged(QTableWidgetItem *current, QTableWidgetItem *previous);

    void on_button_connect_clicked();

    void on_rBtnAutoPose_clicked();

    void on_rBtnManualPose_clicked();

    void on_pBtnClearRobPoseList_clicked();

    void on_pBtnResetList_clicked();

private:

    void on_button_connect_clicked(bool state);
    bool locationAlreadyInList(mission_protobuf::LocationIdentifier::LocationType location);
    void displayWaypoints();
    void displayServiceAreaProp();
    void displayRobotArmPoses();
    void displayPlannerObjects();
    void displayPlannerServiceArea();
    void displayOrders();
    void displayTodoTasks();
    void displayCompletedTask();
    void displayCurrentTask();
    void displayObjectHolder();
    void updateLineEdit(bool readOnly, QColor baseColor, QColor textColor);
    void updateLineEditJointPose(bool readOnly, QColor baseColor, QColor textColor);

    void updateTodoTaskList(int task_type);
    void updateCompletedTaskList(int task_type);
    void updateCurrentTaskList(int task_type);
    void updateObjectHolderList();

    void clearTableWidget(void);
    void saveToFile();
    void loadFromFile();
    void saveFile();
    void openFile();
    Ui::MainWindowForm *ui;
    ziros::RobomisNode rnode;
   QDoubleValidator     *doubleValivator;
   QIntValidator        *intValidator;
   QPalette             *palette;
   Configuration   configuration;
   bool                 shouldDisplayfeedback;
   geometry_msgs::Pose current_pose;
   QTimer *timer1, *timer2, *timer3;
   QModelIndexList *selected_items;
   QMutex     mutex;

};
QDataStream &operator<<(QDataStream &out, const robomis::WaypointData &waypointData);
QDataStream &operator>>(QDataStream &in, robomis::WaypointData &waypointData);

#endif // MAINWINDOW_H
