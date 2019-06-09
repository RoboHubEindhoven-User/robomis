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
    void updateMarker(std::string data);

    void updateFeedback(robomis::Waypoint data);

    void showWaypointsProperties(void);

private Q_SLOTS:
//    void on_pBtnPublish_clicked();
    void on_pBtnAdd_clicked();

    void on_pBtnClearAdd_clicked();

    void on_pBtnRemove_clicked();

    void on_rBtnManual_clicked();

    void on_rBtnFeedback_clicked();

    void on_pBtnPublish_clicked();

    void on_pBtnReset_clicked();
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

    void on_pBtnClearListWs_clicked();

    void on_pBtnAddOrder_clicked();

    void on_pBtnRemoveOrder_2_clicked();

    void on_pBtnRemoveOrder_clicked();

    void on_pBtnStartRotatingTable_clicked();

    void on_pBtnResetMission_clicked();

    void on_pBtnStartMission_clicked();

    void on_pBtnAddObject_clicked();

    void on_pBtnUpdateObject_clicked();

    void on_pBtnClearObject_clicked();

private:
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
    void clearTableWidget(void);
    void saveToFile();
    void loadFromFile();
    void saveFile();
    void openFile();
    Ui::MainWindowForm *ui;
    ziros::robomisNode rnode;
   QDoubleValidator     *doubleValivator;
   QIntValidator        *intValidator;
   QPalette             *palette;
   Configuration   configuration;
   bool                 shouldDisplayfeedback;
   geometry_msgs::Pose current_pose;
   QTimer *timer;

};
QDataStream &operator<<(QDataStream &out, const robomis::WaypointData &waypointData);
QDataStream &operator>>(QDataStream &in, robomis::WaypointData &waypointData);

#endif // MAINWINDOW_H
