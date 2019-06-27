#include "../include/robomis/mainwindow.h"
#include "../include/robomis/waypoint_helper.h"
#include "ui_mainwindow.h"
#include "robomis/Waypoint.h"
#include <QTableWidgetItem>
#include <QAbstractItemView>
#include <QMessageBox>
#include <QFileDialog>

#include <QFile>
#include <QDebug>
//#include <QFileDevice>
#include <sstream>
#include<cctype>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <mission_protobuf/task_specification.pb.h>
#include <robomis/messageaccess.h>


int counter = 0;
MainWindow *MainWindowPtr;
bool is_auto_display_pose = false;

auto& MessageKeeper = MessageAccess::get();

//auto& MessageKeeper = MessageAccess::Instance();

QDataStream &operator<<(QDataStream &out, const robomis::WaypointData &waypointData) {
    out << waypointData;
    return out;
}

QDataStream &operator>>(QDataStream &in, robomis::WaypointData &waypointData){
    in >> waypointData;
    return in;
}

/*************************************************************************************************************************
 *                                             Private C Functions
 * **********************************************************************************************************************/

int extractIntergerFromString(std::string str_name) {
   std::stringstream str_stream;
   int num = 0;

    for (size_t i = 0; i < str_name.length(); i++)
        if (!isdigit(str_name[i]))
           continue;
        else
           str_stream << str_name[i];
    str_stream >>  num ;
    return num;
}


/*************************************************************************************************************************
 *                                             Public Functions
 * **********************************************************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindowForm),
    rnode(argc,argv),
    doubleValivator(new QDoubleValidator( -10000.0, 10000.0, 1, parent )),
    intValidator(new QIntValidator( 0, 10000, parent)),
    palette(new QPalette())
//    timer(new QTimer(this))
{
    MainWindowPtr = this;

    ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
//    QObject::connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    QObject::connect(&rnode, SIGNAL(rosShutdown()), this, SLOT(close()));

//    ui->setupUi(this);
    ui->lineEditPosX->setValidator(doubleValivator);
    ui->lineEditPosX->setValidator(doubleValivator);
    ui->lineEditPosY->setValidator(doubleValivator);
    ui->lineEditPosZ->setValidator(doubleValivator);
    ui->lineEditOrientX->setValidator(doubleValivator);
    ui->lineEditOrientY->setValidator(doubleValivator);
    ui->lineEditOrientZ->setValidator(doubleValivator);
    ui->lineEditOrientW->setValidator(doubleValivator);

    ui->checkBoxMarkerScan->setStyleSheet("QCheckBox { color: white }");
    ui->rBtnManual->setStyleSheet("QRadioButton { color: white }");
    ui->rBtnFeedback->setStyleSheet("QRadioButton { color: white }");

    for (int i = 0; i <= 6; i++){
        ui->comboBoxName->addItem(QString::fromStdString(WaypointHelper::getLocationStrByIndex(i)));
    }

    ui->lineEditInstanceId->setText(QString::number(WaypointHelper::getNextInstanceID(configuration.getNrOfWaypointsWithType())));

    ui->tableWidgetGoal->verticalHeader()->setVisible(false); // Get Vertical header and hide it
    ui->tableWidgetGoal->setSelectionBehavior(QAbstractItemView::SelectRows);

    ui->tableWidgetWorkServiceAreaProp->verticalHeader()->setVisible(false); // Get Vertical header and hide it
    ui->tableWidgetWorkServiceAreaProp->setSelectionBehavior(QAbstractItemView::SelectRows);

    ui->tableWidgetRobPose->verticalHeader()->setVisible(false); // Get Vertical header and hide it
    ui->tableWidgetRobPose->setSelectionBehavior(QAbstractItemView::SelectRows);

    ui->tableWidgetObjects->verticalHeader()->setVisible(false); // Get Vertical header and hide it
    ui->tableWidgetObjects->setSelectionBehavior(QAbstractItemView::SelectRows);

    ui->tableWidgetObjectSource->verticalHeader()->setVisible(false); // Get Vertical header and hide it
    ui->tableWidgetObjectSource->setSelectionBehavior(QAbstractItemView::SelectRows);

    ui->tableWidgetObjectDestination->verticalHeader()->setVisible(false); // Get Vertical header and hide it
    ui->tableWidgetObjectDestination->setSelectionBehavior(QAbstractItemView::SelectRows);

    ui->tableWidgetToDo->verticalHeader()->setVisible(false); // Get Vertical header and hide it
    ui->tableWidgetToDo->setSelectionBehavior(QAbstractItemView::SelectRows);

    ui->tableWidgetCompTasks->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tableWidgetCompTasks->setFocusPolicy(Qt::NoFocus);
    ui->tableWidgetCompTasks->setSelectionMode(QAbstractItemView::NoSelection);

    ui->tableWidgetCurrTasks->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tableWidgetCurrTasks->setFocusPolicy(Qt::NoFocus);
    ui->tableWidgetCurrTasks->setSelectionMode(QAbstractItemView::NoSelection);

    ui->tableWidgetTodoTasks->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tableWidgetTodoTasks->setFocusPolicy(Qt::NoFocus);
    ui->tableWidgetTodoTasks->setSelectionMode(QAbstractItemView::NoSelection);

    ui->tableWidgetHolderList->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tableWidgetHolderList->setFocusPolicy(Qt::NoFocus);
    ui->tableWidgetHolderList->setSelectionMode(QAbstractItemView::NoSelection);

    ui->tableWidgetOrders->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tableWidgetOrders->setFocusPolicy(Qt::NoFocus);
    ui->tableWidgetOrders->setSelectionMode(QAbstractItemView::NoSelection);

//    connect(ui->tabWidgetProperties, SIGNAL(currentChanged(int)), this, SLOT(tabSelected()))

    rnode.init();

    timer1 = new QTimer(this);
    timer2 = new QTimer(this);
    timer3 = new QTimer(this);

    connect(timer1, SIGNAL(timeout()), this, SLOT(updateRobotPositionDisplay()));
//    timer->start(1000);

    connect(timer2, SIGNAL(timeout()), this, SLOT(updateRobotArmPose()));

    connect(timer3, SIGNAL(timeout()), this, SLOT(updateOverViewLists()));
//    connect(
//     ui->tableWidgetGoal->selectionModel(),
//     SIGNAL(clicked(const QModelIndex&)), this, SLOT(tableGoalSelected())
//    );
//    QObject::connect(&rnode, SIGNAL(rnode.newPosereceived()), this, SLOT(updateRobotPositionDisplay()));
//    connect(ui->tableWidgetGoal, SIGNAL(selectionChanged()), this, SLOT(on_tableViewGoalSelection()));
//    connect(ui->tableWidgetGoal->selectionModel(),
//                 SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)),
//                        this,
//                        SLOT(on_tableViewGoalSelection(const QItemSelection &, const QItemSelection &)));

    /*********************
    ** Auto Start
    **********************/
//    if ( ui->checkbox_remember_settings->isChecked() ) {
//        on_button_connect_clicked(true);
////        rnode.start();
//    }
}

MainWindow::~MainWindow() {
    delete ui;
    delete intValidator;
    delete doubleValivator;
}

void MainWindow::on_tableViewGoalSelection(const QItemSelection &selected, const QItemSelection &deselected){
    QMessageBox messageBox;

    QModelIndexList items = selected.indexes();
    Waypoint *wp = configuration.getWaypoint(items[0].data().toInt());
//    int index = WaypointHelper::getLocationTypeNumeral();
    std::string location = WaypointHelper::getLocationStrByIndex(wp->location.type);

    for ( int i = 0; i < ui->comboBoxName->count(); i++) {
        if(ui->comboBoxName->itemText(i) == QString::fromStdString(location)){
//            ui->comboBoxName->setCurrentIndex(i);
        }
    }
    ui->lineEditInstanceId->setText(QString::number(wp->location.instance_id));
    ui->lineEditPosX->setText(QString("%1").arg(wp->pose.position.x, 0, 'f', 3));
    ui->lineEditPosY->setText(QString("%1").arg(wp->pose.position.y, 0, 'f', 3));
    ui->lineEditPosZ->setText(QString("%1").arg(wp->pose.position.z, 0, 'f', 3));
    ui->lineEditOrientX->setText(QString("%1").arg(wp->pose.orientation.x, 0, 'f', 3));
    ui->lineEditOrientY->setText(QString("%1").arg(wp->pose.orientation.y, 0, 'f', 3));
    ui->lineEditOrientZ->setText(QString("%1").arg(wp->pose.orientation.z, 0, 'f', 3));
    ui->lineEditOrientW->setText(QString("%1").arg(wp->pose.orientation.w, 0, 'f', 3));

    items = deselected.indexes();

//         foreach (index, items)
//             model->setData(index, "");
}


void MainWindow::updateMarker() {
//    int id = extractIntergerFromString(data);
//    ui->lineEditWaypointMarker->setText(QString::fromStdString (data));
//    ui->lineEditWaypointMarkerId->setText(QString::number(id));
//    ui->lineEditExpWaypointID->setText(QString::number(id));
//    std::cout << data <<std::endl;
}


void MainWindow::updateRobotPositionDisplay() {
    if(shouldDisplayfeedback) {
        MessageKeeper.lock(&mutex);
        geometry_msgs::Pose pose = MessageKeeper.getRobotPosition();

        ui->lineEditPosX->setText(QString("%1").arg(pose.position.x, 0, 'f', 3));
        ui->lineEditPosY->setText(QString("%1").arg(pose.position.y, 0, 'f', 3));
        ui->lineEditPosZ->setText(QString("%1").arg(pose.position.z, 0, 'f', 3));
        ui->lineEditOrientX->setText(QString("%1").arg(pose.orientation.x, 0, 'f', 3));
        ui->lineEditOrientY->setText(QString("%1").arg(pose.orientation.y, 0, 'f', 3));
        ui->lineEditOrientZ->setText(QString("%1").arg(pose.orientation.z, 0, 'f', 3));
        ui->lineEditOrientW->setText(QString("%1").arg(pose.orientation.w, 0, 'f', 3));
    }
}


void MainWindow::updateRobotArmPose() {

//    if (is_auto_display_pose) {
        printf("updateRobotArmPose...\n");
        MessageKeeper.lock(&mutex);
        robot_arm::JointPose pose = MessageKeeper.robotArmPose();
        ui->lineEditPoseJ1->setText(QString::number(pose.joint_1));
        ui->lineEditPoseJ2->setText(QString::number(pose.joint_2));
        ui->lineEditPoseJ3->setText(QString::number(pose.joint_3));
        ui->lineEditPoseJ4->setText(QString::number(pose.joint_4));
        ui->lineEditPoseJ5->setText(QString::number(pose.joint_5));
        ui->lineEditPoseJ6->setText(QString::number(pose.joint_6));

}

void MainWindow::updateOverViewLists() {
    MessageKeeper.lock(&mutex);
    int task_type = MessageKeeper.taskType();
    updateTodoTaskList(task_type);
    updateCompletedTaskList(task_type);
    updateCurrentTaskList(task_type);
    updateObjectHolderList();
}

void MainWindow::updateTodoTaskList(int task_type) {
    ui->tableWidgetTodoTasks->clear();
    ui->tableWidgetTodoTasks->model()->removeRows(0, ui->tableWidgetTodoTasks->rowCount());


    std::vector<mission_ros_msgs::Task> tasks = MessageKeeper.todoTasks();

    for (size_t i = 0; i < tasks.size(); i++) {
        /* add a new row */
        ui->tableWidgetTodoTasks->insertRow(static_cast<int>(i));
        /* insert to columns of the new row*/
        int row_index = ui->tableWidgetTodoTasks->rowCount()-1;

        if (WaypointHelper::getTaskType(task_type) == mission_protobuf::Task::TRANSPORTATION){

            ui->tableWidgetTodoTasks->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(tasks[i].transportation.object.description)));
            ui->tableWidgetTodoTasks->setItem(  row_index,  1,  new QTableWidgetItem(QString::fromStdString(tasks[i].transportation.source.description)));
            ui->tableWidgetTodoTasks->setItem(  row_index,  2,  new QTableWidgetItem(QString::fromStdString(tasks[i].transportation.destination.description)));

        }else if (WaypointHelper::getTaskType(task_type) == mission_protobuf::Task::NAVIGATION){
            ui->tableWidgetTodoTasks->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(tasks[i].navigation.destination.description)));
        }
    }
}

void MainWindow::updateCompletedTaskList(int task_type) {
    ui->tableWidgetCompTasks->clear();
    ui->tableWidgetCompTasks->model()->removeRows(0, ui->tableWidgetCompTasks->rowCount());

     std::vector<mission_ros_msgs::Task> tasks = MessageKeeper.completedTasks();

    for (size_t i = 0; i < tasks.size(); i++) {
        /* add a new row */
        ui->tableWidgetCompTasks->insertRow(static_cast<int>(i));
        /* insert to columns of the new row*/
        int row_index = ui->tableWidgetCompTasks->rowCount()-1;

        if (WaypointHelper::getTaskType(task_type) == mission_protobuf::Task::TRANSPORTATION){

            ui->tableWidgetCompTasks->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(tasks[i].transportation.object.description)));
            ui->tableWidgetCompTasks->setItem(  row_index,  1,  new QTableWidgetItem(QString::fromStdString(tasks[i].transportation.destination.description)));

        }else if (WaypointHelper::getTaskType(task_type) == mission_protobuf::Task::NAVIGATION){
            ui->tableWidgetCompTasks->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(tasks[i].navigation.destination.description)));
        }
    }
}

void MainWindow::updateCurrentTaskList(int task_type) {
    ui->tableWidgetCurrTasks->clear();
    ui->tableWidgetCurrTasks->model()->removeRows(0, ui->tableWidgetCurrTasks->rowCount());

     std::vector<mission_ros_msgs::Task> tasks = MessageKeeper.currentTasks();

    for (size_t i = 0; i < tasks.size(); i++) {
        /* add a new row */
        ui->tableWidgetCurrTasks->insertRow(static_cast<int>(i));
        /* insert to columns of the new row*/
        int row_index = ui->tableWidgetCurrTasks->rowCount()-1;

        if (WaypointHelper::getTaskType(task_type) == mission_protobuf::Task::TRANSPORTATION){

            ui->tableWidgetCurrTasks->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(tasks[i].transportation.object.description)));
            ui->tableWidgetCurrTasks->setItem(  row_index,  1,  new QTableWidgetItem(QString::fromStdString(tasks[i].transportation.source.description)));

        }else if (WaypointHelper::getTaskType(task_type) == mission_protobuf::Task::NAVIGATION){
            ui->tableWidgetCurrTasks->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(tasks[i].navigation.destination.description)));
        }
    }
}

void MainWindow::updateObjectHolderList() {
    ui->tableWidgetHolderList->clear();
    ui->tableWidgetHolderList->model()->removeRows(0, ui->tableWidgetHolderList->rowCount());

    std::vector<mission_ros_msgs::Task> holder_list = MessageKeeper.objectHolderList();

    for (size_t i = 0; i < holder_list.size(); i++) {
        /* add a new row */
        ui->tableWidgetHolderList->insertRow(static_cast<int>(i));
        /* insert to columns of the new row*/
        int row_index = ui->tableWidgetHolderList->rowCount()-1;

        ui->tableWidgetHolderList->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(holder_list[i].transportation.object.description)));
        ui->tableWidgetHolderList->setItem(  row_index,  1,  new QTableWidgetItem(QString::fromStdString(holder_list[i].transportation.destination.description)));
    }
}


/*************************************************************************************************************************
 *                                             Private Functions
 * **********************************************************************************************************************/
bool MainWindow::locationAlreadyInList(mission_protobuf::LocationIdentifier::LocationType location) {
//    robomis::WaypointData  *msg = rnode.getWaypointData();
    std::vector<Waypoint> waypoints = configuration.getWaypoints();

    for(size_t i = 0; i < waypoints.size(); i++)
        if(waypoints[i].location.type == WaypointHelper::getLocationTypeNumeral(location))
            return true;

    return false;
}

void MainWindow::on_pBtnAdd_clicked()
{
//    ui->lineEditInstanceId->setText(QString::number(configuration.getNrOfWaypointsWithType()));

    int index = ui->comboBoxName->currentIndex();

    mission_protobuf::LocationIdentifier::LocationType loca_type = WaypointHelper::getLocationTypeByIndex(index);

    ui->lineEditInstanceId->setText(QString::number(WaypointHelper::getNextInstanceID(configuration.getNrOfWaypointsWithType(loca_type))));

    if (WaypointHelper::isUniqueWaypoint(loca_type) && locationAlreadyInList(loca_type)){
        QMessageBox messageBox;
        messageBox.critical(0,"Error", "Service Area already in List. Only one " + ui->comboBoxName->currentText() + " area can be added.");
        messageBox.setFixedSize(500,200);
        return;
   }


    bool scan = ui->checkBoxMarkerScan->isChecked() ? true : false;
    int instance_id   = ui->lineEditInstanceId->text().toInt();

    Location location(
                WaypointHelper::getLocationTypeNumeral(loca_type),
                instance_id,
                WaypointHelper::getDescription(loca_type, instance_id)
                );

    geometry::Pose pose;

    if (ui->rBtnManual->isChecked()){
        pose = geometry::Pose(
                    geometry::Pose::Point(
                        ui->lineEditPosX->text().toDouble(),
                        ui->lineEditPosY->text().toDouble(),
                        ui->lineEditPosZ->text().toDouble()
                        ),
                    geometry::Pose::Quaternion(
                        ui->lineEditOrientX->text().toDouble(),
                        ui->lineEditOrientY->text().toDouble(),
                        ui->lineEditOrientZ->text().toDouble(),
                        ui->lineEditOrientW->text().toDouble()
                       )
                    );
    }else if (ui->rBtnFeedback->isChecked()){
        MessageKeeper.lock(&mutex);
        geometry_msgs::Pose robot_pose = MessageKeeper.getRobotPosition();
        pose = geometry::Pose(
                    geometry::Pose::Point(
                        robot_pose.position.x,
                        robot_pose.position.y,
                        robot_pose.position.z
                        ),
                    geometry::Pose::Quaternion(
                        robot_pose.orientation.x,
                        robot_pose.orientation.y,
                        robot_pose.orientation.z,
                        robot_pose.orientation.w
                       )
                    );
    }


    Waypoint waypoint(location, pose);
    waypoint.should_scan = scan;

    configuration.addWaypoint(waypoint);
    /* Display in the Table Widget */
    displayWaypoints();
    displayServiceAreaProp();
}

void MainWindow::displayWaypoints() {
    /* Clear the table widget */
    ui->tableWidgetGoal->clearContents();
    ui->tableWidgetGoal->model()->removeRows(0, ui->tableWidgetGoal->rowCount());

    std::vector<Waypoint> waypoints = configuration.getWaypoints();
    int sub_index = 0;
    for(size_t row = 0; row < waypoints.size(); row++) {

        Waypoint waypoint = waypoints[row];
        /* add a new row */
        ui->tableWidgetGoal->insertRow(static_cast<int>(row));
        /* insert to columns of the new row*/
        int row_index = ui->tableWidgetGoal->rowCount()-1;
        QString scan = waypoint.should_scan == true ? "Enabled" : "Disabled";

        ui->tableWidgetGoal->setItem(  row_index,  0,  new QTableWidgetItem(QString::number(waypoint.id)));
        ui->tableWidgetGoal->setItem(  row_index,  1,  new QTableWidgetItem(QString::fromStdString(waypoint.location.description)));
        ui->tableWidgetGoal->setItem(  row_index,  2,  new QTableWidgetItem(scan));
        ui->tableWidgetGoal->setItem(  row_index,  3,  new QTableWidgetItem(QString::number(waypoint.pose.position.x)));
        ui->tableWidgetGoal->setItem(  row_index,  4,  new QTableWidgetItem(QString::number(waypoint.pose.position.y)));
        ui->tableWidgetGoal->setItem(  row_index,  5,  new QTableWidgetItem(QString::number(waypoint.pose.position.z)));
        ui->tableWidgetGoal->setItem(  row_index,  6,  new QTableWidgetItem(QString::number(waypoint.pose.orientation.x)));
        ui->tableWidgetGoal->setItem(  row_index,  7,  new QTableWidgetItem(QString::number(waypoint.pose.orientation.y)));
        ui->tableWidgetGoal->setItem(  row_index,  8,  new QTableWidgetItem(QString::number(waypoint.pose.orientation.z)));
        ui->tableWidgetGoal->setItem(  row_index,  9,  new QTableWidgetItem(QString::number(waypoint.pose.orientation.w)));

    }

}


void MainWindow::displayServiceAreaProp() {
    /* Clear the table widget */

    ui->tableWidgetWorkServiceAreaProp->clearContents();
    ui->tableWidgetWorkServiceAreaProp->model()->removeRows(0, ui->tableWidgetWorkServiceAreaProp->rowCount());

    ui->tableWidgetObjectSource->clearContents();
    ui->tableWidgetObjectSource->model()->removeRows(0, ui->tableWidgetObjectSource->rowCount());

    ui->tableWidgetObjectDestination->clearContents();
    ui->tableWidgetObjectDestination->model()->removeRows(0, ui->tableWidgetObjectDestination->rowCount());

    std::vector<Waypoint> waypoints = configuration.getWaypoints();
    int table_row = 0;

    for(size_t i = 0; i < waypoints.size(); i++) {

        Waypoint waypoint = waypoints[i];

        if(WaypointHelper::isServiceArea(WaypointHelper::getLocationTypeByIndex(waypoint.location.type))){
            /* add a new row */
            ui->tableWidgetWorkServiceAreaProp->insertRow(static_cast<int>(table_row));
            /* insert to columns of the new row*/
            int row_index = ui->tableWidgetWorkServiceAreaProp->rowCount()-1;

            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  0,  new QTableWidgetItem(QString::number(waypoint.id)));
            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  1,  new QTableWidgetItem(QString::fromStdString(waypoint.location.description)));
            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  2,  new QTableWidgetItem(QString::number(waypoint.service_area.length)));
            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  3,  new QTableWidgetItem(QString::number(waypoint.service_area.width)));
            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  4,  new QTableWidgetItem(QString::number(waypoint.service_area.height)));
            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  5,  new QTableWidgetItem(QString::number(waypoint.service_area.diameter)));
            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  6,  new QTableWidgetItem(QString::number(waypoint.service_area.tilt_angle)));


            ui->tableWidgetObjectSource->insertRow(static_cast<int>(table_row));
            row_index = ui->tableWidgetObjectSource->rowCount()-1;
            ui->tableWidgetObjectSource->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(waypoint.location.description)));

            ui->tableWidgetObjectDestination->insertRow(static_cast<int>(table_row));
            row_index = ui->tableWidgetObjectDestination->rowCount()-1;
            ui->tableWidgetObjectDestination->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(waypoint.location.description)));

            table_row++;
        }


    }


}

void MainWindow::displayRobotArmPoses() {
    ui->tableWidgetRobPose->clearContents();
    ui->tableWidgetRobPose->model()->removeRows(0, ui->tableWidgetRobPose->rowCount());

    std::vector<robot_arm::JointPose> poses = configuration.getRobotArmPoses();
    int sub_index = 0;
    for(size_t row = 0; row < poses.size(); row++) {

       robot_arm::JointPose pose = poses[row];
        /* add a new row */
        ui->tableWidgetRobPose->insertRow(static_cast<int>(row));
        /* insert to columns of the new row*/
        int row_index = ui->tableWidgetRobPose->rowCount()-1;

        ui->tableWidgetRobPose->setItem(  row_index,  0,  new QTableWidgetItem(QString::number(pose.id)));
        ui->tableWidgetRobPose->setItem(  row_index,  1,  new QTableWidgetItem(QString::fromStdString(pose.description)));
        ui->tableWidgetRobPose->setItem(  row_index,  2,  new QTableWidgetItem(QString::number(pose.joint_1)));
        ui->tableWidgetRobPose->setItem(  row_index,  3,  new QTableWidgetItem(QString::number(pose.joint_2)));
        ui->tableWidgetRobPose->setItem(  row_index,  4,  new QTableWidgetItem(QString::number(pose.joint_3)));
        ui->tableWidgetRobPose->setItem(  row_index,  5,  new QTableWidgetItem(QString::number(pose.joint_4)));
        ui->tableWidgetRobPose->setItem(  row_index,  6,  new QTableWidgetItem(QString::number(pose.joint_5)));
        ui->tableWidgetRobPose->setItem(  row_index,  7,  new QTableWidgetItem(QString::number(pose.joint_6)));

    }
}

void MainWindow::displayPlannerObjects() {
  // from configurations
    ui->tableWidgetObjects->clearContents();
    ui->tableWidgetObjects->model()->removeRows(0, ui->tableWidgetObjects->rowCount());

//    std::vector<Waypoints> waypoint = configuration.getWaypoints();
//    int sub_index = 0;
//    for(size_t row = 0; row < poses.size(); row++) {

//       robot_arm::JointPose pose = poses[row];
//        /* add a new row */
//        ui->tableWidgetObjects->insertRow(static_cast<int>(row));
//        /* insert to columns of the new row*/
//        int row_index = ui->tableWidgetObjects->rowCount()-1;

//        ui->tableWidgetObjects->setItem(  row_index,  0,  new QTableWidgetItem(QString::number(pose.id)));
//        ui->tableWidgetObjects->setItem(  row_index,  1,  new QTableWidgetItem(QString::fromStdString(pose.description)));
//        ui->tableWidgetObjects->setItem(  row_index,  2,  new QTableWidgetItem(QString::number(pose.joint_1)));

//    }
}

void MainWindow::displayOrders() {

}

void MainWindow::displayTodoTasks() {
// from robomis node
}

void MainWindow::displayCompletedTask() {
// from robomis node
}

void MainWindow::displayCurrentTask() {
// from robomis node
}

void MainWindow::displayObjectHolder() {
    // from robomis node
}


void MainWindow::clearTableWidget(void) {


}

void MainWindow::updateLineEdit(bool readOnly, QColor baseColor, QColor textColor) {
    ui->lineEditPosX->setReadOnly(readOnly);
    ui->lineEditPosY->setReadOnly(readOnly);
    ui->lineEditPosZ->setReadOnly(readOnly);
    ui->lineEditOrientX->setReadOnly(readOnly);
    ui->lineEditOrientY->setReadOnly(readOnly);
    ui->lineEditOrientZ->setReadOnly(readOnly);
    ui->lineEditOrientW->setReadOnly(readOnly);

    palette->setColor(QPalette::Base, baseColor);
    palette->setColor(QPalette::Text, textColor);

    ui->lineEditPosX->setPalette(*palette);
    ui->lineEditPosY->setPalette(*palette);
    ui->lineEditPosZ->setPalette(*palette);
    ui->lineEditOrientX->setPalette(*palette);
    ui->lineEditOrientY->setPalette(*palette);
    ui->lineEditOrientZ->setPalette(*palette);
    ui->lineEditOrientW->setPalette(*palette);
}

void MainWindow:: updateLineEditJointPose(bool readOnly, QColor baseColor, QColor textColor) {
    ui->lineEditPoseJ1->setReadOnly(readOnly);
    ui->lineEditPoseJ2->setReadOnly(readOnly);
    ui->lineEditPoseJ3->setReadOnly(readOnly);
    ui->lineEditPoseJ4->setReadOnly(readOnly);
    ui->lineEditPoseJ5->setReadOnly(readOnly);
    ui->lineEditPoseJ6->setReadOnly(readOnly);

    palette->setColor(QPalette::Base, baseColor);
    palette->setColor(QPalette::Text, textColor);

    ui->lineEditPoseJ1->setPalette(*palette);
    ui->lineEditPoseJ2->setPalette(*palette);
    ui->lineEditPoseJ3->setPalette(*palette);
    ui->lineEditPoseJ4->setPalette(*palette);
    ui->lineEditPoseJ5->setPalette(*palette);
    ui->lineEditPoseJ6->setPalette(*palette);
}

void MainWindow::on_pBtnClearAdd_clicked()
{
    ui->lineEditPosX->setText("");
    ui->lineEditPosY->setText("");
    ui->lineEditPosZ->setText("0");
    ui->lineEditOrientX->setText("0");
    ui->lineEditOrientY->setText("0");
    ui->lineEditOrientZ->setText("");
    ui->lineEditOrientW->setText("");
}

void MainWindow::on_pBtnRemove_clicked()
{
    QModelIndexList selection = ui->tableWidgetGoal->selectionModel()->selectedRows();

    if (selection.count() <= 0){
        QMessageBox messageBox;
        messageBox.critical(0, "Error", "Waypoint must be selected from the waypoint list!");
        messageBox.setFixedSize(500,200);
        return;
    }

    for(int i = 0; i < selection.count(); i++) {
        QModelIndex index = selection.at(i);
        configuration.deleteWaypoint(index.data().toInt());
    }

    displayWaypoints();
}

void MainWindow::on_rBtnManual_clicked()
{
    updateLineEdit(false, Qt::white, Qt::black);
    shouldDisplayfeedback = false;
    MessageKeeper.lock(&mutex);
    if(MessageKeeper.getShouldGetRobotPosition()){
        MessageKeeper.shouldGetRobotPosition(false);
        timer1->stop();
    }
}

void MainWindow::on_rBtnFeedback_clicked()
{
    updateLineEdit(true, Qt::gray, Qt::black);
    shouldDisplayfeedback = true;
    MessageKeeper.lock(&mutex);
    if(!MessageKeeper.getShouldGetRobotPosition()){
        MessageKeeper.shouldGetRobotPosition(true);
        timer1->start(100);
    }
}


void MainWindow::on_rBtnAutoPose_clicked()
{
    updateLineEditJointPose(true, Qt::gray, Qt::black);
    timer2->start(100);
}

void MainWindow::on_rBtnManualPose_clicked()
{
    updateLineEditJointPose(false, Qt::white, Qt::black);
    timer2->stop();
}

std::string getPathName(const std::string& filename) {

   char sep = '/';

#ifdef _WIN32
   sep = '\\';
#endif

   size_t pos = filename.rfind(sep, filename.length());

   if (pos != std::string::npos) {
      return(filename.substr(0, pos));
   }

   return("");
}



void MainWindow::saveFile() {
    QString fileName = QFileDialog::getSaveFileName(
                this,
                tr("Save Service Area Configuration"),
                QDir::currentPath(),
                tr("Binary Files (*.bin)"));

        if (fileName != "")
        {
            QFile file(QFileInfo(fileName).absoluteFilePath());

            if (file.exists())
            {
                QMessageBox::StandardButton chosenButton
                    = QMessageBox::warning(this, tr("File exists"), tr("The file already exists. Do you want to overwrite it?"),
                        QMessageBox::Ok | QMessageBox::Cancel,
                        QMessageBox::Cancel);
                if (chosenButton != QMessageBox::Ok)
                {
                    return; //Save was cancelled
                }
            }
            if (!file.open(QIODevice::WriteOnly))
            {
                QMessageBox::critical(this, tr("Error"), tr("Failed to save file"));
                return; //Aborted
            }
            //All ok - save data
//            QString text = ui->plainTextEdit->toPlainText();
//            QTextStream out(&file);
            QDataStream out(&file);
            out << rnode.getWaypointData();
            file.close();
        }
}

void MainWindow::openFile() {

    QFileDialog fileDialog(this);
    fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
    fileDialog.setWindowTitle(tr("Open Files"));
//    QStringList supportedMimeTypes = m_player->supportedMimeTypes();
//    if (!supportedMimeTypes.isEmpty()) {
//        supportedMimeTypes.append("audio/x-m3u"); // MP3 playlists
//        fileDialog.setMimeTypeFilters(supportedMimeTypes);
//    }
    fileDialog.setOption(QFileDialog::DontUseNativeDialog, true); //we need qt layout
//    fileDialog.setDirectory(QStandardPaths::standardLocations(QStandardPaths::MoviesLocation).value(0, QDir::homePath()));
    if (fileDialog.exec() == QDialog::Accepted){}
//        addToPlaylist(fileDialog.selectedUrls());
  }



void MainWindow::on_actionSave_triggered()
{
    saveFile();
}

void MainWindow::on_actionSave_As_triggered()
{
    saveFile();
}

void MainWindow::on_actionOpen_triggered()
{
    openFile();
}


void MainWindow::on_pBtnUpdate_clicked()
{
    QModelIndexList selection = ui->tableWidgetGoal->selectionModel()->selectedRows();

    if (selection.count() <= 0){
        QMessageBox messageBox;
        messageBox.critical(0, "Error", "Waypoint must be selected from the waypoint list!");
        messageBox.setFixedSize(500,200);
        return;
    }

    for(int i = 0; i < selection.count(); i++) {
        QModelIndex index = selection.at(i);

        Waypoint *waypoint = configuration.getWaypoint(index.data().toInt());

        std::string name  = ui->comboBoxName->currentText().toUtf8().constData();
        bool scan         = ui->checkBoxMarkerScan->isChecked() ? true : false;

        mission_protobuf::LocationIdentifier::LocationType loca_type = WaypointHelper::getLocationTypeByIndex(ui->comboBoxName->currentIndex());

        waypoint->location.type         = WaypointHelper::getLocationTypeNumeral(loca_type);
        waypoint->location.description  = WaypointHelper::getLocationStrByType(loca_type);
        waypoint->should_scan           = scan;

        if (ui->rBtnFeedback->isChecked()){
            MessageKeeper.lock(&mutex);
            geometry_msgs::Pose robot_pose = MessageKeeper.getRobotPosition();

            waypoint->pose.position.x       = robot_pose.position.x;
            waypoint->pose.position.x       = robot_pose.position.y;
            waypoint->pose.position.x       = robot_pose.position.z;
            waypoint->pose.orientation.x    = robot_pose.orientation.x;
            waypoint->pose.orientation.y    = robot_pose.orientation.y;
            waypoint->pose.orientation.z    = robot_pose.orientation.z;
            waypoint->pose.orientation.w    = robot_pose.orientation.w;

        }else{
            waypoint->pose.position.x       = ui->lineEditPosX->text().toDouble();
            waypoint->pose.position.x       = ui->lineEditPosY->text().toDouble();
            waypoint->pose.position.x       = ui->lineEditPosZ->text().toDouble();
            waypoint->pose.orientation.x    = ui->lineEditOrientX->text().toDouble();
            waypoint->pose.orientation.y    = ui->lineEditOrientY->text().toDouble();
            waypoint->pose.orientation.z    = ui->lineEditOrientZ->text().toDouble();
            waypoint->pose.orientation.w    = ui->lineEditOrientW->text().toDouble();
        }
//        configuration.updateWaypoint(i, waypoint);



    }
    ui->tableWidgetGoal->clearSelection();
    displayWaypoints();
}



void MainWindow::on_tabWidgetProperties_currentChanged(int index)
{
//    if (index == 1) {
//        showWaypointsProperties();
//    }
}

void MainWindow::on_pBtnBrowse_clicked()
{

}

void MainWindow::on_pBtnLocalFileSave_clicked()
{
    std::string filename = ui->lineEditLocalFilename->text().toUtf8().constData();
    bool successful = false;
    QString type    = "";
    if(ui->rBtnLocalWaypoint->isChecked()){
        type       = "Waypoints";
        successful = configuration.save(filename, Configuration::Type::Waypoint);
    }else if(ui->rBtnLocalPose->isChecked()){
        type       = "Robot arm positions";
        successful = configuration.save(filename, Configuration::Type::RobotArmPose);
    }
    if(successful){
        QMessageBox messageBox;
        messageBox.information(0,"Info", type + " file saved successfully.");
        messageBox.setFixedSize(500,200);
    }else{
        QMessageBox messageBox;
        messageBox.critical(0,"Error", type + " file failed to save.");
        messageBox.setFixedSize(500,200);
    }
}

void MainWindow::on_pBtnLoadFileLoad_clicked()
{
    std::string filename = ui->lineEditLocalFilename->text().toUtf8().constData();
    bool successful = false;
    QString type    = "";
    if(ui->rBtnLocalWaypoint->isChecked()){
        type = "Waypoints";
        if((successful = configuration.load(filename, Configuration::Type::Waypoint))) {
            displayWaypoints();
            displayServiceAreaProp();
        }
    }else if(ui->rBtnLocalPose->isChecked()){
        type = "Robot arm positions";
        if((successful = configuration.load(filename, Configuration::Type::RobotArmPose))) {
            displayRobotArmPoses();
        }
    }
    if(successful){
        QMessageBox messageBox;
        messageBox.information(0,"Info", type + " file loaded successfully.");
        messageBox.setFixedSize(500,200);
    }else{
        QMessageBox messageBox;
        messageBox.critical(0,"Error", type + " file failed to load.");
        messageBox.setFixedSize(500,200);
    }
}

void MainWindow::on_pBtnLocalFileCancel_clicked()
{
    ui->lineEditLocalFilename->setText("");
}

void MainWindow::on_pBtnSshFileLoad_clicked()
{
    std::string filename = ui->lineEditSshFileName->text().toUtf8().constData();
    bool successful = false;
    QString type    = "";
    if(ui->rBtnLocalWaypoint->isChecked()){
        type = "Waypoints";
        if((successful = configuration.load(filename, Configuration::Type::Waypoint))) {
            displayWaypoints();
            displayServiceAreaProp();
        }
    }else if(ui->rBtnLocalPose->isChecked()){
        type = "Robot arm positions";
        if((successful = configuration.load(filename, Configuration::Type::RobotArmPose))) {
            displayRobotArmPoses();
        }
    }
    if(successful){
        QMessageBox messageBox;
        messageBox.information(0,"Info", type + " file loaded successfully.");
        messageBox.setFixedSize(500,200);
    }else{
        QMessageBox messageBox;
        messageBox.critical(0,"Error", type + " file failed to load.");
        messageBox.setFixedSize(500,200);
    }
}

void MainWindow::on_pBtnSshFileSave_clicked()
{
    std::string filename = ui->lineEditSshFileName->text().toUtf8().constData();
    bool successful = false;
    QString type    = "";
    if(ui->rBtnLocalWaypoint->isChecked()){
        type = "Waypoints";
        successful = configuration.save(filename, Configuration::Type::Waypoint);
    }else if(ui->rBtnLocalPose->isChecked()){
        type = "Robot arm positions";
        successful = configuration.save(filename, Configuration::Type::RobotArmPose);
    }
    if(successful){
        QMessageBox messageBox;
        messageBox.information(0,"Info", type + " file loaded successfully.");
        messageBox.setFixedSize(500,200);
    }else{
        QMessageBox messageBox;
        messageBox.critical(0,"Error", type + " file failed to load.");
        messageBox.setFixedSize(500,200);
    }
}

void MainWindow::on_pBtnSshFileCancel_clicked()
{
    ui->lineEditSshFileName->setText("");
}

void MainWindow::on_pBtnAddPose_clicked()
{
        int index = ui->comboBoxPoseName->currentIndex();

        std::string description = ui->lineEditPoseName->text().toUtf8().constData();

        robot_arm::JointPose pose(0, description);

        if (ui->rBtnAutoPose->isChecked()){

            MessageKeeper.lock(&mutex);
            robot_arm::JointPose joint_pose = MessageKeeper.robotArmPose();

            pose.joint_1 = joint_pose.joint_1;
            pose.joint_2 = joint_pose.joint_2;
            pose.joint_3 = joint_pose.joint_3;
            pose.joint_4 = joint_pose.joint_4;
            pose.joint_5 = joint_pose.joint_5;
            pose.joint_6 = joint_pose.joint_6;
        }else if (ui->rBtnManualPose->isChecked()){
            pose.joint_1 = ui->lineEditPoseJ1->text().toDouble();
            pose.joint_2 = ui->lineEditPoseJ2->text().toDouble();
            pose.joint_3 = ui->lineEditPoseJ3->text().toDouble();
            pose.joint_4 = ui->lineEditPoseJ4->text().toDouble();
            pose.joint_5 = ui->lineEditPoseJ5->text().toDouble();
            pose.joint_6 = ui->lineEditPoseJ6->text().toDouble();
        }


        configuration.addRobotArmPose(pose);

        displayRobotArmPoses();
}

void MainWindow::on_pBtnUpdatePose_clicked()
{
    QModelIndexList selection = ui->tableWidgetRobPose->selectionModel()->selectedRows();

    if (selection.count() <= 0){
        QMessageBox messageBox;
        messageBox.critical(0, "Error", "A robot arm position must be seleceted from the list!");
        messageBox.setFixedSize(500,200);
        return;
    }

    for(int i = 0; i < selection.count(); i++) {
        QModelIndex index = selection.at(i);

        robot_arm::JointPose *pose = configuration.getRobotArmPose(index.data().toInt());

        pose->description = ui->lineEditPoseName->text().toUtf8().constData();

        if (ui->rBtnFeedback->isChecked()){
            MessageKeeper.lock(&mutex);
            robot_arm::JointPose joint_pose = MessageKeeper.robotArmPose();

            pose->joint_1 = joint_pose.joint_1;
            pose->joint_2 = joint_pose.joint_2;
            pose->joint_3 = joint_pose.joint_3;
            pose->joint_4 = joint_pose.joint_4;
            pose->joint_5 = joint_pose.joint_5;
            pose->joint_6 = joint_pose.joint_6;

        }else{
            pose->joint_1 = ui->lineEditPoseJ1->text().toDouble();
            pose->joint_2 = ui->lineEditPoseJ2->text().toDouble();
            pose->joint_3 = ui->lineEditPoseJ3->text().toDouble();
            pose->joint_4 = ui->lineEditPoseJ4->text().toDouble();
            pose->joint_5 = ui->lineEditPoseJ5->text().toDouble();
            pose->joint_6 = ui->lineEditPoseJ6->text().toDouble();
        }
//        configuration.updateWaypoint(i, waypoint);



    }
    ui->tableWidgetRobPose->clearSelection();
    displayRobotArmPoses();
}

void MainWindow::on_pBtnClearPose_clicked()
{
    ui->lineEditPoseName->setText("");
    ui->lineEditPoseJ1->setText("");
    ui->lineEditPoseJ2->setText("");
    ui->lineEditPoseJ3->setText("");
    ui->lineEditPoseJ4->setText("");
    ui->lineEditPoseJ5->setText("");
    ui->lineEditPoseJ6->setText("");
}

void MainWindow::on_pBtnRemovePose_clicked()
{
    QModelIndexList selection = ui->tableWidgetRobPose->selectionModel()->selectedRows();

    if (selection.count() <= 0){
        QMessageBox messageBox;
        messageBox.critical(0, "Error", "Robot arm postion must be selected from the waypoint list!");
        messageBox.setFixedSize(500,200);
        return;
    }

    for(int i = 0; i < selection.count(); i++) {
        QModelIndex index = selection.at(i);
        configuration.deleteRobotArmPose(index.data().toInt());
    }

    displayRobotArmPoses();
}

void MainWindow::on_pBtnOkWs_clicked()
{
    QModelIndexList selection = ui->tableWidgetWorkServiceAreaProp->selectionModel()->selectedRows();

    if (selection.count() <= 0){
        QMessageBox messageBox;
        messageBox.critical(0, "Error", "A Service area must be selected from the list!");
        messageBox.setFixedSize(500,200);
        return;
    }

    for(int i = 0; i < selection.count(); i++) {
        QModelIndex index = selection.at(i);

        Waypoint *waypoint = configuration.getWaypoint(index.data().toInt());

        waypoint->service_area.length       = ui->lineEditWorkSurfLength->text().toDouble();
        waypoint->service_area.width        = ui->lineEditWorkSurfWidth->text().toDouble();
        waypoint->service_area.height       = ui->lineEditWorkSurfHeight->text().toDouble();
        waypoint->service_area.diameter     = ui->lineEditWorkSurfDiameter->text().toDouble();
        waypoint->service_area.tilt_angle   = ui->lineEditWorkSurfAngle->text().toDouble();
    }
    ui->tableWidgetWorkServiceAreaProp->clearSelection();
    displayServiceAreaProp();
}

void MainWindow::on_pBtnClearWs_clicked()
{
    ui->lineEditWorkSurfLength->setText("");
    ui->lineEditWorkSurfWidth->setText("");
    ui->lineEditWorkSurfHeight->setText("");
    ui->lineEditWorkSurfDiameter->setText("");
    ui->lineEditWorkSurfAngle->setText("");
}


void MainWindow::on_pBtnAddOrder_clicked()
{

}

void MainWindow::on_pBtnRemoveOrder_2_clicked()
{

}

void MainWindow::on_pBtnRemoveOrder_clicked()
{
    QModelIndexList selection = ui->tableWidgetOrders->selectionModel()->selectedRows();

    if (selection.count() <= 0){
        QMessageBox messageBox;
        messageBox.critical(0, "Error", "A task must be selected from the To Do List!");
        messageBox.setFixedSize(500,200);
        return;
    }

    for(int i = 0; i < selection.count(); i++) {
        QModelIndex index = selection.at(i);
//        configuration.deleteRobotArmPose(index.data().toInt());
    }

//    displayOrders();
}

void MainWindow::on_pBtnStartRotatingTable_clicked()
{

}

void MainWindow::on_pBtnResetMission_clicked()
{

}

void MainWindow::on_pBtnStartMission_clicked()
{

}

void MainWindow::on_pBtnAddObject_clicked()
{

}

void MainWindow::on_pBtnUpdateObject_clicked()
{

}

void MainWindow::on_pBtnClearObject_clicked()
{

}

void MainWindow::on_tableWidgetGoal_activated(const QModelIndex &index)
{
    QString str = QString::number(index.row());
     QMessageBox::information(this, tr("Selected row"), str,
        QMessageBox::Ok | QMessageBox::Cancel,
        QMessageBox::Cancel);
}

void MainWindow::on_button_connect_clicked(bool state)
{
    if ( ui->checkbox_use_environment->isChecked() ) {
            if ( !rnode.init() ) {
                QMessageBox msgBox;
                msgBox.setText("Couldn't find the ros master.");
                msgBox.exec();
                close();
            } else {
                    ui->button_connect->setEnabled(false);
            }
    } else {
            if ( ! rnode.init(ui->line_edit_master->text().toStdString(),
                               ui->line_edit_host->text().toStdString()) ) {
                QMessageBox msgBox;
                msgBox.setText("Couldn't find the ros master.");
                msgBox.exec();
                close();
            } else {
                    ui->button_connect->setEnabled(false);
                    ui->line_edit_master->setReadOnly(true);
                    ui->line_edit_host->setReadOnly(true);
                    ui->line_edit_topic->setReadOnly(true);
            }
    }
}

void MainWindow::on_button_connect_clicked()
{
    on_button_connect_clicked(true);
}


void MainWindow::on_pBtnClearRobPoseList_clicked()
{
    configuration.resetRobotArmPoses();

    ui->tableWidgetRobPose->clearContents();
    ui->tableWidgetRobPose->model()->removeRows(0, ui->tableWidgetRobPose->rowCount());

     displayRobotArmPoses();
}

void MainWindow::on_pBtnResetList_clicked()
{
    configuration.resetWaypoints();

    ui->tableWidgetGoal->clearContents();
    ui->tableWidgetGoal->model()->removeRows(0, ui->tableWidgetGoal->rowCount());

    ui->tableWidgetWorkServiceAreaProp->clearContents();
    ui->tableWidgetWorkServiceAreaProp->model()->removeRows(0, ui->tableWidgetWorkServiceAreaProp->rowCount());
}
