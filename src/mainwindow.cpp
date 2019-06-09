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


int counter = 0;
MainWindow *MainWindowPtr;


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


void MainWindowWrapperCallback(std::string data){
    MainWindowPtr->updateMarker(data);
//    std::cout << data <<std::endl;
}

void MainWindowWrapperFeedbackCallback(robomis::Waypoint data){
    MainWindowPtr->updateFeedback(data);
    std::cout << "Feedback received" <<std::endl;
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
    palette(new QPalette()),
    timer(new QTimer(this))
{
    MainWindowPtr = this;

    ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
//    QObject::connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

//    ui.setupUi(this);
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

    QObject::connect(&rnode, SIGNAL(rosShutdown()), this, SLOT(close()));
//    connect(ui->tabWidgetProperties, SIGNAL(currentChanged(int)), this, SLOT(tabSelected()))



    rnode.init();
    rnode.attachMarkerCallback(MainWindowWrapperCallback);
    rnode.attachFeedbackCallback(MainWindowWrapperFeedbackCallback);

//    qRegisterMetaTypeStreamOperators<robomis::WaypointData>("WaypointData");

//    QTimer *timer1 = new QTimer(this);
//    connect(timer1, SIGNAL(timeout()), this, SLOT(updateRobotPositionDisplay()));
//    timer->start(1000);

    connect(timer, SIGNAL(timeout()), this, SLOT(updateRobotPositionDisplay()));
//    QObject::connect(&rnode, SIGNAL(rnode.newPosereceived()), this, SLOT(updateRobotPositionDisplay()));


}

MainWindow::~MainWindow() {
    delete ui;
    delete intValidator;
    delete doubleValivator;
}

void MainWindow::updateMarker(std::string data) {
    int id = extractIntergerFromString(data);
//    ui->lineEditWaypointMarker->setText(QString::fromStdString (data));
//    ui->lineEditWaypointMarkerId->setText(QString::number(id));
//    ui->lineEditExpWaypointID->setText(QString::number(id));
//    std::cout << data <<std::endl;
}

void MainWindow::updateRobotPositionDisplay() {
    if(shouldDisplayfeedback) {
        geometry_msgs::Pose pose = rnode.getRobotPosition();
        current_pose = rnode.getRobotPosition();
        ui->lineEditPosX->setText(QString("%1").arg(pose.position.x, 0, 'f', 3));
        ui->lineEditPosY->setText(QString("%1").arg(pose.position.y, 0, 'f', 3));
        ui->lineEditPosZ->setText(QString("%1").arg(pose.position.z, 0, 'f', 3));
        ui->lineEditOrientX->setText(QString("%1").arg(pose.orientation.x, 0, 'f', 3));
        ui->lineEditOrientY->setText(QString("%1").arg(pose.orientation.y, 0, 'f', 3));
        ui->lineEditOrientZ->setText(QString("%1").arg(pose.orientation.z, 0, 'f', 3));
        ui->lineEditOrientW->setText(QString("%1").arg(pose.orientation.w, 0, 'f', 3));
//        ui->lineEditPosX->setText(QString::number(pose.position.x));
//        ui->lineEditPosY->setText(QString::number(pose.position.y));
//        ui->lineEditPosZ->setText(QString::number(pose.position.z));
//        ui->lineEditOrientX->setText(QString::number(pose.orientation.x));
//        ui->lineEditOrientY->setText(QString::number(pose.orientation.y));
//        ui->lineEditOrientZ->setText(QString::number(pose.orientation.z));
//        ui->lineEditOrientW->setText(QString::number(pose.orientation.w));
    }
//    counter++;
//    ui->lineEditName->setText(QString::number(counter));
//    std::cout << data <<std::endl;
}

void MainWindow::updateFeedback(robomis::Waypoint data) {

    if(shouldDisplayfeedback) {
        ui->lineEditPosX->setText(QString("%1").arg(data.pose.position.x, 0, 'f', 3));
        ui->lineEditPosY->setText(QString("%1").arg(data.pose.position.y, 0, 'f', 3));
        ui->lineEditPosZ->setText(QString("%1").arg(data.pose.position.z, 0, 'f', 3));
        ui->lineEditOrientX->setText(QString("%1").arg(data.pose.orientation.x, 0, 'f', 3));
        ui->lineEditOrientY->setText(QString("%1").arg(data.pose.orientation.y, 0, 'f', 3));
        ui->lineEditOrientZ->setText(QString("%1").arg(data.pose.orientation.z, 0, 'f', 3));
        ui->lineEditOrientW->setText(QString("%1").arg(data.pose.orientation.w, 0, 'f', 3));
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
                        current_pose.position.x,
                        current_pose.position.y,
                        current_pose.position.z
                        ),
                    geometry::Pose::Quaternion(
                        current_pose.orientation.x,
                        current_pose.orientation.y,
                        current_pose.orientation.z,
                        current_pose.orientation.w
                       )
                    );
    }else if (ui->rBtnFeedback->isChecked()){
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
    }


    Waypoint waypoint(location, pose);
    waypoint.should_scan = scan;

    configuration.addWaypoint(waypoint);
//    ui->lineEditInstanceId->setText(QString::number(WaypointHelper::getNextInstanceID(configuration.getNrOfWaypointsWithType(loca_type))));

    /* Display in the Table Widget */
    displayWaypoints();
    displayServiceAreaProp();
}

void MainWindow::displayWaypoints() {
    /* Clear the table widget */
    ui->tableWidgetGoal->clearContents();
    ui->tableWidgetGoal->model()->removeRows(0, ui->tableWidgetGoal->rowCount());

    ui->tableWidgetObjectSource->clearContents();
    ui->tableWidgetObjectSource->model()->removeRows(0, ui->tableWidgetGoal->rowCount());

    ui->tableWidgetObjectDestination->clearContents();
    ui->tableWidgetObjectDestination->model()->removeRows(0, ui->tableWidgetGoal->rowCount());

    std::vector<Waypoint> waypoints = configuration.getWaypoints();
    int sub_index = 0;
    for(size_t row = 0; row < waypoints.size(); row++) {

        Waypoint waypoint = waypoints[row];
        /* add a new row */
        ui->tableWidgetGoal->insertRow(static_cast<int>(row));
        /* insert to columns of the new row*/
        int row_index = ui->tableWidgetGoal->rowCount()-1;
        QString scan = waypoint.should_scan == true ? "Enabled" : "Disabled";

        ui->tableWidgetGoal->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(waypoint.location.description)));
        ui->tableWidgetGoal->setItem(  row_index,  1,  new QTableWidgetItem(scan));
        ui->tableWidgetGoal->setItem(  row_index,  2,  new QTableWidgetItem(QString::number(waypoint.pose.position.x)));
        ui->tableWidgetGoal->setItem(  row_index,  3,  new QTableWidgetItem(QString::number(waypoint.pose.position.y)));
        ui->tableWidgetGoal->setItem(  row_index,  4,  new QTableWidgetItem(QString::number(waypoint.pose.position.z)));
        ui->tableWidgetGoal->setItem(  row_index,  5,  new QTableWidgetItem(QString::number(waypoint.pose.orientation.x)));
        ui->tableWidgetGoal->setItem(  row_index,  6,  new QTableWidgetItem(QString::number(waypoint.pose.orientation.y)));
        ui->tableWidgetGoal->setItem(  row_index,  7,  new QTableWidgetItem(QString::number(waypoint.pose.orientation.z)));
        ui->tableWidgetGoal->setItem(  row_index,  8,  new QTableWidgetItem(QString::number(waypoint.pose.orientation.w)));

        ui->tableWidgetObjectSource->insertRow(static_cast<int>(row));
        row_index = ui->tableWidgetObjectSource->rowCount()-1;
        ui->tableWidgetObjectSource->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(waypoint.location.description)));

        ui->tableWidgetObjectDestination->insertRow(static_cast<int>(row));
        row_index = ui->tableWidgetObjectDestination->rowCount()-1;
        ui->tableWidgetObjectDestination->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(waypoint.location.description)));


    }

}

void MainWindow::displayServiceAreaProp() {
    /* Clear the table widget */

    ui->tableWidgetWorkServiceAreaProp->clearContents();
    ui->tableWidgetWorkServiceAreaProp->model()->removeRows(0, ui->tableWidgetWorkServiceAreaProp->rowCount());

    std::vector<Waypoint> waypoints = configuration.getWaypoints();
    int table_row = 0;

    for(size_t i = 0; i < waypoints.size(); i++) {

        Waypoint waypoint = waypoints[i];

        if(WaypointHelper::isServiceArea(WaypointHelper::getLocationTypeByIndex(waypoint.location.type))){
            /* add a new row */
            ui->tableWidgetWorkServiceAreaProp->insertRow(static_cast<int>(table_row));
            /* insert to columns of the new row*/
            int row_index = ui->tableWidgetWorkServiceAreaProp->rowCount()-1;

            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(waypoint.location.description)));
            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  1,  new QTableWidgetItem(QString::number(waypoint.service_area.length)));
            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  2,  new QTableWidgetItem(QString::number(waypoint.service_area.width)));
            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  3,  new QTableWidgetItem(QString::number(waypoint.service_area.height)));
            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  4,  new QTableWidgetItem(QString::number(waypoint.service_area.diameter)));
            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  5,  new QTableWidgetItem(QString::number(waypoint.service_area.tilt_angle)));
            table_row++;
        }


    }


}

void MainWindow::displayRobotArmPoses() {
    ui->tableWidgetRobPose->clearContents();
    ui->tableWidgetRobPose->model()->removeRows(0, ui->tableWidgetGoal->rowCount());

    std::vector<robot_arm::JointPose> poses = configuration.getRobotArmPoses();
    int sub_index = 0;
    for(size_t row = 0; row < poses.size(); row++) {

       robot_arm::JointPose pose = poses[row];
        /* add a new row */
        ui->tableWidgetRobPose->insertRow(static_cast<int>(row));
        /* insert to columns of the new row*/
        int row_index = ui->tableWidgetRobPose->rowCount()-1;

        ui->tableWidgetRobPose->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(pose.description)));
        ui->tableWidgetRobPose->setItem(  row_index,  1,  new QTableWidgetItem(QString::number(pose.joint_1)));
        ui->tableWidgetRobPose->setItem(  row_index,  2,  new QTableWidgetItem(QString::number(pose.joint_2)));
        ui->tableWidgetRobPose->setItem(  row_index,  3,  new QTableWidgetItem(QString::number(pose.joint_3)));
        ui->tableWidgetRobPose->setItem(  row_index,  4,  new QTableWidgetItem(QString::number(pose.joint_4)));
        ui->tableWidgetRobPose->setItem(  row_index,  5,  new QTableWidgetItem(QString::number(pose.joint_5)));
        ui->tableWidgetRobPose->setItem(  row_index,  6,  new QTableWidgetItem(QString::number(pose.joint_6)));

    }
}

void MainWindow::displayPlannerObjects() {
  // from configurations
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
    displayRobotArmPoses();
}

void MainWindow::on_rBtnManual_clicked()
{
    updateLineEdit(false, Qt::white, Qt::black);
    shouldDisplayfeedback = false;
    timer->stop();
}

void MainWindow::on_rBtnFeedback_clicked()
{
    updateLineEdit(true, Qt::gray, Qt::black);
    shouldDisplayfeedback = true;
     timer->start(100);
}

void MainWindow::on_pBtnPublish_clicked()
{
    if(configuration.getWaypoints().size() > 0){
//        rnode.publishWaypoints();
        QMessageBox messageBox;
        messageBox.information(0,"Info","Message published succesfully.");
        messageBox.setFixedSize(500,200);
    }else {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","No waypoints entered. You must enter at least one waypoint.");
        messageBox.setFixedSize(500,200);
    }
}

void MainWindow::on_pBtnReset_clicked()
{
    configuration.resetWaypoints();

    ui->tableWidgetGoal->clearContents();
    ui->tableWidgetGoal->model()->removeRows(0, ui->tableWidgetGoal->rowCount());

    ui->tableWidgetWorkServiceAreaProp->clearContents();
    ui->tableWidgetWorkServiceAreaProp->model()->removeRows(0, ui->tableWidgetWorkServiceAreaProp->rowCount());
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
    for(int i = 0; i < selection.count(); i++) {
        QModelIndex index = selection.at(i);

        int instance_id   = ui->lineEditInstanceId->text().toInt();
        std::string name  = ui->comboBoxName->currentText().toUtf8().constData();
        double pos_x      = ui->lineEditPosX->text().toDouble();
        double pos_y      = ui->lineEditPosY->text().toDouble();
        double pos_z      = ui->lineEditPosZ->text().toDouble();
        double orient_x   = ui->lineEditOrientX->text().toDouble();
        double orient_y   = ui->lineEditOrientY->text().toDouble();
        double orient_z   = ui->lineEditOrientZ->text().toDouble();
        double orient_w   = ui->lineEditOrientW->text().toDouble();
        bool scan         = ui->checkBoxMarkerScan->isChecked() ? true : false;


        Waypoint waypoint = configuration.getWaypointAtIndex(i);
        mission_protobuf::LocationIdentifier::LocationType loca_type = WaypointHelper::getLocationTypeByIndex(ui->comboBoxName->currentIndex());

        waypoint.location.type         = WaypointHelper::getLocationTypeNumeral(loca_type);
        waypoint.location.instance_id  = instance_id;
        waypoint.location.description  = WaypointHelper::getLocationStrByType(loca_type);
        waypoint.should_scan           = scan;

        if (shouldDisplayfeedback){
            waypoint.pose.position.x       = current_pose.position.x;
            waypoint.pose.position.x       = current_pose.position.y;
            waypoint.pose.position.x       = current_pose.position.z;
            waypoint.pose.orientation.x    = current_pose.orientation.x;
            waypoint.pose.orientation.y    = current_pose.orientation.y;
            waypoint.pose.orientation.z    = current_pose.orientation.z;
            waypoint.pose.orientation.w    = current_pose.orientation.w;

        }else{
            waypoint.pose.position.x       = pos_x;
            waypoint.pose.position.x       = pos_y;
            waypoint.pose.position.x       = pos_z;
            waypoint.pose.orientation.x    = orient_x;
            waypoint.pose.orientation.y    = orient_y;
            waypoint.pose.orientation.z    = orient_z;
            waypoint.pose.orientation.w    = orient_w;
        }
        configuration.updateWaypoint(i, waypoint);



    }
    ui->tableWidgetGoal->clearSelection();
    displayWaypoints();
}

//void Player::open()
//{
//    QFileDialog fileDialog(this);
//    fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
//    fileDialog.setWindowTitle(tr("Open Files"));
//    QStringList supportedMimeTypes = m_player->supportedMimeTypes();
//    if (!supportedMimeTypes.isEmpty()) {
//        supportedMimeTypes.append("audio/x-m3u"); // MP3 playlists
//        fileDialog.setMimeTypeFilters(supportedMimeTypes);
//    }
//    fileDialog.setDirectory(QStandardPaths::standardLocations(QStandardPaths::MoviesLocation).value(0, QDir::homePath()));
//    if (fileDialog.exec() == QDialog::Accepted)
//        addToPlaylist(fileDialog.selectedUrls());
//}

void MainWindow::showWaypointsProperties(void) {
    std::vector<Waypoint> waypoints = configuration.getWaypoints();

    if(waypoints.size() <= 0)
        return;

    for(size_t row = 0; row < waypoints.size(); row++){

//        Waypoint *waypoint = &(waypoints[row]);

//        if (WaypointHelper::isServiceArea(waypoint.location().type())){
//            std::cout<<"waypoints"<<std::endl;
//            ui->tableWidgetWorkServiceAreaProp->insertRow(static_cast<int>(row));
//            /* insert to columns of the new row*/
//            int row_index = ui->tableWidgetGoal->rowCount()-1;
////            QString scan = waypoint.scan == true ? "Enabled" : "Disabled";
//            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  0,  new QTableWidgetItem(QString::fromStdString(waypoint.service_area.description)));
//            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  2,  new QTableWidgetItem(QString::number(waypoint.service_area.length())));
//            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  2,  new QTableWidgetItem(QString::number(waypoint.service_area.width())));
//            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  2,  new QTableWidgetItem(QString::number(waypoint.service_area.height())));
//            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  2,  new QTableWidgetItem(QString::number(waypoint.service_area.diameter())));
//            ui->tableWidgetWorkServiceAreaProp->setItem(  row_index,  2,  new QTableWidgetItem(QString::number(waypoint.service_area.tilt_angle())));
//        }

    }

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
    if(ui->rBtnLocalWaypoint->isChecked()){
        if (configuration.saveWaypoints(filename)) {
            displayWaypoints();
            displayServiceAreaProp();
            QMessageBox messageBox;
            messageBox.information(0,"Info", "Waypoints file saved successfully.");
            messageBox.setFixedSize(500,200);
            return;
        }
    }else if(ui->rBtnLocalPose->isChecked()){
        if(configuration.saveRobotArmPoses(filename)) {
            QMessageBox messageBox;
            messageBox.information(0,"Info", "Robot arm poses file saved successfully.");
            messageBox.setFixedSize(500,200);
            return;
        }
    }
}

void MainWindow::on_pBtnLoadFileLoad_clicked()
{
    std::string filename = ui->lineEditLocalFilename->text().toUtf8().constData();
    if(ui->rBtnLocalWaypoint->isChecked()){
        if(configuration.loadWaypoints(filename)) {
            QMessageBox messageBox;
            messageBox.information(0,"Info", "Waypoints file loaded successfully.");
            messageBox.setFixedSize(500,200);
            return;
        }
    }else if(ui->rBtnLocalPose->isChecked()){
        if(configuration.loadRobotArmPoses(filename)) {
            QMessageBox messageBox;
            messageBox.information(0,"Info", "Robot arm poses file loaded successfully.");
            messageBox.setFixedSize(500,200);
            return;
        }
    }
}

void MainWindow::on_pBtnLocalFileCancel_clicked()
{
    ui->lineEditLocalFilename->setText("");
}

void MainWindow::on_pBtnSshFileLoad_clicked()
{
    std::string filename = ui->lineEditSshFileName->text().toUtf8().constData();
    if(ui->rBtnLocalWaypoint->isChecked()){
        if(configuration.loadWaypoints(filename)) {
            displayWaypoints();
            displayServiceAreaProp();
            QMessageBox messageBox;
            messageBox.information(0,"Info", "Waypoints file loaded successfully.");
            messageBox.setFixedSize(500,200);
            return;
        }
    }else if(ui->rBtnLocalPose->isChecked()){
        if(configuration.loadRobotArmPoses(filename)) {
            QMessageBox messageBox;
            messageBox.information(0,"Info", "Robot arm poses file saved successfully.");
            messageBox.setFixedSize(500,200);
            return;
        }
    }
}

void MainWindow::on_pBtnSshFileSave_clicked()
{
    std::string filename = ui->lineEditSshFileName->text().toUtf8().constData();
    if(ui->rBtnLocalWaypoint->isChecked()){
        if(configuration.saveWaypoints(filename)) {
            QMessageBox messageBox;
            messageBox.information(0,"Info", "Waypoints file saved successfully.");
            messageBox.setFixedSize(500,200);
            return;
        }
    }else if(ui->rBtnLocalPose->isChecked()){
        if(configuration.saveRobotArmPoses(filename)) {
            QMessageBox messageBox;
           messageBox.information(0,"Info", "Robot arm poses file saved successfully.");
            messageBox.setFixedSize(500,200);
            return;
        }
    }
}

void MainWindow::on_pBtnSshFileCancel_clicked()
{
    ui->lineEditSshFileName->setText("");
}

void MainWindow::on_pBtnAddPose_clicked()
{
    //    ui->lineEditInstanceId->setText(QString::number(configuration.getNrOfWaypointsWithType()));

        int index = ui->comboBoxPoseName->currentIndex();


//        ui->lineEditPoseNr->setText(QString::number(WaypointHelper::getNextInstanceID(configuration.getNrOfWaypointsWithType(loca_type))));


        int id   = ui->lineEditPoseNr->text().toInt();
        std::string description = ui->lineEditPoseName->text().toUtf8().constData();

        robot_arm::JointPose pose(id, description);

        if (ui->rBtnAutoPose->isChecked()){
//            pose.joint_1 = current_joint_pose.joint_1;
//            pose.joint_2 = current_joint_pose.joint_2;
//            pose.joint_3 = current_joint_pose.joint_3;
//            pose.joint_4 = current_joint_pose.joint_4;
//            pose.joint_5 = current_joint_pose.joint_5;
//            pose.joint_6 = current_joint_pose.joint_6;

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
    displayRobotArmPoses();
}

void MainWindow::on_pBtnClearPose_clicked()
{

}

void MainWindow::on_pBtnRemovePose_clicked()
{
    displayRobotArmPoses();
}

void MainWindow::on_pBtnOkWs_clicked()
{

}

void MainWindow::on_pBtnClearWs_clicked()
{

}

void MainWindow::on_pBtnClearListWs_clicked()
{

}

void MainWindow::on_pBtnAddOrder_clicked()
{

}

void MainWindow::on_pBtnRemoveOrder_2_clicked()
{

}

void MainWindow::on_pBtnRemoveOrder_clicked()
{

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
