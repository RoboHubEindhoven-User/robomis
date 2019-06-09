//#include "../include/robomis/waypointconfiguration.h"
//#include <fstream>
//#ifdef WINDOWS
//#include <direct.h>
//#define GetCurrentDir _getcwd
//#else
//#include <unistd.h>
//#define GetCurrentDir getcwd
//#endif
//#include <QWidget>
//#include <QString>
//#include <QFile>
//#include <QMessageBox>
////#include <QIODevice>
////#include <QDataStream>
//#include <QFileDialog>
//#include <yaml-cpp/yaml.h>
////#include <QTranslator>
////#include <QObject>
////#include "yaml.h"

//namespace robomis {

//std::string getPathName(const std::string& filename) {

//   char sep = '/';

//#ifdef _WIN32
//   sep = '\\';
//#endif

//   size_t pos = filename.rfind(sep, filename.length());

//   if (pos != std::string::npos) {
//      return(filename.substr(0, pos));
//   }

//   return("");
//}

//WaypointConfiguration *WaypointConfiguration::Instance() {
//    static WaypointConfiguration instance;
//    return &instance;
//}

//void WaypointConfiguration::saveToFile(WaypointData waypoint_data) {

//    QString fileName = QFileDialog::getSaveFileName(this,
//           tr("Save Waypoint Data"), "",
//           tr("Waypoint Data (*.yaml);;All Files (*)"));

//    if (fileName.isEmpty())
//        return;
//    else {
//        QFile file(fileName);
//        if (!file.open(QIODevice::WriteOnly)) {
//            QMessageBox::information(this, tr("Unable to open file"),
//                file.errorString());
//            return;
//        }
//        QDataStream out_file(&file);
//        out_file.setVersion(QDataStream::Qt_4_5);
//        out_file << waypoint_data;
//    }
//}

////    std::string file_str(file);
////    std::string filename = getPathName(file_str);
////    config = YAML::LoadFile(filename);

////    config["waypoint_data"] = data;

////    std::ofstream fout(filename);
////    fout << config;

////    // close the opened file.
////    fout.close();
////}

//void WaypointConfiguration::loadFromFile(WaypointData *waypoint_data) {

//    QString fileName = QFileDialog::getOpenFileName(this,
//            tr("Open Address Book"), "",
//            tr("Address Book (*.abk);;All Files (*)"));

//    if (fileName.isEmpty())
//        return;
//    else {

//        QFile file(fileName);

//        if (!file.open(QIODevice::ReadOnly)) {
//            QMessageBox::information(this, tr("Unable to open file"),
//                file.errorString());
//            return;
//        }

//        QDataStream in(&file);
//        in.setVersion(QDataStream::Qt_4_5);
//        waypoint_data->frame = "";
//        waypoint_data->waypoints->clear();   // clear existing contacts
//        in >> waypoint_data;
//        if (waypoint_data->waypoints.isEmpty()) {
//            QMessageBox::information(this, tr("No waypoints in file"),
//                tr("The file you are attempting to open contains no waypoints."));
//        }
////        else {
////            QMap<QString, QString>::iterator i = contacts.begin();
////            nameLine->setText(i.key());
////            addressText->setText(i.value());
////        }
//    }

////    updateInterface(NavigationMode);
//}

////    std::string file_str(file);
////    std::string filename = getPathName(file_str);
////    config = YAML::LoadFile(filename);

////    WaypointData tmp_data;

////    if (config["waypoint_data"]) {
////      tmp_data = config["waypoint_data"].as<WaypointData>();
////    }
////    data->frame = tmp_data.frame;
////    data->waypoints = tmp_data->waypoints;

////    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
////         const YAML::Node& sensor = *it;
////      std::cout << it->as<WaypointData>() << "\n";
////    }

//} /* namespace robomis */
