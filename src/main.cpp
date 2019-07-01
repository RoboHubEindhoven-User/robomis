#include <QApplication>
#include "../include/robomis/mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow form(argc, argv);

    form.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    form.startNode();

    int result = app.exec();

    return result;
}
