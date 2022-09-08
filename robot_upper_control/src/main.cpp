#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

//    class1_robot_upper_control::MainWindow w(argc, argv);
    MainWindow w(argc, argv);
    w.show();
//    w.showMaximized();

    return a.exec();
}
