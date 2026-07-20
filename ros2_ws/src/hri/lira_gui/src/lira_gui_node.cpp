#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    std::cout << "INITIALIZING LIRA GUI NODE ..." << std::endl;
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}

