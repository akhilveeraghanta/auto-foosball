#include "ros/ros.h"
#include <QtCore>
#include <QWidget>
#include <QApplication>
#include <iostream>

int main(int argc, char** argv)
{
    // init ROS visual table node
    ros::init(argc, argv, "visual_table");

    std::cout << "Qt version: " << qVersion() << std::endl;

    QApplication app(argc, argv);
    QWidget window;

    window.resize(500, 300);
    window.setWindowTitle("Foosball Visualizer");
    window.show();

    return app.exec();
}
