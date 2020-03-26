#include "ros/ros.h"
#include <QtCore>
#include <QWidget>
#include <QApplication>
#include <QIcon>
#include <QFrame>
#include <QGridLayout>
#include <iostream>
#include "messages/Ball.h"

void adjustBall(const messages::Ball::ConstPtr& msg) {
    ROS_INFO("I heard: [Position.x: %f] [Position.y: %f]", msg->position.x, msg->position.y);
}

int main(int argc, char** argv) {
    // init ROS visual table node
    ros::init(argc, argv, "virtual_table");
    ros::NodeHandle nh;
    ros::Subscriber ball_position_sub = nh.subscribe("/simulator/ball", 1000, adjustBall);

    std::cout << "Qt version: " << qVersion() << std::endl;

    QApplication app(argc, argv);
    QWidget window;

    window.resize(500, 300);
    window.move(300, 300);
    window.setWindowTitle("Foosball Visualizer");
    window.setToolTip("Foosball Table");
    window.setWindowIcon(QIcon("../resources/soccer_ball.png"));
    window.show();

    ros::spin();

    return app.exec();
}
