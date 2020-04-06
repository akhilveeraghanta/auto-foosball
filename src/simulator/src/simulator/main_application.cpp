#include "ros/ros.h"
#include <QLabel>
#include <QtCore>
#include <QWidget>
#include <QPainter>
#include <QTime>
#include <QHBoxLayout>
#include <QApplication>
#include <QIcon>
#include <QFrame>
#include <QGridLayout>
#include <iostream>
#include "messages/Ball.h"
#include "messages/Stick.h"
#include <boost/thread/thread.hpp>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QDialog>
#include <QColor>
#include <QObject>
#include <QTransform>
#include <QMainWindow>
// TODO fix relative imports
#include "virtual_table.h"
#include "virtual_table_communicator.h"

namespace {
    VirtualTable* vt;
}

void updateBall_fixme(const messages::Ball::ConstPtr& msg){
    vt->get_communicator().set_ball(msg);
}

/**********************************************************************
 *                                Main                                *
 **********************************************************************/

int main(int argc, char** argv) {
    // init ROS visual table node
    ros::init(argc, argv, "virtual_table");
    ros::NodeHandle nh;

    QApplication app(argc, argv);

    QBrush greenBrush(Qt::green);
    QPen outlinePen(Qt::black);
    outlinePen.setWidth(2);

    //scene.addEllipse(400, 400, 15, 15, outlinePen, greenBrush);
    //scene.addEllipse(110, 100, 40, 40, outlinePen, greenBrush);
    vt = new VirtualTable(0,500,500);
    vt->resize(500,500);
    vt->setWindowTitle("Foosball Visualizer");
    vt->show();

    ros::Subscriber ball_position_sub =
        nh.subscribe("/simulator/ball", 1, updateBall_fixme);

    boost::thread thread_spin(boost::bind(ros::spin));

    return app.exec();
}
