#include "messages/Ball.h"
#include "messages/Stick.h"
#include "ros/ros.h"
#include "virtual_table.h"
#include "virtual_table_communicator.h"
#include <QApplication>
#include <QColor>
#include <QDialog>
#include <QFrame>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QMainWindow>
#include <QObject>
#include <QPainter>
#include <QTime>
#include <QTransform>
#include <QWidget>
#include <QtCore>
#include <boost/thread/thread.hpp>
#include <iostream>

namespace {
VirtualTable *vt;
}

void updateBall_fixme(const messages::Ball::ConstPtr &msg) {
  vt->get_communicator().set_ball(msg);
}

/**********************************************************************
 *                                Main                                *
 **********************************************************************/

int main(int argc, char **argv) {
  // init ROS visual table node
  ros::init(argc, argv, "virtual_table");
  ros::NodeHandle nh;

  QApplication app(argc, argv);

  vt = new VirtualTable(0, 500, 500);
  vt->resize(500, 500);
  vt->setWindowTitle("Foosball Visualizer");
  vt->show();

  ros::Subscriber ball_position_sub =
      nh.subscribe("/simulator/ball", 1, updateBall_fixme);

  boost::thread thread_spin(boost::bind(ros::spin));

  return app.exec();
}
