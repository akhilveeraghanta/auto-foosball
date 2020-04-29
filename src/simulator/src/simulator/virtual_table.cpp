#include "virtual_table.h"
#include "messages/Ball.h"
#include "messages/Stick.h"
#include "ros/ros.h"
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
/**********************************************************************
 *                    Virtual Table Implementation                    *
 **********************************************************************/

VirtualTable::VirtualTable(QWidget *parent, int width_px, int height_px)
    : QMainWindow(parent) {
  connect(&communicator, &VirtualTableCommunicator::refresh_gui, this,
          &VirtualTable::refresh_gui);

  setStyleSheet("background-color:white;");
  scene.setSceneRect(0, 0, 500, 500);
  main_widget = new QWidget();
  view = new QGraphicsView(&scene);
  setCentralWidget(view);
}

VirtualTable::~VirtualTable() {}

VirtualTableCommunicator &VirtualTable::get_communicator() {
  return communicator;
}

void VirtualTable::refresh_gui() {
  QBrush whiteBrush(Qt::black);
  QPen outlinePen(Qt::black);
  outlinePen.setWidth(20);

  this->scene.clear();
  this->scene.addEllipse(communicator.get_ball().position.x,
                         communicator.get_ball().position.y, 2, 2, outlinePen,
                         whiteBrush);

  this->scene.update();
  this->main_widget->repaint();
  this->main_widget->update();
}
