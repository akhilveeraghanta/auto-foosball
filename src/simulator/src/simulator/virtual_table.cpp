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
#include "virtual_table_communicator.h"
#include "virtual_table.h"
/**********************************************************************
 *                    Virtual Table Implementation                    *
 **********************************************************************/

VirtualTable::VirtualTable(QWidget *parent, int width_px, int height_px):
    QMainWindow(parent)
{
    connect(&communicator,
            &VirtualTableCommunicator::refresh_gui,
            this,
            &VirtualTable::refresh_gui);

    setStyleSheet("background-color:green;");
}

VirtualTable::~VirtualTable(){}

VirtualTableCommunicator& VirtualTable::get_communicator(){
    return communicator;
}

void VirtualTable::refresh_gui() {
    ROS_INFO("Signal Triggered: [Position.x: %f] [Position.y: %f]",
            communicator.get_ball().position.x, communicator.get_ball().position.y);
    QBrush greenBrush(Qt::green);
    QPen outlinePen(Qt::black);
    outlinePen.setWidth(2);
    //this->addEllipse(int(msg -> position.x), int(msg -> position.y), 2, 2, outlinePen, greenBrush);
}

