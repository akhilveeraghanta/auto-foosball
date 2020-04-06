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

    setStyleSheet("background-color:white;");
    scene.setSceneRect(0,0,500,500);
    main_widget = new QWidget();
    view = new QGraphicsView(&scene);
    setCentralWidget(view);
}

VirtualTable::~VirtualTable(){}

VirtualTableCommunicator& VirtualTable::get_communicator(){
    return communicator;
}

void VirtualTable::refresh_gui() {
    QBrush whiteBrush(Qt::black);
    QPen outlinePen(Qt::black);
    outlinePen.setWidth(20);

    this->scene.clear();
    this->scene.addEllipse(
            communicator.get_ball().position.x,
            communicator.get_ball().position.y,
            2, 2, outlinePen, whiteBrush);

    this->scene.update();
    this->main_widget->repaint();
    this->main_widget->update();
}
