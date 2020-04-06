#pragma once
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

/**
 * Virtual Table Communicator
 * Object to send data to the main virtual table
 *
 * @param parent The parent widget, defaults to nothing
 */
class VirtualTableCommunicator : public QObject
{
        Q_OBJECT
    public:
         explicit VirtualTableCommunicator(QObject * parent = 0);

         void send_ball_position(const messages::Ball::ConstPtr& msg) const;
         void send_human_stick_position(const messages::Stick::ConstPtr& msg) const;
         void send_ai_stick_position(const messages::Stick::ConstPtr& msg) const;

         ~VirtualTableCommunicator();

    signals:
        void send_ball_position_to_gui(const messages::Ball::ConstPtr& msg) const;
        void send_human_stick_position_to_gui(const messages::Stick::ConstPtr& msg) const;
        void send_ai_stick_position_to_gui(const messages::Stick::ConstPtr& msg) const;
};
