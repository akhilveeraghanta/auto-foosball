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
        ~VirtualTableCommunicator();

        void set_ball(const messages::Ball::ConstPtr& msg);
        void set_human_stick(const messages::Stick::ConstPtr& msg);
        void set_ai_stick(const messages::Stick::ConstPtr& msg);

        const messages::Ball& get_ball();
        const messages::Stick& get_human_stick();
        const messages::Stick& get_ai_stick();


    private:
        messages::Stick human_stick;
        messages::Stick ai_stick;
        messages::Ball ball;

    signals:
        void refresh_gui(void) const;

};
