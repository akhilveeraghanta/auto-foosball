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
#include "virtual_table_communicator.h"
/**
 * Main Virtual Table Widget
 *
 * - handles drawing the ball position + player position
 * - generates the required shapes and field boundaries
 * - TODO move this to its own file
 */
class VirtualTable : public QMainWindow {

        Q_OBJECT
    public:

        /**
         * Creates a new VirtualTable widget
         *
         * @param parent The parent widget, defaults to nothing
         * @param width_px The width of the widget
         * @param height_px The height of the widget
         */
        explicit VirtualTable(QWidget *parent=0, int width_px=500, int height_px=500);
        ~VirtualTable();
        const VirtualTableCommunicator& get_communicator();

        private slots:
        void update_ball_position(const messages::Ball::ConstPtr& msg);
        void update_human_stick_position(const messages::Stick::ConstPtr& msg);
        void update_ai_stick_position(const messages::Stick::ConstPtr& msg);


    private:
        VirtualTable* ui;
        QImage ball_img;
        VirtualTableCommunicator communicator;
};
