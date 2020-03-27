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
#include <boost/thread/thread.hpp>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QDialog>
#include <QColor>
#include <QTransform>

class Foosball : public QWidget {
    public:
        Foosball(QWidget *parent=0) : QWidget(parent), scene(new QGraphicsScene(this)) {
            setStyleSheet("background-color:white;");
            resize(B_WIDTH, B_HEIGHT);
            move(500, 500);
        }

        void adjustBall(const messages::Ball::ConstPtr& msg) {
            ROS_INFO("I heard: [Position.x: %f] [Position.y: %f]", msg->position.x, msg->position.y);
            QBrush greenBrush(Qt::green);
            QPen outlinePen(Qt::black);
            outlinePen.setWidth(2);
            scene -> addEllipse(int(msg -> position.x), int(msg -> position.y), 2, 2, outlinePen, greenBrush);
        }

    private:
        QImage ball_img;
        QGraphicsScene* scene;

        int x_pos;
        int y_pos;

        static const int B_WIDTH = 300;
        static const int B_HEIGHT = 300;
};

void adjustBall(const messages::Ball::ConstPtr& msg) {
    ROS_INFO("I heard: [Position.x: %f] [Position.y: %f]", msg->position.x, msg->position.y);
    QBrush greenBrush(Qt::green);
    QPen outlinePen(Qt::black);
    outlinePen.setWidth(2);
    //scene -> addEllipse(int(msg -> position.x), int(msg -> position.y), 15, 15, outlinePen, greenBrush);
}

int main(int argc, char** argv) {
    // init ROS visual table node
    ros::init(argc, argv, "virtual_table");
    ros::NodeHandle nh;

    QApplication app(argc, argv);

    QBrush greenBrush(Qt::green);
    QPen outlinePen(Qt::black);
    outlinePen.setWidth(2);

    QGraphicsScene scene;
    scene.addEllipse(400,400,15,15,outlinePen,greenBrush);
    scene.addEllipse(110,100,40,40,outlinePen,greenBrush);

    QGraphicsView view(&scene);
    view.resize(500,500);
    view.setWindowTitle("Foosball Visualizer");
    view.show();

    ros::Subscriber ball_position_sub = nh.subscribe("/simulator/ball", 1000, adjustBall);

    boost::thread thread_spin(boost::bind(ros::spin));

    return app.exec();
}
