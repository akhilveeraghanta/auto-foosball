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

class Foosball : public QWidget {
    public:
        Foosball(QWidget *parent=0) : QWidget(parent) {
            setStyleSheet("background-color:white;");
            resize(B_WIDTH, B_HEIGHT);
            move(500, 500);
            loadImages();
        }

        void adjustBall(const messages::Ball::ConstPtr& msg) {
            ROS_INFO("I heard: [Position.x: %f] [Position.y: %f]", msg->position.x, msg->position.y);
            x_pos = msg -> position.x;
            y_pos = msg -> position.y;
        }

    private:
        QImage ball_img;
        int x_pos;
        int y_pos;

        static const int B_WIDTH = 300;
        static const int B_HEIGHT = 300;

        void loadImages() {
            ball_img.load("soccer.jpg");
        }

        void paintEvent(QPaintEvent *e) {
            ROS_INFO("Paint event");
            Q_UNUSED(e);
            doDrawing();
        }

        void doDrawing() {
            QPainter qp(this);
            qp.drawImage(1,1,ball_img);
            gameOver(qp);
        }

        void gameOver(QPainter &qp) {
            QString message = "O";
            QFont font("Courier", 15, QFont::DemiBold);
            QFontMetrics fm(font);
            int textWidth = fm.width(message);

            qp.setFont(font);
            int h =height();
            int w = width();

            if(!x_pos && !y_pos) {
                qp.translate(QPoint(w/2, h/2));
            } else {
                qp.translate(QPoint(x_pos, y_pos));
            }
            qp.drawText(-textWidth/2, 0, message);
        }
};

int main(int argc, char** argv) {
    // init ROS visual table node
    ros::init(argc, argv, "virtual_table");
    ros::NodeHandle nh;

    QApplication app(argc, argv);
    Foosball window;

    ros::Subscriber ball_position_sub = nh.subscribe("/simulator/ball", 1000, &Foosball::adjustBall, &window);

    //QWidget window;
    //Timer window;

    
    //window.resize(250,150);
    window.setMinimumSize(50,200);
    window.setWindowTitle("Foosball Visualizer");
    window.show();

    boost::thread thread_spin (boost::bind(ros::spin));

    return app.exec();
}
