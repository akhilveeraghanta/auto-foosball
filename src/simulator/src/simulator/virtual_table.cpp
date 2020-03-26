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

class Foosball : public QWidget {
    public:
        Foosball(QWidget *parent=0) : QWidget(parent) {
            setStyleSheet("background-color:white;");
            resize(B_WIDTH, B_HEIGHT);
            move(500, 500);
            loadImages();
        }

    private:
        QImage ball_img;

        static const int B_WIDTH = 300;
        static const int B_HEIGHT = 300;

        void loadImages() {
            ball_img.load("../../resources/soccer_ball.png");
        }

        void paintEvent(QPaintEvent *e) {
            Q_UNUSED(e);
            doDrawing();
        }

        void doDrawing() {
            QPainter qp(this);
            qp.drawImage(100,100,ball_img);
            gameOver(qp);
        }

        void gameOver(QPainter &qp) {
            QString message = "Game over";
            QFont font("Courier", 15, QFont::DemiBold);
            QFontMetrics fm(font);
            int textWidth = fm.width(message);

            qp.setFont(font);
            int h =height();
            int w = width();

            qp.translate(QPoint(w/2, h/2));
            qp.drawText(-textWidth/2, 0, message);
        }
};

void adjustBall(const messages::Ball::ConstPtr& msg) {
    ROS_INFO("I heard: [Position.x: %f] [Position.y: %f]", msg->position.x, msg->position.y);
}

class Timer : public QWidget {
    public:
        Timer(QWidget *parent=0) : QWidget(parent) {
            QHBoxLayout *hbox = new QHBoxLayout(this);
            hbox -> setSpacing(5);
            label = new QLabel("", this);
            hbox->addWidget(label, 0, Qt::AlignLeft | Qt::AlignTop);
            QTime qtime = QTime::currentTime();
            QString stime = qtime.toString();
            label->setText(stime);
            startTimer(1000);
        }

    protected:
        void timerEvent(QTimerEvent *e){
            Q_UNUSED(e);
            QTime qtime = QTime::currentTime();
            QString stime = qtime.toString();
            label->setText(stime);
        }
    private:
        QLabel *label;
};

int main(int argc, char** argv) {
    // init ROS visual table node
    ros::init(argc, argv, "virtual_table");
    ros::NodeHandle nh;
    ros::Subscriber ball_position_sub = nh.subscribe("/simulator/ball", 1000, adjustBall);

    QApplication app(argc, argv);
    //QWidget window;
    //Foosball window;
    Timer window;

    window.resize(250,150);
    window.setWindowTitle("Foosball Visualizer");
    window.show();

    ros::spin();

    return app.exec();
}
