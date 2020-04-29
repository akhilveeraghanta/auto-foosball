#pragma once
#include "messages/Ball.h"
#include "messages/Stick.h"
#include "ros/ros.h"
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

/**
 * Virtual Table Communicator
 * Object to send data to the main virtual table
 *
 * @param parent The parent widget, defaults to nothing
 */
class VirtualTableCommunicator : public QObject {
  Q_OBJECT

public:
  explicit VirtualTableCommunicator(QObject *parent = 0);
  ~VirtualTableCommunicator();

  void set_ball(const messages::Ball::ConstPtr &msg);
  void set_human_stick(const messages::Stick::ConstPtr &msg);
  void set_ai_stick(const messages::Stick::ConstPtr &msg);

  const messages::Ball &get_ball();
  const messages::Stick &get_human_stick();
  const messages::Stick &get_ai_stick();

private:
  messages::Stick human_stick;
  messages::Stick ai_stick;
  messages::Ball ball;

signals:
  void refresh_gui(void) const;
};
