#pragma once
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
  explicit VirtualTable(QWidget *parent = 0, int width_px = 500,
                        int height_px = 500);
  ~VirtualTable();

  VirtualTableCommunicator &get_communicator(void);

private slots:
  void refresh_gui(void);

private:
  QWidget *main_widget;
  QGraphicsScene scene;
  QGraphicsView *view;
  VirtualTable *ui;
  QImage ball_img;
  VirtualTableCommunicator communicator;
};
