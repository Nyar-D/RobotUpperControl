#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QDebug>
#include <QThread>
#include "qnode.h"
#include "render_panel.hpp"
#include "route_goal_panel.hpp"
#include "mapselectdialog.h"
#include "ui_mainwindow.h"

namespace Ui {
class MainWindow;
class MapSelectDialog;
}

class MainWindow : public QWidget
{
  Q_OBJECT

public:
  explicit MainWindow(int argc, char **argv, QWidget *parent = nullptr);
  ~MainWindow();

private slots:
  void on_set_interact_btn_clicked();
  void on_set_pos_btn_clicked();
  void on_set_pointGoal_btn_clicked();
  void on_set_routeGoal_btn_clicked();
  void on_set_obstacleAdd_btn_clicked();
  void on_set_obstacleDel_btn_clicked();
  void on_set_mapSelect_btn_clicked();

private:
  Ui::MainWindow* ui;
  QNode* qnode;
  rviz_panel::QRenderPanel* qrender_panel;
  rviz_panel::QRouteGoalPanel* qroute_goal_penel;
  QThread *th_node;
  QVBoxLayout* main_VBox;

};

#endif // MAINWINDOW_H
