#include "mainwindow.h"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
  QWidget(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  // 启动ROS节点
  qnode = new QNode(argc, argv);
//  qnode->init("http://robot:11311/", "dmaster");
  qnode->init();

  // 嵌入视图控件
  main_VBox = new QVBoxLayout();
  qrender_panel = new rviz_panel::QRenderPanel(main_VBox);
  qroute_goal_penel = new rviz_panel::QRouteGoalPanel(main_VBox);
  QHBoxLayout* panel_HBox = new QHBoxLayout();
  QHBoxLayout* btn_HBox = new QHBoxLayout();
  btn_HBox->addWidget(ui->set_interact_btn);
  btn_HBox->addWidget(ui->set_pos_btn);
  btn_HBox->addWidget(ui->set_pointGoal_btn);
  btn_HBox->addWidget(ui->set_routeGoal_btn);
  btn_HBox->addWidget(ui->set_obstacleAdd_btn);
  btn_HBox->addWidget(ui->set_obstacleDel_btn);
  btn_HBox->addStretch();
  panel_HBox->addLayout(qroute_goal_penel->getPanel());
  panel_HBox->addWidget(qrender_panel->getPanel());
  main_VBox->addLayout(btn_HBox);
  main_VBox->addLayout(panel_HBox);
  this->setLayout(main_VBox);
}

MainWindow::~MainWindow()
{
  delete ui;
  delete qrender_panel;
  delete qnode;
  delete main_VBox;
}

void MainWindow::on_set_interact_btn_clicked()
{
  qrender_panel->setInteract();
}

void MainWindow::on_set_pos_btn_clicked()
{
  qrender_panel->setPos();
}

void MainWindow::on_set_pointGoal_btn_clicked()
{
  qrender_panel->setPointGoal();
}

void MainWindow::on_set_routeGoal_btn_clicked()
{
  qrender_panel->setRouteGoal();
}

void MainWindow::on_set_obstacleAdd_btn_clicked()
{
  qrender_panel->addObstacle();
}

void MainWindow::on_set_obstacleDel_btn_clicked()
{
  qrender_panel->delObstacle();
}

void MainWindow::on_set_mapSelect_btn_clicked()
{
  MapSelectDialog *dialog = new MapSelectDialog(this);
  dialog->setWindowTitle(tr("hello"));
  dialog->setAttribute(Qt::WA_DeleteOnClose);
  dialog->exec();
}

