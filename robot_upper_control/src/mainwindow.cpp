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
  qrender_panel = new rviz_panel::QRenderPanel(this);
  qroute_goal_penel = new rviz_panel::QRouteGoalPanel(this);

  QHBoxLayout* btn_HBox = new QHBoxLayout();
  btn_HBox->addWidget(ui->set_interact_btn);
  btn_HBox->addWidget(ui->set_pos_btn);
  btn_HBox->addWidget(ui->set_pointGoal_btn);
  btn_HBox->addWidget(ui->set_routeGoal_btn);
  btn_HBox->addWidget(ui->set_obstacleAdd_btn);
  btn_HBox->addWidget(ui->set_obstacleDel_btn);
  btn_HBox->addWidget(ui->set_mapSelect_btn);

  QHBoxLayout* panel_HBox = new QHBoxLayout();
  QBoxLayout *rg_panel = qroute_goal_penel->getPanel();
  rviz::RenderPanel *re_panel = qrender_panel->getPanel();
  hide_left_dock_button_ = new QToolButton(this);
  hide_left_dock_button_->setContentsMargins(0,0,0,0);
  hide_left_dock_button_->setArrowType( Qt::LeftArrow );
  hide_left_dock_button_->setSizePolicy( QSizePolicy( QSizePolicy::Minimum, QSizePolicy::Expanding ) );
  hide_left_dock_button_->setFixedWidth(16);
  hide_left_dock_button_->setAutoRaise(true);
  hide_left_dock_button_->setCheckable(true);

  panel_HBox->addLayout(rg_panel);
  panel_HBox->addWidget(hide_left_dock_button_);
  panel_HBox->addWidget(re_panel);
  panel_HBox->setStretchFactor(rg_panel, 1);
  panel_HBox->setStretchFactor(hide_left_dock_button_, 1);
  panel_HBox->setStretchFactor(re_panel, 5);

  main_VBox->addLayout(btn_HBox);
  main_VBox->addLayout(panel_HBox);
  this->setLayout(main_VBox);
}

MainWindow::~MainWindow()
{
  delete ui;
  delete qnode;
  delete qrender_panel;
  delete qroute_goal_penel;
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
  qrender_panel->selectMap();
}

