#ifndef ROUTE_GOAL_PANEL_HPP
#define ROUTE_GOAL_PANEL_HPP

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <rviz/panel.h>
#include <QString>
#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_datatypes.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>

namespace rviz_panel {

class QRouteGoalPanel : public QObject
{
  Q_OBJECT

public:

  explicit QRouteGoalPanel(QWidget *parent = 0) :
    maxNumGoal_(1)
  {
    if (parent != nullptr)
      setParent(parent);

    // 创建订阅和发布
    nh1_ = new ros::NodeHandle("robot_upper_control");
    marker_pub_ = nh1_->advertise<visualization_msgs::Marker>("goal_marker", 10);
    goal_sub_ = nh1_->subscribe<geometry_msgs::PoseStamped>("goal_temp", 10, boost::bind(&QRouteGoalPanel::NaviGoalCnt, this, _1));

    nh2_ = new ros::NodeHandle();
    status_sub_ = nh2_->subscribe<actionlib_msgs::GoalStatusArray>("move_base/status", 10, boost::bind(&QRouteGoalPanel::NaviStatusCB, this, _1));
    goal_pub_ = nh2_->advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    cancel_pub_ = nh2_->advertise<actionlib_msgs::GoalID>("move_base/cancel", 10);

    // 启动1个spinner线程并发执行可用回调
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 创建Panel窗体
    initMainWidget();
  }

  ~QRouteGoalPanel()
  {
    delete panel_layout_;
    delete output_maxNumGoal_editor_;
    delete output_maxNumGoal_button_;
    delete output_reset_button_;
    delete output_startNavi_button_;
    delete output_cancel_button_;
    delete poseArray_table_;
    delete cycle_checkbox_;
  }

  QBoxLayout* getPanel()
  {
    return panel_layout_;
  }


private slots:

  // 更新最多目标数量
  void updateMaxNumGoal()
  {
    QString new_maxNumGoal = output_maxNumGoal_editor_->text();
    // 检查maxNumGoal是否发生改变.
    if (new_maxNumGoal != output_maxNumGoal_) {
      output_maxNumGoal_ = new_maxNumGoal;

      // 如果命名为空，不发布任何信息
      if (output_maxNumGoal_ == "") {
        maxNumGoal_ = 1;
        nh1_->setParam("maxNumGoal_", maxNumGoal_);
      } else {
        maxNumGoal_ = output_maxNumGoal_.toInt();
        nh1_->setParam("maxNumGoal_", maxNumGoal_);
      }
    }
  }

  // 更新表格行数
  void updatePoseTable()
  {
    poseArray_table_->setRowCount(maxNumGoal_);
    QStringList pose_header;
    pose_header << "x" << "y" << "yaw";
    poseArray_table_->setHorizontalHeaderLabels(pose_header);
    poseArray_table_->show();
  }

  // 初始化表格
  void initPoseTable()
  {
    curGoalIdx_ = 0;
    cycleCnt_ = 0;
    permit_ = false;
    cycle_ = false;
    poseArray_table_->clear();
    pose_array_.poses.clear();
    removeRouteMark();

    poseArray_table_->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    poseArray_table_->horizontalHeader()->setMinimumSectionSize(40);
    poseArray_table_->horizontalHeader()->setMinimumSize(40, 25);
    poseArray_table_->setRowCount(maxNumGoal_);
    poseArray_table_->setColumnCount(3);
    poseArray_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    poseArray_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    QStringList pose_header;
    pose_header << "x" << "y" << "yaw";
    poseArray_table_->setHorizontalHeaderLabels(pose_header);
  }

  void checkCycle()
  {
    cycle_ = cycle_checkbox_->isChecked();
  }

  // 取消导航
  void cancelNavi()
  {
    if (!cur_goalid_.id.empty()) {
      cancel_pub_.publish(cur_goalid_);
      ROS_ERROR("Navigation have been canceled");
    }
  }

  // 从第一个目标点开始发布导航目标
  void startNavi()
  {
    curGoalIdx_ = curGoalIdx_ % pose_array_.poses.size();
    if (!pose_array_.poses.empty() && curGoalIdx_ < maxNumGoal_) {
      geometry_msgs::PoseStamped goal;
      goal.header = pose_array_.header;
      goal.pose = pose_array_.poses.at(curGoalIdx_);
      goal_pub_.publish(goal);
      ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);
      poseArray_table_->item(curGoalIdx_, 0)->setBackground(QBrush(QColor(255, 69, 0)));
      poseArray_table_->item(curGoalIdx_, 1)->setBackground(QBrush(QColor(255, 69, 0)));
      poseArray_table_->item(curGoalIdx_, 2)->setBackground(QBrush(QColor(255, 69, 0)));
      curGoalIdx_ += 1;
      permit_ = true;
    } else {
      ROS_ERROR("Something Wrong");
    }
  }


private:

  // 创建Panel窗体
  void initMainWidget()
  {
    // 设置panel布局
    panel_layout_ = new QVBoxLayout;

    // maxNumGoal_layout
    QHBoxLayout *maxNumGoal_layout = new QHBoxLayout;
    // Lable
    maxNumGoal_layout->addWidget(new QLabel("目标数量"));
    // LineEdit
    output_maxNumGoal_editor_ = new QLineEdit();
    output_maxNumGoal_editor_->setMinimumWidth(20);
    output_maxNumGoal_editor_->setFixedHeight(25);
    output_maxNumGoal_editor_->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed));
    maxNumGoal_layout->addWidget(output_maxNumGoal_editor_);
    // Button
    output_maxNumGoal_button_ = new QPushButton("确定");
    output_maxNumGoal_button_->setMinimumWidth(40);
    output_maxNumGoal_button_->setFixedHeight(25);
    output_maxNumGoal_button_->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed));
    maxNumGoal_layout->addWidget(output_maxNumGoal_button_);
    panel_layout_->addLayout(maxNumGoal_layout);

    // CheckBox
    cycle_checkbox_ = new QCheckBox("循环");
    cycle_checkbox_->setCheckState(Qt::Unchecked);
    panel_layout_->addWidget(cycle_checkbox_);

    // Table
    poseArray_table_ = new QTableWidget;
    initPoseTable();
    panel_layout_->addWidget(poseArray_table_);

    // manipulate layout
    QHBoxLayout *manipulate_layout = new QHBoxLayout;
    manipulate_layout->setAlignment(Qt::AlignJustify);
    // Button
    output_reset_button_ = new QPushButton("重置");
    output_reset_button_->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed));
    output_reset_button_->setMinimumSize(40, 25);
    manipulate_layout->addWidget(output_reset_button_);
    // Button
    output_cancel_button_ = new QPushButton("取消");
    output_cancel_button_->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed));
    output_cancel_button_->setMinimumSize(40, 25);
    manipulate_layout->addWidget(output_cancel_button_);
    // Button
    output_startNavi_button_ = new QPushButton("导航");
    output_startNavi_button_->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed));
    output_startNavi_button_->setMinimumSize(40, 25);
    manipulate_layout->addWidget(output_startNavi_button_);
    panel_layout_->addLayout(manipulate_layout);

    // 连接信号与槽
    connect(output_maxNumGoal_button_, SIGNAL(clicked()), this, SLOT(updateMaxNumGoal()));
    connect(output_maxNumGoal_button_, SIGNAL(clicked()), this, SLOT(updatePoseTable()));
    connect(output_reset_button_, SIGNAL(clicked()), this, SLOT(initPoseTable()));
    connect(cycle_checkbox_, SIGNAL(clicked(bool)), this, SLOT(checkCycle()));
    connect(output_cancel_button_, SIGNAL(clicked()), this, SLOT(cancelNavi()));
    connect(output_startNavi_button_, SIGNAL(clicked()), this, SLOT(startNavi()));
  }

    void writePose(geometry_msgs::Pose pose)
    {
      poseArray_table_->setItem(pose_array_.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
      poseArray_table_->setItem(pose_array_.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
      poseArray_table_->setItem(pose_array_.poses.size() - 1, 2,
                                new QTableWidgetItem(
                                  QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    void markPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
    {
      if (ros::ok()) {
        visualization_msgs::Marker arrow;
        visualization_msgs::Marker number;
        arrow.header.frame_id = number.header.frame_id = pose->header.frame_id;
        arrow.ns = "navi_point_arrow";
        number.ns = "navi_point_number";
        arrow.action = number.action = visualization_msgs::Marker::ADD;
        arrow.type = visualization_msgs::Marker::ARROW;
        number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        arrow.pose = number.pose = pose->pose;
        number.pose.position.z += 1.0;
        arrow.pose.position.z += 0.01;
        arrow.scale.x = 0.3;
        arrow.scale.y = 0.04;
        number.scale.z = 0.3;
        arrow.color.r = number.color.r = 0.0f;
        arrow.color.g = number.color.g = 0.0f;
        arrow.color.b = number.color.b = 0.0f;
        arrow.color.a = number.color.a = 0.7;
      arrow.id = number.id = pose_array_.poses.size();
      number.text = std::to_string(pose_array_.poses.size());
      marker_pub_.publish(arrow);
      marker_pub_.publish(number);
    }
  }

  // 清除标记
  void removeRouteMark()
  {
    visualization_msgs::Marker marker_delete;
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker_delete);
  }

  // 若未完成导航，则发布下一个目标点
  void completeNavi()
  {
    if (curGoalIdx_ < pose_array_.poses.size()) {
      geometry_msgs::PoseStamped goal;
      goal.header = pose_array_.header;
      goal.pose = pose_array_.poses.at(curGoalIdx_);
      goal_pub_.publish(goal);
      ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);
      poseArray_table_->item(curGoalIdx_, 0)->setBackground(QBrush(QColor(255, 69, 0)));
      poseArray_table_->item(curGoalIdx_, 1)->setBackground(QBrush(QColor(255, 69, 0)));
      poseArray_table_->item(curGoalIdx_, 2)->setBackground(QBrush(QColor(255, 69, 0)));
      curGoalIdx_ += 1;
      permit_ = true;
    } else {
      ROS_ERROR("All goals are completed");
      permit_ = false;
    }
  }

  // 循环导航
  void cycleNavi()
  {
    if (permit_) {
      geometry_msgs::PoseStamped goal;
      goal.header = pose_array_.header;
      goal.pose = pose_array_.poses.at(curGoalIdx_ % pose_array_.poses.size());
      goal_pub_.publish(goal);
      ROS_INFO("Navi to the Goal%lu, in the %dth cycle", curGoalIdx_ % pose_array_.poses.size() + 1,
               cycleCnt_ + 1);
      bool even = ((cycleCnt_ + 1) % 2 != 0);
      QColor color_table;
      if (even) color_table = QColor(255, 69, 0); else color_table = QColor(100, 149, 237);
      poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 0)->setBackground(QBrush(QColor(255, 69, 0)));
      poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 1)->setBackground(QBrush(QColor(255, 69, 0)));
      poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 2)->setBackground(QBrush(QColor(255, 69, 0)));
      curGoalIdx_ += 1;
      cycleCnt_ = curGoalIdx_ / pose_array_.poses.size();
    }
  }

  // 检查是否达到目标
  bool checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list)
  {
    bool done;
    if (!status_list.empty()) {
        for (auto &i : status_list) {
            if (i.status == 3) {
                done = true;
                ROS_INFO("completed Goal%d", curGoalIdx_);
            } else if (i.status == 4) {
                ROS_ERROR("Goal%d is Invalid, Navi to Next Goal%d", curGoalIdx_, curGoalIdx_ + 1);
                return true;
            } else if (i.status == 0) {
                done = true;
            } else if (i.status == 1) {
                cur_goalid_ = i.goal_id;
//                emit set_param_signal("cur_goalid_", i.goal_id);
                done = false;
            } else done = false;
        }
    } else {
        ROS_INFO("Please input the Navi Goal");
        done = false;
    }
    return done;
  }

  // goal_sub_回调函数, 用于获取多个导航目标
  void NaviGoalCnt(const geometry_msgs::PoseStamped::ConstPtr& pose)
  {
    if (pose_array_.poses.size() < maxNumGoal_) {
      pose_array_.poses.push_back(pose->pose);
      pose_array_.header.frame_id = pose->header.frame_id;
      writePose(pose->pose);
      markPose(pose);
    } else {
      ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
    }
  }

  // status_pub_回调函数, 用于获取当前导航action状态
  void NaviStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& statuses)
  {
    bool arrived_pre = arrived_;
    arrived_ = checkGoal(statuses->status_list);
    if (arrived_) { ROS_ERROR("%d,%d", int(arrived_), int(arrived_pre)); }
    if (arrived_ && arrived_ != arrived_pre && ros::ok() && permit_) {
      if (cycle_) cycleNavi();
      else completeNavi();
    }
  }


private:

  QVBoxLayout *panel_layout_;
  QLineEdit *output_maxNumGoal_editor_;
  QPushButton *output_maxNumGoal_button_, *output_reset_button_, *output_startNavi_button_, *output_cancel_button_;
  QTableWidget *poseArray_table_;
  QCheckBox *cycle_checkbox_;
  QString output_maxNumGoal_;

  int maxNumGoal_;
  int curGoalIdx_ = 0;
  int cycleCnt_ = 0;
  bool permit_ = false;
  bool cycle_ = false;
  bool arrived_ = false;

  // ROS node
  ros::NodeHandle *nh1_, *nh2_;
  ros::Publisher goal_pub_, cancel_pub_, marker_pub_;
  ros::Subscriber goal_sub_, status_sub_;
  geometry_msgs::PoseArray pose_array_;
  actionlib_msgs::GoalID cur_goalid_;
};

} // end of rviz_panel

#endif // ROUTE_GOAL_PANEL_HPP
