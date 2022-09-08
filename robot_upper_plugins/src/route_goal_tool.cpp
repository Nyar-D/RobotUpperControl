#include <tf/transform_listener.h>
#include "route_goal_tool.h"

namespace robot_upper_plugins
{

RouteGoalTool::RouteGoalTool() :
  Tool()
{
  shortcut_key_ = 'n';
}

RouteGoalTool::~RouteGoalTool()
{
  delete goal_line_;
  delete nh_;
}

void RouteGoalTool::onInitialize()
{
  setName("2D Route Nav Goal");

  setlocale(LC_ALL, "");
  nh_ = new ros::NodeHandle("robot_upper_control");
  route_goal_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("goal_temp", 10);
  marker_delete_pub_ = nh_->advertise<visualization_msgs::Marker>("goal_marker", 1);
  goal_cancel_sub_ = nh_->subscribe<visualization_msgs::Marker>
      ("goal_marker", 1, boost::bind(&RouteGoalTool::markerCallback, this, _1));

  // 启动1个spinner线程并发执行可用回调
  ros::AsyncSpinner spinner(1);
  spinner.start();

  setPointIntervel(20);
  setDistanceIntervel(5.0);

  // 路线绘制
  goal_line_ = new rviz::BillboardLine(context_->getSceneManager());
  goal_line_->setColor(1, 1, 1, 0.5);
  goal_line_->setLineWidth(0.025);
  goal_line_->setMaxPointsPerLine(1000);
  goal_line_->setNumLines(1);
  std_cursor_ = rviz::getDefaultCursor();
  hit_cursor_ = rviz::makeIconCursor( "package://rviz/icons/crosshair.svg" );
}

void RouteGoalTool::activate()
{
  state_ = END;
}

void RouteGoalTool::deactivate()
{
  state_ = END;
}

void RouteGoalTool::save( rviz::Config config ) const
{
}

void RouteGoalTool::load( const rviz::Config& config )
{
}

int RouteGoalTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  int flags = 0;
  Ogre::Vector3 pos;

  // 获取鼠标在地图上的坐标
  bool success = context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, pos);
  setCursor(success ? hit_cursor_ : std_cursor_);

  // 监听鼠标事件，获取坐标，组织并发布路线和导航目标
  if (event.leftDown() && success) {
    switch (state_) {
    case START:
      break;
    case END:
      // 开始
      state_ = START;
      publishMarkerDelete();
      // point_count_ = 0;
      // route_mark_->clear();
      distance_ = 0;
      last_pos_ = pos;
      last_last_pos_ = last_pos_;
      break;
    }

    flags |= Render;
  }

  if (event.leftUp() && success) {
    double theta = 0;
    switch (state_) {
    case START:
      // 终点在此处绘制
//      goal_line_->addPoint(pos);
      theta = atan2((last_pos_.y - last_last_pos_.y),
                     (last_pos_.x - last_last_pos_.x));
      publishRouteGoal(pos.x, pos.y, theta);
      state_ = END;
      break;
    case END:
      break;
    }

    flags |= Render;
  }

  switch (state_) {
  case START:
    // 起点、中途点在此处绘制
    publishRouteGoal(pos, last_pos_);
    goal_line_  ->addPoint(pos);
    point_count_ = (point_count_ + 1) % point_intervel_;
    last_last_pos_ = last_pos_;
    last_pos_ = pos;
    break;
  case END:
    break;

    flags |= Finished;
  }

  return flags;
}

// 这个函数用于发布路线上的导航目标
void RouteGoalTool::publishRouteGoal(Ogre::Vector3& pos, Ogre::Vector3& last_pos)
{
//  publishByPointInterval(pos, last_pos);
  publishByDistanceInterval(pos, last_pos);
}

// 不同的策略
void RouteGoalTool::publishByPointInterval(Ogre::Vector3& pos, Ogre::Vector3& last_pos)
{
  if (point_count_ == 1) {
    double theta = 0;
    theta = atan2((pos.y - last_pos_.y), (pos.x - last_pos_.x));
    publishRouteGoal(pos.x, pos.y, theta);
  }
}

void RouteGoalTool::publishByDistanceInterval(Ogre::Vector3& pos, Ogre::Vector3& last_pos)
{
  if (distance_ >= distance_intervel_ || point_count_ == 1) {
    double theta = 0;
    theta = atan2((pos.y - last_pos_.y), (pos.x - last_pos_.x));
    publishRouteGoal(pos.x, pos.y, theta);
    distance_ = 0.0f;
  }
  distance_ += (pos - last_pos_).length();
}

void RouteGoalTool::publishRouteGoal(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);
  route_goal_pub_.publish(goal);
}

void RouteGoalTool::publishMarkerDelete()
{
  visualization_msgs::Marker marker_delete;
  marker_delete.action = visualization_msgs::Marker::DELETEALL;
  marker_delete_pub_.publish(marker_delete);
}

void RouteGoalTool::markerCallback(const visualization_msgs::Marker::ConstPtr& marker_delete)
{
  if (marker_delete->action == visualization_msgs::Marker::DELETEALL) {
    point_count_ = 0;
    goal_line_->clear();
  }
}

void RouteGoalTool::setPointIntervel(int point_intervel)
{
  point_intervel_ = point_intervel;
}

void RouteGoalTool::setDistanceIntervel(double distance_intervel)
{
  distance_intervel_ = distance_intervel;
}

} // end namespace robot_upper_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robot_upper_plugins::RouteGoalTool, rviz::Tool )

