#include "robot_upper_plugins/MapAddObstacle.h"
#include "robot_upper_plugins/MapCrop.h"
#include "obstacle_add_tool.h"

namespace robot_upper_plugins
{

ObstacleAddTool::ObstacleAddTool() :
  Tool()
{
  shortcut_key_ = 'm';
}

ObstacleAddTool::~ObstacleAddTool()
{
  destoryAllPoints();
  delete obstacle_line_;
  delete nh_;
}

void ObstacleAddTool::onInitialize()
{
  setName("2D Obstacle Add");

  setlocale(LC_ALL, "");
  nh_ = new ros::NodeHandle();
  aobs_client_ = nh_->serviceClient<robot_upper_plugins::MapAddObstacle>("MapAddObstacle");

  obstacle_line_ = new rviz::BillboardLine(context_->getSceneManager());
  obstacle_line_->setColor(1, 1, 1, 1);
  obstacle_line_->setLineWidth(0.05);
  obstacle_line_->setMaxPointsPerLine(500);
  obstacle_line_->setNumLines(20);

  // 路线绘制
  std_cursor_ = rviz::getDefaultCursor();
  hit_cursor_ = rviz::makeIconCursor( "package://rviz/icons/crosshair.svg" );
}

void ObstacleAddTool::activate()
{
  state_ = END;
}

void ObstacleAddTool::deactivate()
{
  // 绘制结束发布消息
  if (current_line_ != -1) {
    sendObstacleAddRequest();
  }

  state_ = END;
  destoryAllPoints();
  obstacle_line_->clear();
  current_line_ = -1;
}

void ObstacleAddTool::save( rviz::Config config ) const
{
}

void ObstacleAddTool::load( const rviz::Config& config )
{
}

int ObstacleAddTool::processMouseEvent( rviz::ViewportMouseEvent& event )
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
      addNewLine();
      break;
    }

    flags |= Render;
  }

  switch (state_) {
  case START:
    // 起点、中途点、终点在此处绘制
    obstacle_line_->addPoint(pos);
    points_list_[current_line_]->insert(std::make_pair(pos.x, pos.y));
    break;
  case END:
    break;

    flags |= Finished;
  }

  if (event.leftUp() && success) {
    switch (state_) {
    case START:
      state_ = END;
      break;
    case END:
      break;
    }

    flags |= Render;
  }

  return flags;
}

void ObstacleAddTool::sendObstacleAddRequest()
{
  if (current_line_ == -1) {
    ROS_ERROR("Error occured!");
    return;
  }

  // 发送服务请求
  robot_upper_plugins::MapAddObstacle mao;
  mao.request.gen_map_name = "gen.pgm";
  for (int i = 0; i <= current_line_; i++) {
    for (auto iter = points_list_[i]->begin(); iter != points_list_[i]->end(); iter++) {
      geometry_msgs::Point p;
      p.x = iter->first;
      p.y = iter->second;
      p.z = 0;
      mao.request.points.push_back(p);
    }
  }

  // 等待服务响应
  ros::service::waitForService("MapAddObstacle");
  bool flag = aobs_client_.call(mao);
  if (flag) {
    const std::string status = mao.response.status;
    if (status == "success")
      ROS_INFO("Obstacle Add: 添加障碍物成功！");
    else if (status == "")
      ROS_ERROR("Obstacle Add: 异常, 服务无返回数据");
    else
      ROS_ERROR("Obstacle Add: %s", status.c_str());
  } else
    ROS_ERROR("Obstacle Add: 服务调用失败！");
}

void ObstacleAddTool::addNewLine()
{
  obstacle_line_->newLine();
  points* new_points = new points();
  points_list_.append(new_points);
  current_line_++;
}

void ObstacleAddTool::destoryAllPoints()
{
  for (auto iter = points_list_.begin(); iter != points_list_.end(); iter++) {
    if (*iter != nullptr) {
      delete *iter;
      *iter = nullptr;
    }
  }
  points_list_.clear();
}

} // end namespace robot_upper_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robot_upper_plugins::ObstacleAddTool, rviz::Tool )
