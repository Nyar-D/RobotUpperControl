#ifndef RENDER_PANEL_HPP
#define RENDER_PANEL_HPP

// rviz
// TODO 测试哪些头文件是不需要引入的
#include <QWidget>
#include <QString>
#include <QVBoxLayout>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <rviz/view_manager.h>
#include <QDebug>

namespace rviz_panel {

class QRenderPanel : public QObject
{
  Q_OBJECT

public:

  explicit QRenderPanel(QBoxLayout *parent = nullptr) :
    grid_(nullptr),
    robot_model_(nullptr),
    marker_(nullptr),
    map_(nullptr),
    laser_scan_(nullptr),
    pose_array_(nullptr),
    global_path_(nullptr),
    local_path_(nullptr),
    global_map_(nullptr),
    local_map_(nullptr)
  {
    if (parent != nullptr)
      setParent(parent);

    // 创建3D面板、中央管理器和tool管理器
    render_panel_ = new rviz::RenderPanel();
    visualization_manager_ = new rviz::VisualizationManager(render_panel_);
    tool_manager_ = visualization_manager_->getToolManager();
    render_panel_->initialize(visualization_manager_->getSceneManager(), visualization_manager_);
    visualization_manager_->initialize();
    visualization_manager_->startUpdate();

    // 设置panel布局
    render_panel_->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

    // 设置插件
    setGlobalOptions("/map", QColor(48, 48, 48), 30);
    enableGrid(true, "<Fixed Frame>", 80, 1.0, QColor(160, 160, 164));
    enableRobotModel(true, "robot_description");
    enableMarker(true, "/robot_upper_control/goal_marker");
    enableMap(true, "/map", 0.5, "map");
    enableLaserScan(true, "/scan");
    enablePoseArray(true, "/particlecloud", "Arrow(Flat)", QColor(170, 255, 127));
    enableGlobalMap(true, "/move_base/global_costmap/costmap", "costmap");
    enableLocalMap(true, "/move_base/local_costmap/costmap", "map");
    enableGlobalPath(true, "/move_base/TrajectoryPlannerROS/global_plan", QColor(25, 255, 0));
    enableLocalPath(true, "/move_base/TrajectoryPlannerROS/local_plan", QColor(164, 0, 0));

    // 设置工具
    if (tool_manager_) {
      tool_manager_->removeAll();
      tool_manager_->addTool("rviz/Interact");
      tool_manager_->addTool("rviz/SetInitialPose");
      tool_manager_->addTool("rviz/SetGoal");
      tool_manager_->addTool("robot_upper_plugins/RouteGoalTool");
      tool_manager_->addTool("robot_upper_plugins/ObstacleAddTool");
    }
  }

  ~QRenderPanel()
  {
    delete render_panel_;
    delete visualization_manager_;
    delete tool_manager_;
    delete grid_;
    delete robot_model_;
    delete marker_;
    delete map_;
    delete laser_scan_;
    delete pose_array_;
    delete global_map_;
    delete local_map_;
    delete global_path_;
    delete local_path_;
  }

  rviz::RenderPanel* getPanel()
  {
    // 获取render_panel
    return this->render_panel_;
  }

  void setGlobalOptions(QString frame_name,  QColor Background_Color, int Frame_Rate)
  {
    visualization_manager_->setFixedFrame(frame_name);
    visualization_manager_->setProperty("Background Color", Background_Color);
    visualization_manager_->setProperty("Frame Rate", Frame_Rate);
  }

  void enableGrid(bool enable, QString Reference_Frame, int Plan_Cell_Count, float Cell_Size, QColor Color=QColor(125, 125, 125))
  {
    if(!enable && grid_)
    {
      grid_ ->setEnabled(false);
      return ;
    }

    if(grid_)
      delete grid_;

    // 创建一个类型为rviz/Grid的网格图层grid_, 根据传入参数设置其在rviz左侧的参数设置
    grid_ = visualization_manager_->createDisplay("rviz/Grid", "QGrid", true);
    ROS_ASSERT(grid_ != nullptr);
    grid_->subProp("Color")->setValue(Color);
    grid_->subProp("Reference Frame")->setValue(Reference_Frame);
    grid_->subProp("Plane Cell Count")->setValue(Plan_Cell_Count);
    grid_->subProp("Line Style")->setValue("Lines");
    grid_->subProp("Cell Size")->setValue(Cell_Size);
    grid_->subProp("Alpha")->setValue(0.5);
    // rid_->subProp("Plane")->setV alue("XY");
    // grid_->subProp("Normal Cell Count")->setValue("0");
    grid_->setEnabled(enable);
  }

  void enableRobotModel(bool enable, QString Robot_Description)
  {
    if(!enable && robot_model_)
    {
      robot_model_->setEnabled(false);
      return ;
    }

    if (robot_model_)
      delete robot_model_;

    // 创建robotModel图层，显示机器人模型
    robot_model_ = visualization_manager_->createDisplay("rviz/RobotModel", "QRobotModel", true);
    ROS_ASSERT(robot_model_ != nullptr);
    robot_model_->subProp("Update Interval")->setValue("0");
    robot_model_->subProp("Alpha")->setValue("1");
    robot_model_->subProp("Robot Description")->setValue(Robot_Description);
    robot_model_->setEnabled(enable);
  }

  void enableMarker(bool enable, QString Marker_Topic)
  {
    if(!enable && marker_)
    {
      marker_->setEnabled(false);
      return ;
    }

    if (marker_)
      delete marker_;

    // 创建robotModel图层，显示机器人模型
    marker_ = visualization_manager_->createDisplay("rviz/Marker", "QMarker", true);
    ROS_ASSERT(marker_ != nullptr);
    marker_->subProp("Marker Topic")->setValue(Marker_Topic);
    marker_->setEnabled(enable);
  }

  void enableMap(bool enable, QString Topic, double Alpha, QString Color_Scheme)
  {
    if(!enable && map_)
    {
      map_->setEnabled(false);
      return ;
    }

    if(map_)
      delete map_;

    // 创建map图层map_，接收/map的数据
    map_=visualization_manager_->createDisplay("rviz/Map", "QMap", true);
    ROS_ASSERT(map_);
    map_->subProp("Topic")->setValue(Topic);
    map_->subProp("Alpha")->setValue(Alpha);
    map_->subProp("Color Scheme")->setValue(Color_Scheme);
    map_->setEnabled(enable);
  }

  void enableLaserScan(bool enable, QString Topic)
  {
    if(!enable && laser_scan_)
    {
      laser_scan_->setEnabled(false);
      return ;
    }

    if (laser_scan_)
      delete laser_scan_;

    // 创建laser图层laserScan_，接收/scan的数据
    laser_scan_ = visualization_manager_->createDisplay("rviz/LaserScan", "QLaserScan", true);
    ROS_ASSERT(laser_scan_ != nullptr);
    laser_scan_->subProp("Topic")->setValue(Topic);
    laser_scan_->subProp("Style")->setValue("Flat Squares");
    laser_scan_->subProp("Size (m)")->setValue("0.03");
    laser_scan_->subProp("Color Transformer")->setValue("FlatColor");  // how to set color?
    laser_scan_->subProp("Color")->setValue("255;0;0");
    laser_scan_->subProp("Queue Size")->setValue("10");
    laser_scan_->subProp("Alpha")->setValue("1");
    laser_scan_->subProp("Position Transformer")->setValue("XYZ");
    laser_scan_->subProp("Channel Name")->setValue("intensity");
    laser_scan_->setEnabled(enable);
  }

  void enablePoseArray(bool enable, QString Topic, QString Shape, QColor Color)
  {
    if (!enable && pose_array_)
    {
      pose_array_->setEnabled(false);
      return;
    }

    if (pose_array_)
      delete pose_array_;

    // 创建poseArray图层poseArray_，接收/particlecould的数据
    pose_array_ = visualization_manager_->createDisplay("rviz/PoseArray", "QPoseArray", true);
    ROS_ASSERT(pose_array_ != nullptr);
    pose_array_->subProp("Topic")->setValue(Topic);
    pose_array_->subProp("Shape")->setValue(Shape);
    pose_array_->subProp("Color")->setValue(Color);
    //  poseArray_->subProp("Queue Size")->setValue(10);
    //  poseArray_->subProp("Alpha")->setValue(1);
    //  poseArray_->subProp("Arrow Length")->setValue(0.3);
    pose_array_->setEnabled(enable);
  }

  void enableGlobalMap(bool enable, QString Topic, QString Color_Scheme)
  {
    if (!enable && global_map_)
    {
      global_map_->setEnabled(false);
      return;
    }

    if (global_map_)
      delete global_map_;

    // 创建Map图层globalMap_
    global_map_ = visualization_manager_->createDisplay("rviz/Map", "QGlobalMap", true);
    ROS_ASSERT(global_map_ != nullptr);
    global_map_->subProp("Topic")->setValue(Topic);
    global_map_->subProp("Color Scheme")->setValue(Color_Scheme);
    global_map_->setEnabled(enable);
  }

  void enableLocalMap(bool enable, QString Topic, QString Color_Scheme)
  {
    if (!enable && local_map_)
    {
      local_map_->setEnabled(false);
      return;
    }

    if (local_map_)
      delete local_map_;

    // 创建Map图层localMap_
    local_map_ = visualization_manager_->createDisplay("rviz/Map", "QLocalMap", true);
    ROS_ASSERT(local_map_ != nullptr);
    local_map_->subProp("Topic")->setValue(Topic);
    local_map_->subProp("Color Scheme")->setValue(Color_Scheme);
    local_map_->setEnabled(enable);
  }

  void enableGlobalPath(bool enable, QString Topic, QColor Color)
  {
    if (!enable && global_path_)
    {
      global_path_->setEnabled(false);
      return;
    }

    if (global_path_)
      delete global_path_;

    // 创建Path图层globalPath_
    global_path_ = visualization_manager_->createDisplay("rviz/Path", "QGlobalPath", true);
    ROS_ASSERT(global_path_ != nullptr);
    global_path_->subProp("Topic")->setValue(Topic);
    global_path_->subProp("Color")->setValue(Color);
    global_path_->setEnabled(enable);
  }

  void enableLocalPath(bool enable, QString Topic, QColor Color)
  {
    if (!enable && local_path_)
    {
      local_path_->setEnabled(false);
      return;
    }

    if (local_path_)
      delete local_path_;

    // 创建Path图层localPath_
    local_path_ = visualization_manager_->createDisplay("rviz/Path", "QLocalPath", true);
    ROS_ASSERT(global_path_ != nullptr);
    local_path_->subProp("Topic")->setValue(Topic);
    local_path_->subProp("Color")->setValue(Color);
    local_path_->setEnabled(enable);
  }

  void setInteract()
  {
    // 设置当前使用的工具
    tool_manager_->setCurrentTool(tool_manager_->getTool(Interact));
  }

  void setPos()
  {
    tool_manager_->setCurrentTool(tool_manager_->getTool(SetInitialPose));
  }

  void setPointGoal()
  {
    tool_manager_->setCurrentTool(tool_manager_->getTool(SetGoal));
    //设置goal的话题
    rviz::Property* pro= tool_manager_->getCurrentTool()->getPropertyContainer();
    pro->subProp("Topic")->setValue("/robot_upper_control/goal_temp");
    visualization_manager_->setFixedFrame("map");
  }

  void setRouteGoal()
  {
    visualization_manager_->setFixedFrame("map");
    tool_manager_->setCurrentTool(tool_manager_->getTool(RouteGoalTool));
  }

  void addObstacle()
  {
    visualization_manager_->setFixedFrame("map");
    tool_manager_->setCurrentTool(tool_manager_->getTool(ObstacleAddTool));
  }

  void delObstacle()
  {
    visualization_manager_->setFixedFrame("map");
    tool_manager_->setCurrentTool(tool_manager_->getTool(ObstacleAddTool));
  }


private:

  rviz::RenderPanel* render_panel_;
  rviz::VisualizationManager* visualization_manager_;
  rviz::ToolManager* tool_manager_;
  rviz::Display* grid_;
  rviz::Display* robot_model_;
  rviz::Display* marker_;
  rviz::Display* map_;
  rviz::Display* laser_scan_;
  rviz::Display* pose_array_;
  rviz::Display* global_map_;
  rviz::Display* local_map_;
  rviz::Display* global_path_;
  rviz::Display* local_path_;

  enum{
    Interact,
    SetInitialPose,
    SetGoal,
    RouteGoalTool,
    ObstacleAddTool
  };
};

} // end of rviz_panel

#endif // RENDER_PANEL_HPP
