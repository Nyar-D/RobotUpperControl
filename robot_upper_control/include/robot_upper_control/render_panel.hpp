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
#include <std_msgs/String.h>

#include "mapselectdialog.h"

namespace rviz_panel {

class QRenderPanel : public QObject
{
  Q_OBJECT

public:

  explicit QRenderPanel(QWidget *parent = nullptr) :
    grid_(nullptr),
    robot_model_(nullptr),
    goal_arrow_marker_(nullptr),
    goal_pose_(nullptr),
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

    nh_ = new ros::NodeHandle("robot_upper_control");
    map_req_pub = nh_->advertise<std_msgs::String>("static_map", 1);

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
    enableGrid(true, "<Fixed Frame>", 20, 1.0, QColor(160, 160, 164));
    enableRobotModel(true, "robot_description");
    enableGoalArrowMarker(true, "/robot_upper_control/goal_marker");
    enableMap(true, "/map");
    enableLaserScan(true, "/scan", QColor(0, 255, 0));
    enablePoseArray(true, "/particlecloud", QColor(0, 192, 0));
    enableGoalPose(true, "/move_base_simple/goal", QColor(255, 25, 0));
    enableGlobalCostmap(true, "/move_base/global_costmap/costmap");
    enableLocalCostmap(true, "/move_base/local_costmap/costmap");
    enableGlobalPlanner(true, "/move_base/TrajectoryPlannerROS/global_plan", QColor(255, 0, 0));
    enableLocalPlanner(true, "/move_base/TrajectoryPlannerROS/local_plan", QColor(255, 255, 0));

    // 设置工具
    if (tool_manager_) {
      tool_manager_->removeAll();
      tool_manager_->addTool("rviz/Interact");
      tool_manager_->addTool("rviz/SetInitialPose");
      tool_manager_->addTool("rviz/SetGoal");
      tool_manager_->addTool("robot_upper_plugins/RouteGoalTool");
      tool_manager_->addTool("robot_upper_plugins/ObstacleAddTool");
      tool_manager_->addTool("robot_upper_plugins/ObstacleDelTool");
    }
  }

  ~QRenderPanel()
  {
    delete render_panel_;
    delete visualization_manager_;
    delete tool_manager_;
    delete grid_;
    delete robot_model_;
    delete goal_arrow_marker_;
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

  void setGlobalOptions(QString frameName,  QColor backgroundColor, int frameRate)
  {
    visualization_manager_->setFixedFrame(frameName);
    visualization_manager_->setProperty("Background Color", backgroundColor);
    visualization_manager_->setProperty("Frame Rate", frameRate);
  }

  void enableGrid(bool enable, QString referenceFrame, int planCellCount, float cellSize, QColor color=QColor(125, 125, 125))
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
    grid_->subProp("Color")->setValue(color);
    grid_->subProp("Reference Frame")->setValue(referenceFrame);
    grid_->subProp("Plane Cell Count")->setValue(planCellCount);
    grid_->subProp("Cell Size")->setValue(cellSize);
    grid_->subProp("Line Style")->setValue("Lines");
    grid_->subProp("Alpha")->setValue(0.5);
    grid_->subProp("Plane")->setValue("XY");
    grid_->subProp("Normal Cell Count")->setValue("0");
    grid_->setEnabled(enable);
  }

  void enableRobotModel(bool enable, QString robotDescription)
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
    robot_model_->subProp("Robot Description")->setValue(robotDescription);
    robot_model_->setEnabled(enable);
  }

  void enableGoalArrowMarker(bool enable, QString markerTopic)
  {
    if(!enable && goal_arrow_marker_)
    {
      goal_arrow_marker_->setEnabled(false);
      return ;
    }

    if (goal_arrow_marker_)
      delete goal_arrow_marker_;

    goal_arrow_marker_ = visualization_manager_->createDisplay("rviz/Marker", "GoalArrowMarker", true);
    ROS_ASSERT(goal_arrow_marker_ != nullptr);
    goal_arrow_marker_->subProp("Marker Topic")->setValue(markerTopic);
    goal_arrow_marker_->setEnabled(enable);
  }

  void enableMap(bool enable, QString topic)
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
    map_->subProp("Topic")->setValue(topic);
    map_->subProp("Alpha")->setValue("0.7");
    map_->subProp("Color Scheme")->setValue("map");
    map_->setEnabled(enable);
  }

  void enableLaserScan(bool enable, QString topic, QColor color)
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
    laser_scan_->subProp("Topic")->setValue(topic);
    laser_scan_->subProp("Color")->setValue(color);
    laser_scan_->subProp("Style")->setValue("Flat Squares");
    laser_scan_->subProp("Size (m)")->setValue("0.03");
    laser_scan_->subProp("Color Transformer")->setValue("FlatColor");  // how to set color?
    laser_scan_->subProp("Queue Size")->setValue("10");
    laser_scan_->subProp("Alpha")->setValue("1");
    laser_scan_->subProp("Position Transformer")->setValue("XYZ");
    laser_scan_->setEnabled(enable);
  }

  void enablePoseArray(bool enable, QString Topic, QColor Color)
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
    pose_array_->subProp("Color")->setValue(Color);
    pose_array_->subProp("Shape")->setValue("Arrow(Flat)");
    pose_array_->subProp("Queue Size")->setValue("10");
    pose_array_->subProp("Alpha")->setValue(1);
    pose_array_->subProp("Arrow Length")->setValue(0.05);
    pose_array_->setEnabled(enable);
  }

  void enableGoalPose(bool enable, QString Topic, QColor color)
  {
    if(!enable && goal_pose_)
    {
      goal_pose_->setEnabled(false);
      return ;
    }

    if (goal_pose_)
      delete goal_pose_;

    goal_pose_ = visualization_manager_->createDisplay("rviz/Pose", "GoalMarker", true);
    ROS_ASSERT(goal_pose_ != nullptr);
    goal_pose_->subProp("Topic")->setValue(Topic);
    goal_pose_->subProp("Color")->setValue(color);
    goal_pose_->subProp("Queue Size")->setValue("10");
    goal_pose_->subProp("Shape")->setValue("Arrow");
    goal_pose_->subProp("Shaft Length")->setValue("0.5");
    goal_pose_->subProp("Shaft Radius")->setValue("0.05");
    goal_pose_->subProp("Head Length")->setValue("0.3");
    goal_pose_->subProp("Head Radius")->setValue("0.1");
    goal_pose_->setEnabled(enable);
  }

  void enableGlobalCostmap(bool enable, QString topic)
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
    global_map_->subProp("Topic")->setValue(topic);
    global_map_->subProp("Alpha")->setValue("0.7");
    global_map_->subProp("Color Scheme")->setValue("costmap");
    global_map_->setEnabled(enable);
  }

  void enableLocalCostmap(bool enable, QString Topic)
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
    local_map_->subProp("Alpha")->setValue("0.7");
    local_map_->subProp("Color Scheme")->setValue("costmap");
    local_map_->setEnabled(enable);
  }

  void enableGlobalPlanner(bool enable, QString Topic, QColor Color)
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
    global_path_->subProp("Queue Size")->setValue("10");
    global_path_->subProp("Alpha")->setValue("1");
    global_path_->subProp("Buffer Length")->setValue("1");
    global_path_->setEnabled(enable);
  }

  void enableLocalPlanner(bool enable, QString Topic, QColor Color)
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
    local_path_->subProp("Queue Size")->setValue("10");
    local_path_->subProp("Alpha")->setValue("1");
    local_path_->subProp("Buffer Length")->setValue("1");
    local_path_->subProp("Line Style")->setValue("Lines");
    local_path_->setEnabled(enable);
  }

  bool setCurrentTool(int toolIndex)
  {
    rviz::Tool* tool = tool_manager_->getTool(toolIndex);
    if (tool == current_tool_)
      return false;

    current_tool_ = tool;
    tool_manager_->setCurrentTool(tool);
    return true;
  }

  void setInteract()
  {
    // 设置当前使用的工具
    current_tool_ = tool_manager_->getTool(Interact);
    tool_manager_->setCurrentTool(current_tool_);
  }

  void setPos()
  {
    current_tool_ = tool_manager_->getTool(SetInitialPose);
    tool_manager_->setCurrentTool(current_tool_);
  }

  void setPointGoal()
  {
    current_tool_ = tool_manager_->getTool(SetGoal);
    tool_manager_->setCurrentTool(current_tool_);
    //设置goal的话题
    rviz::Property* pro= tool_manager_->getCurrentTool()->getPropertyContainer();
    pro->subProp("Topic")->setValue("/robot_upper_control/goal_temp");
    visualization_manager_->setFixedFrame("map");
  }

  void setRouteGoal()
  {
    if (!setCurrentTool(RouteGoalTool)) return;
    visualization_manager_->setFixedFrame("map");
  }

  void addObstacle()
  {
    if (!setCurrentTool(ObstacleAddTool)) return;
    visualization_manager_->setFixedFrame("map");

    MapSelectDialog *dialog = new MapSelectDialog();  // TODO: parent!
    dialog->setWindowTitle(tr("保存到地图"));
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    connect(dialog, SIGNAL(dialogClose(void)), this, SLOT(setDefaultTool(void)), Qt::UniqueConnection);
    connect(dialog, SIGNAL(confirm_signal(QString)), current_tool_, SLOT(sendObstacleAddRequest(QString)), Qt::UniqueConnection);
    connect(current_tool_, SIGNAL(toolDeactivated(void)), dialog, SLOT(closeDialog(void)), Qt::UniqueConnection);
    dialog->show();
  }

  void delObstacle()
  {
    if (!setCurrentTool(ObstacleDelTool)) return;
    visualization_manager_->setFixedFrame("map");

    MapSelectDialog *dialog = new MapSelectDialog();  // TODO: parent!
    dialog->setWindowTitle(tr("保存到地图"));
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    connect(dialog, SIGNAL(dialogClose(void)), this, SLOT(setDefaultTool(void)), Qt::UniqueConnection);
    connect(dialog, SIGNAL(confirm_signal(QString)), current_tool_, SLOT(sendObstacleDelRequest(QString)), Qt::UniqueConnection);
    connect(current_tool_, SIGNAL(toolDeactivated(void)), dialog, SLOT(closeDialog(void)), Qt::UniqueConnection);
    dialog->show();
  }

  void selectMap()
  {
    MapSelectDialog *dialog = new MapSelectDialog();
    dialog->setWindowTitle(tr("选择地图"));
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    connect(dialog, SIGNAL(dialogClose(void)), this, SLOT(setDefaultTool(void)), Qt::UniqueConnection);
    connect(dialog, &MapSelectDialog::confirm_signal, this, [&](QString mapName){
      std_msgs::String msg;
      msg.data = mapName.toStdString();
      map_req_pub.publish(msg);
    }, Qt::UniqueConnection);
    dialog->exec();
  }


public slots:

  void setDefaultTool(void)
  {
    current_tool_ = tool_manager_->getTool(Interact);
    tool_manager_->setCurrentTool(tool_manager_->getDefaultTool());
  }


private:

  rviz::RenderPanel* render_panel_;
  rviz::VisualizationManager* visualization_manager_;
  rviz::ToolManager* tool_manager_;
  rviz::Tool* current_tool_;
  rviz::Display* grid_;
  rviz::Display* robot_model_;
  rviz::Display* goal_arrow_marker_;
  rviz::Display* goal_pose_;
  rviz::Display* map_;
  rviz::Display* laser_scan_;
  rviz::Display* pose_array_;
  rviz::Display* global_map_;
  rviz::Display* local_map_;
  rviz::Display* global_path_;
  rviz::Display* local_path_;

  ros::NodeHandle* nh_;
  ros::Publisher map_req_pub;

  enum{
    Interact,
    SetInitialPose,
    SetGoal,
    RouteGoalTool,
    ObstacleAddTool,
    ObstacleDelTool
  };
};

} // end of rviz_panel

#endif // RENDER_PANEL_HPP
