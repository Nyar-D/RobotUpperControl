#ifndef ROUTE_GOAL_TOOL_H
#define ROUTE_GOAL_TOOL_H

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/load_resource.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <QDebug>
#include <cmath>
#include <sstream>

namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class BillboardLine;
}

namespace robot_upper_plugins
{

class RouteGoalTool : public rviz::Tool {
  Q_OBJECT

public:
  RouteGoalTool();
  ~RouteGoalTool();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;
  void setPointIntervel(int point_intervel);
  void setDistanceIntervel(double distance_intervel);

private:
  void publishRouteGoal(Ogre::Vector3& pos, Ogre::Vector3& last_pos);
  void publishRouteGoal(double x, double y, double theta);
  void publishByPointInterval(Ogre::Vector3& pos, Ogre::Vector3& last_pos);
  void publishByDistanceInterval(Ogre::Vector3& pos, Ogre::Vector3& last_pos);
  void publishMarkerDelete();
  void markerCallback(const visualization_msgs::Marker::ConstPtr& marker_delete);

private:
  enum {
    START,
    END
  } state_;

  rviz::BillboardLine* goal_line_;
  Ogre::Vector3 last_pos_, last_last_pos_;
  float distance_intervel_, distance_;
  unsigned int point_intervel_, point_count_;

  QCursor std_cursor_;
  QCursor hit_cursor_;

  ros::NodeHandle* nh_;
  ros::Publisher route_goal_pub_, marker_delete_pub_;
  ros::Subscriber goal_cancel_sub_;
};

} // end namespace robot_upper_plugins

#endif // ROUTE_GOAL_TOOL_H
