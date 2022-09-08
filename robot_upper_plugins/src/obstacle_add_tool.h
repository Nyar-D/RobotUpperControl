#ifndef OBSTACLE_ADD_TOOL_H
#define OBSTACLE_ADD_TOOL_H

#include <ros/ros.h>
#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/load_resource.h>
#include <OgreVector3.h>

namespace robot_upper_plugins
{

class ObstacleAddTool : public rviz::Tool {
  Q_OBJECT

public:
  ObstacleAddTool();
  ~ObstacleAddTool();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

Q_SIGNALS:
  void toolDeactivated(void);

private Q_SLOTS:
  void sendObstacleAddRequest(QString genMapName);

private:
  void addNewLine();
  void destoryAllPoints();

private:
  enum {
    START,
    END
  } state_;

  typedef std::set<std::pair<double, double>> points;
  QList<points*> points_list_;  // 储存每条线上的点
  rviz::BillboardLine* obstacle_line_;
  unsigned int current_line_ = -1;

  QCursor std_cursor_;
  QCursor hit_cursor_;

  ros::NodeHandle* nh_;
  ros::ServiceClient aobs_client_;
};

} // end namespace robot_upper_plugins


#endif // OBSTACLE_ADD_TOOL_H
