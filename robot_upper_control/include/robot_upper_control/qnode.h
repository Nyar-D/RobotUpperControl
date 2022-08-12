#ifndef QNODE_H
#define QNODE_H

#include <ros/ros.h>
#include <QStringListModel>
#include <QTimer>
#include <string>
#include <QDebug>
#include <QMetaType>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_datatypes.h>

class QNode : public QObject  {
  Q_OBJECT

public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  void init();
  void init(const std::string &master_url, const std::string &host_url);

private:
  int init_argc;
  char** init_argv;
};

#endif /* QNODE_H */
