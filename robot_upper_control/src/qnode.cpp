#include "qnode.h"

QNode::QNode(int argc, char** argv) :
  init_argc(argc),
  init_argv(argv)
{
}

QNode::~QNode()
{
  if(ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }

  delete init_argv;
}

void QNode::init()
{
  setlocale(LC_ALL, "");

  // 初始化ros节点
  ros::init(init_argc, init_argv, "robot_upper_control");
  while(!ros::master::check()) {
    qDebug() << "等待主节点启动...";
    sleep(1);
  }
}

void QNode::init(const std::string &master_url, const std::string &host_url)
{
  setlocale(LC_ALL, "");

  // 初始化ros节点
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "robot_upper_control");
  while(!ros::master::check()) {
    qDebug() << "等待主节点启动...";
    sleep(1);
  }
}
