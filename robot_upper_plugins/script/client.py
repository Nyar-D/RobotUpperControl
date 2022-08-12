#! /usr/bin/env python

#1.导包
import rospy
from robot_upper_plugins.srv import *
from geometry_msgs.msg import Point


if __name__ == "__main__":
    rospy.init_node("AddInts_Client_p")
    client = rospy.ServiceProxy("MapAddObstacle", MapAddObstacle)
    client.wait_for_service()
    
    p = Point()
    p.x = 1.0
    p.y = 2.0
    p.z = 3.0
    req = MapAddObstacleRequest()
    req.points.append(p)
    req.gen_map_name = "gen.pgm"    

    #优化
    resp = client.call(req)
    rospy.loginfo(resp.status)

