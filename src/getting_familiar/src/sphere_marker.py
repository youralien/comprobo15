#!/usr/bin/env python

""" The script will be used to explore basics of ROS messages in Python """
from std_msgs.msg import ColorRGBA

from geometry_msgs.msg import Point, Pose, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import rospy

rospy.init_node('test_marker')

SPHERE = 2
point_msg = Point(x=1.0, y=2.0, z=0.0) # position specified by point
pose_msg = Pose(position=point_msg) # pose specifices position/orientation
color_msg = ColorRGBA(r=.5, a=1) # color plus transparency
scale_msg = Vector3(x=1.0, y=1, z=1) # scale (x,y,z of sphere) of the Marker
header_msg = Header(stamp=rospy.Time.now(),
		    frame_id="odom")
msg = Marker(
      header=header_msg
    , pose=pose_msg
    , color=color_msg
    , scale=scale_msg
    )
msg.type = SPHERE
pub = rospy.Publisher("/my_sphere", Marker, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(msg)
    r.sleep()
