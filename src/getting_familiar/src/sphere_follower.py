#!/usr/bin/env python

""" The script will be used to explore basics of ROS messages in Python """
from std_msgs.msg import ColorRGBA

from geometry_msgs.msg import Point, Pose, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import rospy
import tf

rospy.init_node('sphere_follow')

# Fixed Messages for the Marker
color_msg = ColorRGBA(r=.5, a=1) # color plus transparency
scale_msg = Vector3(x=1.0, y=1, z=1) # scale (x,y,z of sphere) of the Marker
pub = rospy.Publisher("/my_sphere", Marker, queue_size=10)

br = tf.TransformBroadcaster()
listener = tf.TransformListener()

r = rospy.Rate(10)
while not rospy.is_shutdown():
    header_msg = Header(stamp=rospy.Time.now(),
                        frame_id="base_link")
    point_msg = Point(x=1.0, y=0.0, z=0.0) # position specified by point
    pose_msg = Pose(position=point_msg) # pose specifices position/orientation
    msg = Marker(
          header=header_msg
        , pose=pose_msg
        , color=color_msg
        , scale=scale_msg
        )
    msg.type = msg.SPHERE
    pub.publish(msg)
    r.sleep()
