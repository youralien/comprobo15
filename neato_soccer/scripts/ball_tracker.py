#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        # image params
        self.GREEN_MIN = np.array([60,76,2])
        self.GREEN_MAX = np.array([80,255,255])
        self.center_view_percentage = 0.50 

        # control params
        self.forward_vel = 0.25
        self.angular_k = 1.0
        self.twist = None 

        # bump counter
        self.bumps = 0
        self.bump_last = False
        self.bump_time_threshold = 3                          # seconds
        self.current_bump_time = 0

        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')
        
        # when mouse hovers over video window
        # cv2.setMouseCallback('video_window', self.process_mouse_event)

        # setup_slider()

    def setup_slider(self):
        # sliders
        cv2.namedWindow('threshold_image')
        self.hsv_lb = np.array([0, 0, 0])
        cv2.createTrackbar('H lb', 'threshold_image', 0, 255, self.set_h_lb)
        cv2.createTrackbar('S lb', 'threshold_image', 0, 255, self.set_s_lb)
        cv2.createTrackbar('V lb', 'threshold_image', 0, 255, self.set_v_lb)
        self.hsv_ub = np.array([255, 255, 255])
        cv2.createTrackbar('H ub', 'threshold_image', 0, 255, self.set_h_ub)
        cv2.createTrackbar('S ub', 'threshold_image', 0, 255, self.set_s_ub)
        cv2.createTrackbar('V ub', 'threshold_image', 0, 255, self.set_v_ub)

    def set_h_lb(self, val):
        self.hsv_lb[0] = val

    def set_s_lb(self, val):
        self.hsv_lb[1] = val

    def set_v_lb(self, val):
        self.hsv_lb[2] = val

    def set_h_ub(self, val):
        self.hsv_ub[0] = val

    def set_s_ub(self, val):
        self.hsv_ub[1] = val

    def set_v_ub(self, val):
        self.hsv_ub[2] = val

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        # binarize based on the slider values
        # self.binarized_image = cv2.inRange(self.hsv_image, self.hsv_lb, self.hsv_ub)

        # binarize based on preset values
        self.binarized_image = cv2.inRange(self.hsv_image, self.GREEN_MIN, self.GREEN_MAX)

        # State 1: Ball is in Frame
        if self.find_object_center(self.binarized_image):
            image_width = self.cv_image.shape[1]
            horizontal_midpoint = image_width / 2
            left_from_center = horizontal_midpoint - image_width * self.center_view_percentage / 2
            right_from_center = horizontal_midpoint + image_width * self.center_view_percentage / 2
            
            # positive z = positive theta = left, and vice versa
            off_centered_vector = horizontal_midpoint - self.center_x
            
            if (self.center_x > left_from_center) and (self.center_x < right_from_center):
                # print "Centered!"
                
                # Move forward, while turning proportionally
                self.twist = Twist(
                    linear=Vector3(x=self.forward_vel),
                    angular=Vector3(z=self.angular_k*off_centered_vector/image_width)
                )
            else:
                # print "Sides..."

                # Turn proportionally, while staying still
                self.twist = Twist(
                    linear=Vector3(x=0.0),
                    angular=Vector3(z=self.angular_k*off_centered_vector/image_width)
                )

        # State2: Ball is not in frame, so stop and search? 
        # TODO: ...

        cv2.imshow('video_window', self.hsv_image)
        cv2.waitKey(5)

    def find_object_center(self, binary_image):
        moments = cv2.moments(binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
            return True
        return False

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        image_info_window = 255*np.ones((500,500,3))
        
        # show hsv values
        cv2.putText(image_info_window,
                    'Color (h=%d,s=%d,v=%d)' % (self.hsv_image[y,x,0], self.hsv_image[y,x,1], self.hsv_image[y,x,2]),
                    (5,50), # 5 = x, 50 = y
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))

        # show bgr values 
        cv2.putText(image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)    

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.twist:
                self.pub.publish(self.twist)            
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()