#!/usr/bin/env python

import rospy
import rospkg
import cv2
import os

from cv_bridge import CvBridge
from runtime_manager.msg import traffic_light
from sensor_msgs.msg import Image


class TrafficLightViz:
  def __init__(self):
    rospy.init_node('traffic_light_viz')
    self.path = rospkg.RosPack().get_path('traffic_light_classifier')
    self.traffic_viz_publisher = rospy.Publisher('traffic_light_viz', Image, queue_size=1)
    self.green_image = cv2.imread(os.path.join(self.path, 'light_images', 'green.jpg'))
    self.red_image = cv2.imread(os.path.join(self.path, 'light_images', 'red.jpg'))
    self.unknown_image = cv2.imread(os.path.join(self.path, 'light_images', 'unknown.jpg'))
    self.cv_bridge = CvBridge()
    rospy.Subscriber('light_color', traffic_light, self.traffic_light_callback)

  def traffic_light_callback(self, traffic_light):
    if(traffic_light.traffic_light == 0):
      self.traffic_viz_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.red_image, 'bgr8'))
    elif(traffic_light.traffic_light == 1):
      self.traffic_viz_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.green_image, 'bgr8'))
    else:
      self.traffic_viz_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.unknown_image, 'bgr8'))

if __name__ == '__main__':
  TrafficLightViz()
  rospy.spin()
