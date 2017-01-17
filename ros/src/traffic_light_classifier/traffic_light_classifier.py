#!/usr/bin/env python

import PIL
import sys
from PIL import Image

import message_filters
import numpy
import rospy
from cv_bridge import CvBridge
from keras.models import load_model
from keras.preprocessing.image import img_to_array
from road_wizard.msg import Signals
from runtime_manager.msg import traffic_light
from sensor_msgs.msg import Image


class TrafficClassifier:
  RED = 0
  GREEN = 1
  UNKNOWN = 2
  RADIUS_MULTIPLIER = 6

  model = None
  last_published_prediction = UNKNOWN

  def __init__(self):
    rospy.init_node('traffic_light_classifier')

    roi_signal = message_filters.Subscriber('roi_signal', Signals)
    camera_image = message_filters.Subscriber('image_raw', Image)
    self.light_detected_publisher = rospy.Publisher('light_color', traffic_light, queue_size=1)
    self.roi_image = rospy.Publisher('roi_image', Image, queue_size=1)

    time_sync = message_filters.ApproximateTimeSynchronizer([roi_signal, camera_image], 5, .1)
    time_sync.registerCallback(self.detect_signal)

  def calculate_bounds(self, signal):
    xmin = sys.maxint
    xmax = -sys.maxint - 1
    ymin = sys.maxint
    ymax = -sys.maxint - 1
    radius_max = 0

    for signal in signal.Signals:
      x = signal.u
      y = signal.v
      radius_max = max(radius_max, signal.radius)
      xmin = min(xmin, x)
      xmax = max(xmax, x)
      ymin = min(ymin, y)
      ymax = max(ymax, y)

    return int(xmin - self.RADIUS_MULTIPLIER * radius_max), int(xmax + self.RADIUS_MULTIPLIER * radius_max), int(
      ymin - self.RADIUS_MULTIPLIER * radius_max), int(ymax + self.RADIUS_MULTIPLIER * radius_max)

  @staticmethod
  def crop_image(image, xmin, xmax, ymin, ymax):
    return image.crop((xmin, ymin, xmax, ymax))

  def predict_light(self, cropped_roi):
    # Load CNN Model
    loaded_model = self.get_model()
    image_array = img_to_array(cropped_roi.resize((64, 64), PIL.Image.ANTIALIAS))
    prediction = loaded_model.predict(image_array[None, :])
    if prediction[0][0] == 1:
      return self.GREEN
    elif prediction[0][1] == 1:
      return self.RED
    else:
      return self.UNKNOWN

  def get_model(self):
    # TODO Fix: Must load model from ROS callback thread
    if not self.model:
      self.model = load_model('light_classifier_model.h5')
    return self.model

  def detect_signal(self, signal, image):
    if len(signal.Signals) == 0:
      # No signals are visible
      self.publish_prediction(self.UNKNOWN)
      return

    # Convert the image to PIL
    cv_bridge = CvBridge()
    cv_image = cv_bridge.imgmsg_to_cv2(image, "rgb8")
    image = PIL.Image.fromarray(cv_image)

    # Find the bounds of the signal
    xmin, xmax, ymin, ymax = self.calculate_bounds(signal)

    # Crop the image for the ROI
    cropped_roi = self.crop_image(image, xmin, xmax, ymin, ymax)

    self.roi_image.publish(cv_bridge.cv2_to_imgmsg(numpy.array(cropped_roi), "rgb8"))
    # Run the cropped image through the NN
    prediction = self.predict_light(cropped_roi)

    self.publish_prediction(prediction)

  def publish_prediction(self, prediction):
    # Publish the prediction
    if prediction != self.last_published_prediction:
      self.light_detected_publisher.publish(traffic_light(traffic_light=prediction))
      self.last_published_prediction = prediction


if __name__ == '__main__':
  try:
    TrafficClassifier()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
