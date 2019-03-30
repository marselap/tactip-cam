#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('cam_to_cv')
import sys
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys, select
import os

class create_dataset:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image_topic",Image,self.callback, queue_size = 1)

    self.targets = {}
    self.r = "-"
    self.theta = "0"

  def callback(self,data):

    if self.r == '-':
        print("current r: " + self.r)
        print("current theta: " + self.theta)
        try_r_theta = raw_input("r, theta? ")
        if try_r_theta:
            input_list = [x.strip() for x in try_r_theta.split(',')]
            if input_list[0]:
                self.r = input_list[0]
            if input_list[1]:
                self.theta = input_list[1]

        touchClass = 'r' + self.r + 'theta' + self.theta
        if touchClass in self.targets:
            self.targets[touchClass] += 1
        else:
            self.targets[touchClass] = 1

        filename = touchClass + '_' + str(self.targets[touchClass]) + '.bmp'
        self.img_path = os.path.join(os.path.expanduser('~'), 'datasets', 'rtheta', filename)
    else:
        cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        cv2.imwrite(self.img_path , cv_image)
        self.r = '-'

def main(args):
  ic = create_dataset()
  rospy.init_node('create_dataset', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
