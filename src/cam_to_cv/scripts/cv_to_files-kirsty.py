#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('cam_to_cv')
import sys
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys, select
import os
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped
import csv


class create_dataset:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image_topic",Image,self.callback, queue_size = 1)
    self.trigger_sub = rospy.Subscriber("/image_trigger", Bool, self.trigger_callback, queue_size = 1)

    self.force_sub = rospy.Subscriber("/netft_data", WrenchStamped, self.force_callback, queue_size = 1)

    self.noSaved = 0
    self.r = "-"
    self.theta = "0"
    self.flag_save = False

    self.force = [0.0, 0.0, 0.0]

    self.csv_forces = os.path.join(os.path.expanduser('~'), 'datasets', 'kirsty', 'forces.csv')

    with open(self.csv_forces, mode='w') as csv_file:
        fieldnames = ['filename', 'force_x', 'force_y', 'force_z']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

  def force_callback(self, data):
      self.force[0] = data.wrench.force.x
      self.force[1] = data.wrench.force.y
      self.force[2] = data.wrench.force.z

  def trigger_callback(self, data):
    self.flag_save = data.data
    print(self.flag_save)

  def callback(self,data):

    cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    if self.flag_save == True:
        print("save image")
        tempDict = {}
        tempDict['filename'] = 'aaaa'
        tempDict['force_x'] = self.force[0]
        tempDict['force_y'] = self.force[1]
        tempDict['force_z'] = self.force[2]
        with open(self.csv_forces, mode='a') as csv_file:
            fieldnames = ['filename', 'force_x', 'force_y', 'force_z']
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writerow(tempDict)

        self.noSaved += 1
        filename = 'bla_' + str(self.noSaved) + '.bmp'
        self.img_path = os.path.join(os.path.expanduser('~'), 'datasets', 'kirsty', filename)
        cv2.imwrite(self.img_path , cv_image)
        self.flag_save = False




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
