#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('cam_to_cv')
import sys
import numpy as np
import rospy
import cv2
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time


class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image, self.videoCallback)
    self.start_recording = rospy.Subscriber("/record_start", Bool, self.startCallback)
    self.stop_recording = rospy.Subscriber("/record_stop", Bool, self.stopCallback)

    self.touchId = rospy.Subscriber("/touch_id", String, self.touchIdCallback)

    self.time_prev = time.time()
    self.start_flag = False
    self.stop_flag = True

    self.new_recording = False

  def touchIdCallback(self, data):
      self.touchId = data.data

  def startCallback(self, data):
      self.start_flag = True
      self.stop_flag = False

  def stopCallback(self, data):
      self.stop_flag = True

  def videoCallback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.new_recording = True
    except CvBridgeError as e:
      print(e)


def main(args):
    ic = image_converter()
    rospy.init_node('save_video', anonymous=True)

    frame_array = []

    try:
        while True:
            if ic.start_flag:
                if ic.stop_flag:
                    outFolder = "/home/marsela/datasets/force_all/"
                    height, width, layers = ic.cv_image.shape
                    size = (width,height)
                    pathOut = outFolder + "video_" + ic.touchId + ".avi"
                    out = cv2.VideoWriter(pathOut,cv2.VideoWriter_fourcc(*'DIVX'), 30.0, size)
                    for frame in frame_array:
                        out.write(frame)
                    out.release()
                    ic.start_flag = False
                    frame_array = []
                else:
                    if ic.new_recording:
                        frame_array.append(ic.cv_image)
                        ic.new_recording = False

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
