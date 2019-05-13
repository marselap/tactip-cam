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
import time
import glob


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic",Image,queue_size=1)

    self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    self.time_prev = time.time()

    self.paths = []
    data_folder_s = ['cetvrti_'+str(i+5) for i in xrange(4)]
    for data_folder in data_folder_s:
        self.paths.append('/home/marsela/algorithms/franka_tactip/node_test/' + data_folder + '/')


    self.path_i = 0
    self.id_i = 0

  def load_new_image(self):
      path = self.paths[self.path_i]
      inputpath = np.sort(glob.glob(path + 'frame_*'))
      # print("++___________________________++")
      # print(path)
      # print(inputpath)
      img = cv2.imread(inputpath[self.id_i])

      if self.id_i < len(inputpath) - 1:
          self.id_i += 1
      else:
          self.id_i = 0
          self.path_i += 1

      return img

def process_2(cv_image):
    threshold=170
    kernel_size=2
    x_centar_kruga=27
    y_centar_kruga=12

    imageSize = 28
    #cv2.imshow("cv_image", cv_image)
    (rows,cols,channels) = cv_image.shape

    rotationMatrix = cv2.getRotationMatrix2D((cols / 2, rows / 2), 0, 1)  # rotacija
    frame = cv2.warpAffine(cv_image, rotationMatrix, (cols, rows))
    frame_gray_initial = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    frame_gray = cv2.equalizeHist(frame_gray_initial)  # histogram contrast
    ret, frame_binary = cv2.threshold(frame_gray, threshold, 255,cv2.THRESH_BINARY)
    mask_circle = np.zeros((rows, cols, 3), np.uint8)  # maska krug
    cv2.circle(mask_circle, (int(cols / 2) +y_centar_kruga, int(rows / 2) + x_centar_kruga), 145, (255, 255, 255), -1)
    mask_circle = cv2.cvtColor(mask_circle, cv2.COLOR_BGR2GRAY)
    mask = cv2.bitwise_and(frame_gray_initial, frame_binary, mask=mask_circle)
    kernel = np.ones((kernel_size, kernel_size), np.uint8)  # 3,3 za sample 4,4 za input1
    morph_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # dosta dobar
    x_start = 130 #114
    y_start = 160 #182
    cv_image = morph_open[x_start:x_start+300, y_start:y_start+300]
    return cv_image


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
      while True:
          cv_image = ic.load_new_image()
          cv_image = process_2(cv_image)
          image_message = ic.bridge.cv2_to_imgmsg(cv_image)#, encoding="passthrough")
          ic.image_pub.publish(image_message)
          time.sleep(1.0)
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
