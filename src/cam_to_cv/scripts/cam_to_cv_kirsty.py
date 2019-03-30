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

def process_orig(cv_image):


    threshold=100 #220
    kernel_size=2 #4
    x_centar_kruga=-5 # 22
    y_centar_kruga=0 #11

    # cv2.imshow("cv_image", cv_image)
    (rows,cols,channels) = cv_image.shape

    rotationMatrix = cv2.getRotationMatrix2D((cols / 2, rows / 2), 28, 1) # ovdje 28, prije # 9, 1)
    frame = cv2.warpAffine(cv_image, rotationMatrix, (cols, rows))
    #frame = cv_image

    frame_gray_initial = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    frame_gray = cv2.equalizeHist(frame_gray_initial)  # histogram contrast

    ret, frame_binary = cv2.threshold(frame_gray, threshold, 255,cv2.THRESH_BINARY)

    mask_circle = np.zeros((rows, cols, 3), np.uint8)  # maska krug
    cv2.circle(mask_circle, (int(cols / 2) +y_centar_kruga, int(rows / 2) + x_centar_kruga), 145, (255, 255, 255), -1)
    mask_circle = cv2.cvtColor(mask_circle, cv2.COLOR_BGR2GRAY)

    masked = cv2.bitwise_and(frame_gray_initial, frame_binary, mask=mask_circle)

    ret, masked = cv2.threshold(masked, threshold, 255, cv2.THRESH_BINARY)


    kernel = np.ones((kernel_size, kernel_size), np.uint8)  # 3,3 za sample 4,4 za input1
    morph_open = cv2.morphologyEx(masked, cv2.MORPH_OPEN, kernel)  # dosta dobar

    x_start = 90 #114
    y_start = 170 #182
    cv_image = morph_open[x_start:x_start+300, y_start:y_start+300]

    return cv_image


def process(cv_image):


    (rows,cols,channels) = cv_image.shape

    scale = 2
    cv_image = cv2.resize(cv_image, (cols/scale, rows/scale))

    threshold=180
    kernel_size=2
    y_centar_kruga=-20/scale
    x_centar_kruga=10/scale
    imageSize = 28
    circle_mask_radius = 180/scale

    (rows,cols,channels) = cv_image.shape

    rotationMatrix = cv2.getRotationMatrix2D((cols / 2, rows / 2), -1, 1)
    frame = cv2.warpAffine(cv_image, rotationMatrix, (cols, rows))

    frame_gray_initial = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray = cv2.equalizeHist(frame_gray_initial)

    ret, frame_binary = cv2.threshold(frame_gray, threshold, 255,cv2.THRESH_BINARY)

    mask_circle = np.zeros((rows, cols, 3), np.uint8)
    cv2.circle(mask_circle, (int(cols / 2) +x_centar_kruga, int(rows / 2) +y_centar_kruga), circle_mask_radius, (255, 255, 255), -1)
    mask_circle = cv2.cvtColor(mask_circle, cv2.COLOR_BGR2GRAY)

    masked = cv2.bitwise_and(frame_gray_initial, frame_binary, mask=mask_circle)

    ret, masked = cv2.threshold(masked, threshold, 255, cv2.THRESH_BINARY)

    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    morph_open = cv2.morphologyEx(masked, cv2.MORPH_OPEN, kernel)

    x_start = 28 / scale#90
    y_start = 134/ scale#170
    morphopen_x = 400 / scale
    morphopen_y = 400 / scale
    cv_image = morph_open[x_start:x_start+morphopen_x, y_start:y_start+morphopen_x]
    cv_image = cv2.resize(cv_image, (300, 300))

    return cv_image

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic",Image,queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    self.time_prev = time.time()

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv_image = process_orig(cv_image)

    # print("aaaaaaa")
    # imageSize = 28
    # cv_image = cv2.resize(cv_image, (imageSize, imageSize))

    cv2.imshow("cv_image", cv_image)
    cv2.waitKey(10)



    try:
      # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
