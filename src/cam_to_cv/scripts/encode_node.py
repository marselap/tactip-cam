#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('cam_to_cv')
import sys
import numpy as np
import rospy
import cv2
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped
from cv_bridge import CvBridge, CvBridgeError
import sys, select
import os
import termios
import keras
from keras.models import Model, load_model
import itertools


class create_dataset_opto:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_topic", Image, self.callback_image, queue_size = 1)

        self.force_pub = rospy.Publisher("/force_tactip",Float64,queue_size=1)

        self.new_image = False

        encoderPath = '/home/marsela/catkin_ws/src/Optical-Based-Tactile-Sensor/src/cam_to_cv/scripts/5encoder.h5'
        self.encoderModel = load_model(encoderPath)
        estimatorPath = '/home/marsela/catkin_ws/src/Optical-Based-Tactile-Sensor/src/cam_to_cv/scripts/model-r-regr.h5'
        self.estimatorModel = load_model(estimatorPath)

    def callback_image(self, data):

        self.cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        self.new_image = True

    def encode(self):
        resize_size = 28
        data = np.empty((1, resize_size, resize_size), dtype=np.float32)
        # imgBW = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        img = cv2.resize(self.cv_image, (resize_size, resize_size))
        data[0, ...] = img
        data = data.astype('float32') / 255.
        data = np.round(data + 0.3)
        data.resize(1, resize_size, resize_size, 1)
        img_enc = self.encoderModel.predict(data)
        dmy = list(itertools.chain.from_iterable(img_enc))
        dmy = list(itertools.chain.from_iterable(dmy))
        img_enc_1d = list(itertools.chain.from_iterable(dmy))
        return img_enc_1d

    def predict(self):
        img_encoded = self.encode()
        force = self.estimatorModel.predict(np.array([img_encoded,]))
        return force

def main(args):
    ic = create_dataset_opto()
    rospy.init_node('create_dataset_opto', anonymous = True, disable_signals = True)
    try:
        while True:
            if ic.new_image:
                force = ic.predict()
                ic.new_image = False
                ic.force_pub.publish(force[0][0])
                print("-------------")
                print(force)
                print("-------------")

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
