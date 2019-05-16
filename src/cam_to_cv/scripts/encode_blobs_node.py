#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('cam_to_cv')
import sys
import numpy as np
import rospy
import cv2
import matplotlib.pyplot as plt
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped, Wrench
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

        self.force_pub = rospy.Publisher("/force_tactip",Wrench,queue_size=1)

        self.new_image = False

        encoderPath = '/home/marsela/catkin_ws/src/tactip-cam/src/cam_to_cv/scripts/5encoder.h5'
        self.encoderModel = load_model(encoderPath)
        estimatorPath = '/home/marsela/catkin_ws/src/tactip-cam/src/cam_to_cv/scripts/tockice-focus.h5'
        self.estimatorModel = load_model(estimatorPath)

    def callback_image(self, data):

        self.cv_image = self.bridge.imgmsg_to_cv2(data)#, "mono8")
        self.new_image = True


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

    def preproc(self):
        # cv_image = process_2(self.cv_image)
        cv_image = self.cv_image
        success, centresX, centresY = blob_centres(cv_image)
        if success:

            blank_image = np.zeros((300,300,1), np.uint8)
            for (x,y) in zip(centresX, centresY):
                cv2.circle(blank_image, (x,y), 4, (255,255,255), -1)
            kernel_size = 4
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            img = cv2.morphologyEx(blank_image, cv2.MORPH_OPEN, kernel)

            # my_dpi = 166.
            # fig = plt.figure(figsize=(300./my_dpi,300./my_dpi),frameon=False)
            # ax = plt.Axes(fig, [0., 0., 1., 1.])
            # ax.set_axis_off()
            # fig.add_axes(ax)
            # plt.scatter(centresX, centresY, c='black')
            # fig.subplots_adjust(bottom=0.,left=0.,right=1.,top=1.)
            # # plt.show()
            #
            # fig.canvas.draw()
            # img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
            # print(fig.canvas.get_width_height()[::-1] + (3,))
            # img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            # img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            return True, img
        else:
            return False, []

    def encode(self, cv_image):
        resize_size = 28
        data = np.empty((1, resize_size, resize_size), dtype=np.float32)
        # imgBW = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        img = cv2.resize(cv_image, (resize_size, resize_size))
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
        success, img = self.preproc()

        if success:
            cv2.imshow("img", img)
            cv2.waitKey(10)
            img_encoded = self.encode(img)
            force = self.estimatorModel.predict(np.array([img_encoded,]))
            return True, force
        else:
            return False, []



def blob_centres(cv_image):
    detector = cv2.SimpleBlobDetector_create()
    cnts = cv2.findContours(cv_image, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cv_image, cnts[1], -1, (255, 255, 255), -1)
    im = cv2.bitwise_not(cv_image)
    cv2.drawContours(im, cnts[1], -1, (255, 255, 255), -1)
    centresX = []
    centresY = []
    success = True
    for (i,c) in enumerate(cnts[1]):
        M = cv2.moments(c)
        # print("++++++++++++++++++++++++++++++++")
        # print(M)
        # print("++++++++++++++++++++++++++++++++")
        try:
            Cx= int(M["m10"] / M["m00"])
            Cy = int(M["m01"] / M["m00"])
            centresX.append(Cx)
            centresY.append(Cy)
        except:
            # success = False
            pass
    centresX = [centresX[i] for i in np.argsort(centresY)]
    centresY = [centresY[i] for i in np.argsort(centresY)]
    return success, centresX, centresY

def createForceMsg(force):
    force_msg = Wrench()
    i = 0
    force_msg.force.x = force[i]
    i += 1
    force_msg.force.y = force[i]
    i += 1
    force_msg.force.z = force[i]
    i += 1
    force_msg.torque.x = force[i]
    i += 1
    force_msg.torque.y = force[i]
    i += 1
    force_msg.torque.z = force[i]
    return force_msg

def main(args):
    ic = create_dataset_opto()
    rospy.init_node('create_dataset_opto', anonymous = True, disable_signals = True)
    min_y = [0.0, 0.0, 0.0, -0.93527323, -0.17835155, -0.36657366]
    max_y = [3.628354, 2.8112476, 8.98534, 0.0, 0.5463106, 0.024878157]

    try:
        while True:
            if ic.new_image:
                success, force = ic.predict()
                ic.new_image = False
                if success:
                    force = force[0]
                    # for i in xrange(6):
                    force = [y * (max_y[i] - min_y[i]) + min_y[i] for i,y in enumerate(force)]
                    force_msg = createForceMsg(force)
                    ic.force_pub.publish(force_msg)
                print("-------------")
                print(force)
                print("-------------")

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
