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
from geometry_msgs.msg import WrenchStamped
from cv_bridge import CvBridge, CvBridgeError
import sys, select
import os
import termios

class create_dataset_opto:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_topic", Image, self.callback_image, queue_size = 1)
        self.opto_sub = rospy.Subscriber("/OptoForceWrench_raw", WrenchStamped, self.callback_opto, queue_size = 1)

        self.force = 0.0
        self.targets = {}
        self.target_timestamps = {}

    def callback_image(self, data):

        self.cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")

    def callback_opto(self, data):
        self.force = data.wrench.force.z

def wait_key():
    ''' Wait for a key press on the console and return it. '''
    result = None

    fd = sys.stdin.fileno()

    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    try:
        result = sys.stdin.read(1)
    except IOError:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)

    return result

def main(args):
    ic = create_dataset_opto()
    rospy.init_node('create_dataset_opto', anonymous = True, disable_signals = True)
    try:
        while not rospy.is_shutdown():
            # should be: check flag from goal_achieved topic
            try:
                force = int(ic.force * 100.00)
                if force in ic.targets:
                    ic.targets[force] += 1
                else:
                    ic.targets[force] = 1
                filename = 'force' + str(force) + '_' + str(ic.targets[force]) + '.bmp'
                img_path = os.path.join(os.path.expanduser('~'), 'datasets', 'forces', filename)
                cv2.imwrite(img_path, ic.cv_image)
            except CvBridgeError as e:
                print(e)
            except AttributeError:
                print("No image available from the camera")
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
