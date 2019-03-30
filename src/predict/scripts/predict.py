#!/usr/bin/env python
import rospy
import cv2
import roslib
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from keras.preprocessing import image
from keras.applications.resnet50 import ResNet50, preprocess_input, decode_predictions
from keras.models import load_model
from numpy import argmax
from keras.preprocessing.image import img_to_array



cv_image = np.zeros((300,300,1), np.uint8)

class predict():
        def print_screen(self,data):
            rospy.loginfo(data)
            rospy.sleep(0.2)

        def callback(self,data):
            try:
                global cv_image
                cv_image = self.bridge.imgmsg_to_cv2(data, 'mono8')
            except CvBridgeError as e:
                print(e)
            self.neuron()
            
        def neuron(self):
            global cv_image
            oblik=["KvadarV10A9B27","Polozena3StranaPrizmaV10A16B27","ValjakR4V10","ValjakR8V10"]
            #cv2.imshow("cv_image_1, cv_image)
            cv2.waitKey(1)
            cv_image = cv2.resize(cv_image, (50,50))
            cv_image = cv_image.astype("float") / 255.0
            cv_image = img_to_array(cv_image)
            cv_image = np.expand_dims(cv_image, axis=0)
            
            global graph
            with graph.as_default(): 
                prediction_c = model_c.predict(cv_image)
                inverted = argmax(prediction_c[0])
                obl = oblik[inverted]
                if (obl == oblik[0]):
                    prediction_h = model_h_K10.predict(cv_image)
                elif ( obl == oblik[1]):
                    prediction_h = model_h_P10.predict(cv_image)
                elif ( obl == oblik[2]):
                    prediction_h = model_h_V4.predict(cv_image)
                elif ( obl == oblik[3]):
                    prediction_h = model_h_V8.predict(cv_image)


                self.msg_string.data = (oblik[inverted] + ' Udubljenje ' + str(round((prediction_h[0][0])*19,2)))
                self.pub.publish(self.msg_string)
                self.print_screen(self.msg_string)                    



        def __init__(self):
            self.pub = rospy.Publisher('object_detected', String, queue_size = 1)
            self.bridge = CvBridge()
            self.msg_string = String() 
            self.image_sub = rospy.Subscriber("image_topic_2", Image, self.callback, queue_size = 1, buff_size = 2**24)

        def run(self):
            while not rospy.is_shutdown():
                rospy.spin()

if __name__=='__main__':
    rospy.init_node('predict', anonymous=True)
    model_h_V8 = load_model('model_h_V8.h5')
    model_h_V4 = load_model('model_h_V4.h5')
    model_h_P10 = load_model('model_h_P10.h5')
    model_h_K10 = load_model('model_h_K10.h5')
    model_c = load_model('predict_class.h5')    

    graph = tf.get_default_graph()
    try:
        ne = predict()
        ne.run()
    except rospy.ROSInterruptException:
        pass
