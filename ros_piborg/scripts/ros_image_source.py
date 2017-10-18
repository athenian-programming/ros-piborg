from threading import Condition

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class RosImageSource(object):
    def __init__(self, topic, compressed, format):
        self.__topic = topic
        self.__compressed = compressed
        self.__format = format

        self.__cond = Condition()
        self.__cv2_img = None
        self.__bridge = CvBridge()

        rospy.init_node('ros_image_source')

    def start(self):
        rospy.Subscriber(self.__topic,
                         CompressedImage if self.__compressed else Image,
                         self.__image_cb)

    def stop(self):
        pass

    def __image_cb(self, msg):
        self.__cond.acquire()
        if self.__compressed:
            self.__cv2_img = self.__bridge.compressed_imgmsg_to_cv2(msg, self.__format)
        else:
            self.__cv2_img = self.__bridge.imgmsg_to_cv2(msg, self.__format)
        self.__cond.notify()
        self.__cond.release()

    def get_image(self):
        self.__cond.acquire()
        while self.__cv2_img is None:
            self.__cond.wait()
        retval = self.__cv2_img
        self.__cv2_img = None
        self.__cond.release()
        return retval