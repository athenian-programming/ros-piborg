from threading import Condition

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class DirectImageSource(object):
    def __init__(self):
        self.__cond = Condition()
        self.__cv2_img = None
        self.__bridge = CvBridge()

    def start(self):
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.__image_cb)

    def stop(self):
        pass

    def __image_cb(self, msg):
        self.__cond.acquire()
        self.__cv2_img = self.__bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
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
