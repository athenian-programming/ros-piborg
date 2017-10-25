from threading import Condition
from threading import Thread

import cv2
import imutils
import rospy

import cli_args  as cli


class VideoImageSource(object):
    args = [cli.filename]

    def __init__(self, filename, fps_rate=30):
        self.__cond = Condition()
        self.__rate = rospy.Rate(fps_rate)
        self.__cv2_img = None
        self.__video = cv2.VideoCapture(filename)

    def start(self):
        Thread(target=self.__read_image).start()

    def stop(self):
        pass

    def __read_image(self):
        while True:
            self.__cond.acquire()

            ret, self.__cv2_img = self.__video.read()
            if not ret:
                break
            self.__cv2_img = imutils.resize(self.__cv2_img, width=600)
            self.__cond.notify()
            self.__cond.release()
            self.__rate.sleep()

    def get_image(self):
        self.__cond.acquire()
        while self.__cv2_img is None:
            self.__cond.wait()
        retval = self.__cv2_img
        self.__cv2_img = None
        self.__cond.release()
        return retval
