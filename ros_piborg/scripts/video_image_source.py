import logging
from threading import Condition
from threading import Thread

import cv2
import imutils
import rospy

import cli_args  as cli

logger = logging.getLogger(__name__)


class VideoImageSource(object):
    args = [cli.filename, cli.fps]

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
                logger.info("Breaking on null read")
                break
            self.__cv2_img = imutils.resize(self.__cv2_img, width=900)

            self.__cond.notify()
            self.__cond.release()
            self.__rate.sleep()

    def get_image(self):
        logger.info("Getting Image")
        self.__cond.acquire()
        while self.__cv2_img is None:
            self.__cond.wait()
        retval = self.__cv2_img
        self.__cv2_img = None
        self.__cond.release()
        return retval
