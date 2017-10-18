from cv_bridge import CvBridge
from threading import Condition

import rospy
from sensor_msgs.msg import CompressedImage


class RosImageSource(object):
    def __init__(self):
        self.cond = Condition()
        self.cv2_img = None

        # Instantiate CvBridge
        self.bridge = CvBridge()

    def __read_image(self, msg):
        try:
            self.cond.acquire()
            self.cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.cond.notify()
        except BaseException as e:
            rospy.logerr("Unexpected error in read_image [{0}]".format(e))
        finally:
            self.cond.release()

    def start(self):
        rospy.init_node('ros_image_source')
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.__read_image)

    def stop(self):
        pass

    def get_image(self):
        retval = None
        try:
            self.cond.acquire()
            while self.cv2_img is None:
                self.cond.wait()
            retval = self.cv2_img
            self.cv2_img = None
        except BaseException as e:
            rospy.logerr("Unexpected error in image [{0}]".format(e))
        finally:
            self.cond.release()
        return retval
