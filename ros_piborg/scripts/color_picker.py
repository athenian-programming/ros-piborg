#!/usr/bin/env python2

import logging
import numpy as np
import time
from cv_bridge import CvBridge

import cv2
import imutils
import rospy
from sensor_msgs.msg import CompressedImage

import cli_args  as cli
import opencv_defaults as defs
from cli_args import LOG_LEVEL
from cli_args import setup_cli_args
from image_server import ImageServer
from opencv_utils import GREEN
from opencv_utils import RED
from utils import setup_logging
from utils import strip_loglevel

logger = logging.getLogger(__name__)


class ColorPicker(object):
    roi_size = 24
    orig_roi_size = roi_size
    roi_inc = 6
    move_inc = 4
    x_adj = 0
    y_adj = 0

    def __init__(self,
                 width,
                 usb_camera,
                 usb_port,
                 flip_x,
                 flip_y,
                 display,
                 http_host,
                 http_file,
                 http_delay_secs,
                 http_verbose):
        self.__width = width
        self.__usb_camera = usb_camera
        self.__flip_x = flip_x
        self.__flip_y = flip_y
        self.__display = display
        self.__orig_width = self.__width
        self.__image = None
        self.roi_x = 0
        self.roi_y = 0
        self.bgr_text = ""
        self.width = 0
        self.height = 0
        self.stopped = False
        self.cnt = 0

        # Instantiate CvBridge
        self.bridge = CvBridge()

        self.__image_server = ImageServer(http_file,
                                          camera_name="Color Picker",
                                          http_host=http_host,
                                          http_delay_secs=http_delay_secs,
                                          http_verbose=http_verbose)

    def __read_image(self, msg):
        try:
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            cv2_img = imutils.resize(cv2_img, width=self.__width)

            if self.__flip_x:
                cv2_img = cv2.flip(cv2_img, 0)

            if self.__flip_y:
                cv2_img = cv2.flip(cv2_img, 1)

            self.height, self.width = cv2_img.shape[:2]

            self.roi_x = (self.width / 2) - (self.roi_size / 2) + self.x_adj
            self.roi_y = (self.height / 2) - (self.roi_size / 2) + self.y_adj
            roi = cv2_img[self.roi_y:self.roi_y + self.roi_size, self.roi_x:self.roi_x + self.roi_size]

            roi_h, roi_w = roi.shape[:2]
            roi_canvas = np.zeros((roi_h, roi_w, 3), dtype="uint8")
            roi_canvas[0:roi_h, 0:roi_w] = roi

            # Calculate averge color in ROI
            avg_color_per_row = np.average(roi_canvas, axis=0)
            avg_color = np.average(avg_color_per_row, axis=0)
            avg_color = np.uint8(avg_color)

            # Draw a rectangle around the sample area
            cv2.rectangle(cv2_img, (self.roi_x, self.roi_y), (self.roi_x + self.roi_size, self.roi_y + self.roi_size),
                          GREEN, 1)

            # Add text info
            self.bgr_text = "BGR value: [{0}, {1}, {2}]".format(avg_color[0], avg_color[1], avg_color[2])
            roi_text = " ROI: {0}x{1} ".format(str(self.roi_size), str(self.roi_size))
            cv2.putText(cv2_img, self.bgr_text + roi_text, defs.TEXT_LOC, defs.TEXT_FONT, defs.TEXT_SIZE, RED, 1)

            # Overlay color swatch on image
            size = int(self.width * 0.20)
            cv2_img[self.height - size:self.height, self.width - size:self.width] = avg_color

            self.cnt += 1

            if self.__image_server.enabled and self.cnt % 30 == 0:
                logger.info(self.bgr_text)

            self.__image = cv2_img
            self.__image_server.image = cv2_img

        except BaseException as e:
            logger.error("Unexpected error in main loop [{0}]".format(e), exc_info=True)
            time.sleep(1)

    # Do not run this in a background thread. cv2.waitKey has to run in main thread
    def start(self):
        self.__image_server.start()

        rospy.init_node('object_tracker')
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.__read_image)

        if self.__display:
            while not self.stopped:
                if self.__image is not None:
                    self.display_image()
                else:
                    rospy.sleep(0.1)
        else:
            rospy.spin()

    def display_image(self):
        # Display image
        cv2.imshow("Image", self.__image)

        key = cv2.waitKey(30) & 0xFF

        if key == 255:
            pass
        elif key == ord("c") or key == ord(" "):
            print(self.bgr_text)
        elif self.roi_y >= self.move_inc and (key == 0 or key == ord("k")):  # Up
            self.y_adj -= self.move_inc
        elif self.roi_y <= self.height - self.roi_size - self.move_inc and (key == 1 or key == ord("j")):  # Down
            self.y_adj += self.move_inc
        elif self.roi_x >= self.move_inc and (key == 2 or key == ord("h")):  # Left
            self.x_adj -= self.move_inc
        elif self.roi_x <= self.width - self.roi_size - self.move_inc - self.move_inc \
                and (key == 3 or key == ord("l")):  # Right
            self.x_adj += self.move_inc
        elif self.roi_size >= self.roi_inc * 2 and (key == ord("-") or key == ord("_")):
            self.roi_size -= self.roi_inc
            self.x_adj, self.y_adj = 0, 0
        elif self.roi_size <= self.roi_inc * 49 and (key == ord("+") or key == ord("=")):
            self.roi_size += self.roi_inc
            self.x_adj, self.y_adj = 0, 0
        elif key == ord("r"):
            self.__width = self.__orig_width
            self.roi_size = self.orig_roi_size
        elif key == ord("w"):
            self.__width -= 10
        elif key == ord("e"):
            self.__width += 10
        elif key == ord("q"):
            self.stopped = True

    def stop(self):
        self.__image_server.stop()


if __name__ == "__main__":
    # Parse CLI args
    args = setup_cli_args(cli.width,
                          cli.usb,
                          cli.usb_port,
                          cli.display,
                          cli.flip_x,
                          cli.flip_y,
                          cli.http_host,
                          cli.http_file,
                          cli.http_delay_secs,
                          cli.http_verbose,
                          cli.verbose)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    color_picker = ColorPicker(**strip_loglevel(args))
    try:
        color_picker.start()
    except KeyboardInterrupt:
        pass
    finally:
        color_picker.stop()

    logger.info("Exiting...")
