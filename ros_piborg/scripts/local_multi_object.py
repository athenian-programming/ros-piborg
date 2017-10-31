#!/usr/bin/env python

import logging

import rospy

import cli_args  as cli
from camera_image_source import CameraImageSource
from cli_args import LOG_LEVEL
from cli_args import setup_cli_args
from constants import DISPLAY, WIDTH, MIDDLE_PERCENT
from constants import DRAW_LINE, DRAW_CONTOUR, DRAW_BOX, MAXIMUM_OBJECTS
from constants import FLIP_X, FLIP_Y, HTTP_DELAY_SECS, HTTP_FILE, HTTP_VERBOSE
from constants import MINIMUM_PIXELS, HSV_RANGE, CAMERA_NAME, HTTP_HOST, SO_TOPIC
from constants import USB_CAMERA, USB_PORT, MASK_X, MASK_Y, BGR_COLOR
from image_server import ImageServer
from multi_object_filter import MultiObjectFilter
from object_tracker import ObjectTracker
from utils import setup_logging

logger = logging.getLogger(__name__)

if __name__ == "__main__":
    # Parse CLI args
    args = setup_cli_args(CameraImageSource.args,
                          ImageServer.args, cli.camera_name_optional,
                          ObjectTracker.args,
                          MultiObjectFilter.args,
                          cli.log_level)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    rospy.init_node('video_multi_object')

    image_source = CameraImageSource(usb_camera=args[USB_CAMERA], usb_port=args[USB_PORT])

    image_server = ImageServer(http_file=args[HTTP_FILE],
                               camera_name=args[CAMERA_NAME],
                               http_host=args[HTTP_HOST],
                               http_delay_secs=args[HTTP_DELAY_SECS],
                               http_verbose=args[HTTP_VERBOSE])

    tracker = ObjectTracker(image_source=image_source,
                            image_server=image_server,
                            display=args[DISPLAY],
                            width=args[WIDTH],
                            middle_percent=args[MIDDLE_PERCENT],
                            flip_x=args[FLIP_X],
                            flip_y=args[FLIP_Y],
                            mask_x=args[MASK_X],
                            mask_y=args[MASK_Y])

    obj_filter = MultiObjectFilter(tracker=tracker,
                                   so_topic=args[SO_TOPIC],
                                   maximum_objects=args[MAXIMUM_OBJECTS],
                                   bgr_color=args[BGR_COLOR],
                                   hsv_range=args[HSV_RANGE],
                                   minimum_pixels=args[MINIMUM_PIXELS],
                                   display_text=True,
                                   draw_line=args[DRAW_LINE],
                                   draw_contour=args[DRAW_CONTOUR],
                                   draw_box=args[DRAW_BOX])

    rospy.loginfo("Running")

    try:
        image_source.start()
        image_server.start()
        tracker.run(obj_filter)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.cleanup()
        image_server.stop()
        image_source.stop()

    rospy.loginfo("Exiting")
