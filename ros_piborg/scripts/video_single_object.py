#!/usr/bin/env python

import logging

import rospy

import cli_args  as cli
from cli_args import LOG_LEVEL
from cli_args import setup_cli_args
from constants import DISPLAY, WIDTH, MIDDLE_PERCENT
from constants import DRAW_CONTOUR, DRAW_BOX, VERTICAL_LINES, HORIZONTAL_LINES
from constants import FLIP_X, FLIP_Y, HTTP_DELAY_SECS, HTTP_FILE, HTTP_VERBOSE
from constants import MASK_X, MASK_Y, BGR_COLOR
from constants import MINIMUM_PIXELS, HSV_RANGE, CAMERA_NAME, FILENAME, HTTP_HOST, SO_TOPIC
from image_server import ImageServer
from object_tracker import ObjectTracker
from single_object_filter import SingleObjectFilter
from utils import setup_logging
from video_image_source import VideoImageSource

logger = logging.getLogger(__name__)

if __name__ == "__main__":
    # Parse CLI args
    args = setup_cli_args(VideoImageSource.args,
                          ImageServer.args, cli.camera_name_optional,
                          SingleObjectFilter.args,
                          ObjectTracker.args,
                          cli.log_level)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    logger.info("Setting up ROS")
    rospy.init_node('video_single_object')

    logger.info("Setting up VideoImageSource")
    rospy.loginfo("Setting up VideoImageSource")
    image_source = VideoImageSource(filename=args[FILENAME])

    logger.info("Setting up ImageServer")
    image_server = ImageServer(http_file=args[HTTP_FILE],
                               camera_name=args[CAMERA_NAME],
                               http_host=args[HTTP_HOST],
                               http_delay_secs=args[HTTP_DELAY_SECS],
                               http_verbose=args[HTTP_VERBOSE])

    logger.info("Setting up ObjectTracker")
    tracker = ObjectTracker(image_source=image_source,
                            image_server=image_server,
                            display=args[DISPLAY],
                            width=args[WIDTH],
                            middle_percent=args[MIDDLE_PERCENT],
                            flip_x=args[FLIP_X],
                            flip_y=args[FLIP_Y],
                            mask_x=args[MASK_X],
                            mask_y=args[MASK_Y])

    logger.info("Setting up SingleObjectFilter")
    obj_filter = SingleObjectFilter(tracker=tracker,
                                    so_topic=args[SO_TOPIC],
                                    bgr_color=args[BGR_COLOR],
                                    hsv_range=args[HSV_RANGE],
                                    minimum_pixels=args[MINIMUM_PIXELS],
                                    display_text=True,
                                    draw_contour=args[DRAW_CONTOUR],
                                    draw_box=args[DRAW_BOX],
                                    vertical_lines=args[VERTICAL_LINES],
                                    horizontal_lines=args[HORIZONTAL_LINES])
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

    logger.info("Exiting...")
