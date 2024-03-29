#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging

import arc852.cli_args  as cli
import rospy
from arc852.cli_args import LOG_LEVEL
from arc852.cli_args import setup_cli_args
from arc852.constants import DISPLAY, WIDTH, MIDDLE_PERCENT
from arc852.constants import DRAW_LINE, DRAW_CONTOUR, DRAW_BOX, MAXIMUM_OBJECTS
from arc852.constants import FLIP_X, FLIP_Y, HTTP_DELAY_SECS, TEMPLATE_FILE, HTTP_VERBOSE
from arc852.constants import MASK_X, MASK_Y, BGR_COLOR
from arc852.constants import MINIMUM_PIXELS, HSV_RANGE, CAMERA_NAME, FILENAME, FPS, HTTP_HOST, SO_TOPIC
from arc852.image_server import ImageServer
from arc852.object_tracker import ObjectTracker
from arc852.utils import setup_logging
from arc852.video_image_source import VideoImageSource

from multi_object_filter import MultiObjectFilter

logger = logging.getLogger(__name__)


def main():
    # Parse CLI args
    args = setup_cli_args(VideoImageSource.args,
                          ImageServer.args, cli.camera_name_optional,
                          ObjectTracker.args,
                          MultiObjectFilter.args,
                          cli.log_level)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    rospy.init_node('video_multi_object')

    image_source = VideoImageSource(filename=args[FILENAME], fps_rate=args[FPS])

    image_server = ImageServer(template_file=args[TEMPLATE_FILE],
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


if __name__ == "__main__":
    main()
