#!/usr/bin/env python

import logging

import rospy

import cli_args  as cli
from camera_image_source import CameraImageSource
from cli_args import LOG_LEVEL
from cli_args import setup_cli_args
from constants import DISPLAY, WIDTH, MIDDLE_PERCENT
from constants import DRAW_CONTOUR, DRAW_BOX, VERTICAL_LINES, HORIZONTAL_LINES
from constants import FLIP_X, FLIP_Y, HTTP_DELAY_SECS, HTTP_FILE, HTTP_VERBOSE
from constants import MASK_X, MASK_Y, USB_PORT, BGR_COLOR
from constants import MINIMUM_PIXELS, HSV_RANGE, CAMERA_NAME, USB_CAMERA, HTTP_HOST
from image_server import ImageServer
from object_tracker import ObjectTracker
from single_object_filter import SingleObjectFilter
from utils import setup_logging

logger = logging.getLogger(__name__)

if __name__ == "__main__":
    # Parse CLI args
    args = setup_cli_args(cli.bgr,
                          cli.usb_camera,
                          cli.usb_port,
                          cli.width,
                          cli.middle_percent,
                          cli.minimum_pixels,
                          cli.hsv_range,
                          cli.flip_x,
                          cli.flip_y,
                          cli.mask_x,
                          cli.mask_y,
                          cli.vertical_lines,
                          cli.horizontal_lines,
                          cli.camera_name_optional,
                          cli.display,
                          cli.draw_contour,
                          cli.draw_box,
                          cli.http_host,
                          cli.http_file,
                          cli.http_delay_secs,
                          cli.http_verbose,
                          cli.log_level)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    image_source = CameraImageSource(usb_camera=args[USB_CAMERA], usb_port=args[USB_PORT])

    image_server = ImageServer(http_file=args[HTTP_FILE],
                               camera_name=args[CAMERA_NAME],
                               http_host=args[HTTP_HOST],
                               http_delay_secs=args[HTTP_DELAY_SECS],
                               http_verbose=args[HTTP_VERBOSE])

    tracker = ObjectTracker(image_source=image_source,
                            image_server=image_server,
                            width=args[WIDTH],
                            middle_percent=args[MIDDLE_PERCENT],
                            display=args[DISPLAY],
                            flip_x=args[FLIP_X],
                            flip_y=args[FLIP_Y],
                            mask_x=args[MASK_X],
                            mask_y=args[MASK_Y])

    obj_filter = SingleObjectFilter(tracker,
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

    rospy.loginfo("Exiting...")