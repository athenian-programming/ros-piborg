#!/usr/bin/env python

import logging

import rospy

import cli_args  as cli
from cli_args import LOG_LEVEL
from cli_args import setup_cli_args
from color_picker import ColorPicker
from constants import IMAGE_TOPIC, COMPRESSED, FORMAT, WIDTH, FLIP_X, FLIP_Y
from ros_image_source import RosImageSource
from utils import setup_logging

logger = logging.getLogger(__name__)

if __name__ == "__main__":
    # Parse CLI args
    args = setup_cli_args(RosImageSource.args, ColorPicker.args, cli.log_level)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    rospy.init_node('ros_color_picker')

    image_source = RosImageSource(topic=args[IMAGE_TOPIC],
                                  compressed=args[COMPRESSED],
                                  format=args[FORMAT])

    color_picker = ColorPicker(image_source=image_source,
                               width=args[WIDTH],
                               flip_x=args[FLIP_X],
                               flip_y=args[FLIP_Y])

    try:
        image_source.start()
        color_picker.run()
    except KeyboardInterrupt:
        pass
    finally:
        image_source.stop()

    logger = logging.getLogger(__name__)
