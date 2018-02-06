#!/usr/bin/env python

import logging

import arc852.cli_args  as cli
import rospy
from arc852.cli_args import LOG_LEVEL
from arc852.cli_args import setup_cli_args
from arc852.color_picker import ColorPicker
from arc852.constants import IMAGE_TOPIC, COMPRESSED, FORMAT, WIDTH, FLIP_X, FLIP_Y
from arc852.ros_image_source import RosImageSource
from arc852.utils import setup_logging

logger = logging.getLogger(__name__)


def main():
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


if __name__ == "__main__":
    main()
