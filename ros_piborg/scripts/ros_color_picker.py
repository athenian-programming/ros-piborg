#!/usr/bin/env python

import rospy

import cli_args  as cli
from cli_args import LOG_LEVEL
from cli_args import setup_cli_args
from color_picker import ColorPicker
from constants import DISPLAY, HTTP_HOST, HTTP_FILE, HTTP_DELAY_SECS, HTTP_VERBOSE
from constants import TOPIC, COMPRESSED, FORMAT, WIDTH, FLIP_X, FLIP_Y
from image_server import ImageServer
from ros_image_source import RosImageSource
from utils import setup_logging

if __name__ == "__main__":
    # Parse CLI args
    args = setup_cli_args(cli.topic,
                          cli.compressed,
                          cli.format,
                          cli.display,
                          cli.width,
                          cli.flip_x,
                          cli.flip_y,
                          cli.http_host,
                          cli.http_file,
                          cli.http_delay_secs,
                          cli.http_verbose,
                          cli.log_level)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    image_source = RosImageSource(topic=args[TOPIC],
                                  compressed=args[COMPRESSED],
                                  format=args[FORMAT])

    image_server = ImageServer(http_file=args[HTTP_FILE],
                               camera_name="Color Picker",
                               http_host=args[HTTP_HOST],
                               http_delay_secs=args[HTTP_DELAY_SECS],
                               http_verbose=args[HTTP_VERBOSE])

    color_picker = ColorPicker(image_source=image_source,
                               image_server=image_server,
                               width=args[WIDTH],
                               flip_x=args[FLIP_X],
                               flip_y=args[FLIP_Y],
                               display=args[DISPLAY])
    try:
        image_source.start()
        image_server.start()
        color_picker.run()
    except KeyboardInterrupt:
        pass
    finally:
        image_server.stop()
        image_source.stop()

    rospy.loginfo("Exiting...")
