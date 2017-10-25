#!/usr/bin/env python

import logging

import cli_args  as cli
from cli_args import LOG_LEVEL
from cli_args import setup_cli_args
from color_picker import ColorPicker
from constants import DISPLAY, HTTP_HOST, HTTP_FILE, HTTP_DELAY_SECS, HTTP_VERBOSE
from constants import FILENAME, WIDTH, FLIP_X, FLIP_Y
from file_image_source import FileImageSource
from image_server import ImageServer
from utils import setup_logging

logger = logging.getLogger(__name__)

if __name__ == "__main__":
    # Parse CLI args
    args = setup_cli_args(FileImageSource.args,
                          ImageServer.args,
                          ColorPicker.args,
                          cli.log_level)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    image_source = FileImageSource(filename=args[FILENAME])

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

    logger.info("Exiting...")
