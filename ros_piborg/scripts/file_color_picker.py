#!/usr/bin/env python

import logging

import arc852.cli_args  as cli
from arc852.cli_args import LOG_LEVEL
from arc852.cli_args import setup_cli_args
from arc852.color_picker import ColorPicker
from arc852.constants import FILENAME, WIDTH, FLIP_X, FLIP_Y
from arc852.file_image_source import FileImageSource
from arc852.utils import setup_logging

logger = logging.getLogger(__name__)

if __name__ == "__main__":
    # Parse CLI args
    args = setup_cli_args(FileImageSource.args, ColorPicker.args, cli.log_level)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    image_source = FileImageSource(filename=args[FILENAME])

    color_picker = ColorPicker(image_source=image_source,
                               width=args[WIDTH],
                               flip_x=args[FLIP_X],
                               flip_y=args[FLIP_Y])
    try:
        color_picker.run()
    except KeyboardInterrupt:
        pass

    logger.info("Exiting")
