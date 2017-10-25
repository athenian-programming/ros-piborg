#!/usr/bin/env python

import logging

import cli_args  as cli
from camera_image_source import CameraImageSource
from cli_args import LOG_LEVEL
from cli_args import setup_cli_args
from color_picker import ColorPicker
from constants import USB_CAMERA, USB_PORT, WIDTH, FLIP_X, FLIP_Y
from utils import setup_logging

logger = logging.getLogger(__name__)

if __name__ == "__main__":
    # Parse CLI args
    args = setup_cli_args(CameraImageSource.args, ColorPicker.args, cli.log_level)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    image_source = CameraImageSource(usb_camera=args[USB_CAMERA], usb_port=args[USB_PORT])

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

    logger.info("Exiting...")
