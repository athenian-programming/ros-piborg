import rospy

import cli_args  as cli
from cli_args import LOG_LEVEL
from cli_args import setup_cli_args
from color_picker import ColorPicker
from direct_image_source import DirectImageSource
from utils import setup_logging
from utils import strip_loglevel

if __name__ == "__main__":
    # Parse CLI args
    args = setup_cli_args(cli.width,
                          cli.usb,
                          cli.usb_port,
                          cli.display,
                          cli.flip_x,
                          cli.flip_y,
                          cli.http_host,
                          cli.http_file,
                          cli.http_delay_secs,
                          cli.http_verbose,
                          cli.verbose)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    image_source = DirectImageSource()
    image_source.start()

    color_picker = ColorPicker(image_source, **strip_loglevel(args))
    try:
        color_picker.run()
    except KeyboardInterrupt:
        pass
    finally:
        color_picker.stop()
        image_source.stop()

    rospy.loginfo("Exiting...")
