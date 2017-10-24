import logging

from constants import MINIMUM_PIXELS_DEFAULT, HSV_RANGE_DEFAULT
from contour_finder import ContourFinder

logger = logging.getLogger(__name__)


class GenericFilter(object):
    def __init__(self,
                 tracker,
                 bgr_color,
                 hsv_range=HSV_RANGE_DEFAULT,
                 minimum_pixels=MINIMUM_PIXELS_DEFAULT,
                 display_text=False,
                 draw_contour=False,
                 draw_box=False,
                 vertical_lines=False,
                 horizontal_lines=False,
                 predicate=None):
        self.tracker = tracker
        self.display_text = display_text
        self.draw_contour = draw_contour
        self.draw_box = draw_box
        self.vertical_lines = vertical_lines
        self.horizontal_lines = horizontal_lines
        self.predicate = predicate
        self._prev_x, self._prev_y = -1, -1
        self.height, self.width = -1, -1
        self.contours = None
        self.contour_finder = ContourFinder(bgr_color, hsv_range, minimum_pixels)
        # self.location_server = LocationServer(grpc_port)

    @property
    def prev_x(self):
        return self._prev_x

    @prev_x.setter
    def prev_x(self, val):
        self._prev_x = val

    @property
    def prev_y(self):
        return self._prev_y

    @prev_y.setter
    def prev_y(self, val):
        self._prev_y = val

    @property
    def middle_inc(self):
        # The middle margin calculation is based on % of width for horizontal and vertical boundary
        mid_x = self.width / 2
        middle_pct = (float(self.tracker.middle_percent) / 100.0) / 2
        return int(mid_x * middle_pct)

    def start(self):
        # try:
        #    self.location_server.start()
        # except BaseException as e:
        #    logger.error("Unable to start location server [{0}]".format(e), exc_info=True)
        #    sys.exit(1)
        pass

    def stop(self):
        # self.location_server.stop()
        pass

    def reset(self):
        self.prev_x, self.prev_y = -1, -1

    def reset_data(self):
        raise Exception("Should be implemented by sub-class")

    def process_image(self, image):
        raise Exception("Should be implemented by sub-class")

    def publish_data(self):
        raise Exception("Should be implemented by sub-class")

    def markup_image(self, image):
        raise Exception("Should be implemented by sub-class")
