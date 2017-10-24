import cv2

import cli_args  as cli
import opencv_defaults as defs
from generic_filter import GenericFilter
from opencv_utils import BLUE, GREEN, RED
from opencv_utils import get_moment


class SingleObjectFilter(GenericFilter):
    args = [cli.bgr, cli.hsv_range, cli.min_pixels, cli.draw_contour, cli.draw_box, cli.vert_lines, cli.horiz_lines]

    def __init__(self, tracker, *args, **kwargs):
        super(SingleObjectFilter, self).__init__(tracker, *args, **kwargs)
        self.contour = None
        self.area = None
        self.img_x, self.img_y = -1, -1
        self.height, self.width = None, None

    def reset_data(self):
        self.img_x, self.img_y = -1, -1

    def process_image(self, image):
        self.contour = None
        self.reset_data()
        self.height, self.width = image.shape[:2]

        # Find the largest contour
        self.contours = self.contour_finder.get_max_contours(image, count=1)

        if self.contours is not None and len(self.contours) == 1:
            self.contour, self.area, self.img_x, self.img_y = get_moment(self.contours[0])

    def publish_data(self):
        # Write location if it is different from previous value written
        if self.img_x != self.prev_x or self.img_y != self.prev_y:
            # self.location_server.write_location(self.img_x, self.img_y, self.width, self.height, self.middle_inc)
            self.prev_x, self.prev_y = self.img_x, self.img_y

    def markup_image(self, image):
        mid_x, mid_y = self.width / 2, self.height / 2
        middle_inc = int(self.middle_inc)

        x_in_middle = mid_x - middle_inc <= self.img_x <= mid_x + middle_inc
        y_in_middle = mid_y - middle_inc <= self.img_y <= mid_y + middle_inc
        x_color = GREEN if x_in_middle else RED if self.img_x == -1 else BLUE
        y_color = GREEN if y_in_middle else RED if self.img_y == -1 else BLUE

        if not self.tracker.markup_image:
            return

        text = "#{0} ({1}, {2})".format(self.tracker.cnt, self.width, self.height)
        text += " {0}%".format(self.tracker.middle_percent)

        if self.contours is not None and len(self.contours) == 1:
            x, y, w, h = cv2.boundingRect(self.contour)
            if self.draw_box:
                cv2.rectangle(image, (x, y), (x + w, y + h), BLUE, 2)
            if self.draw_contour:
                cv2.drawContours(image, [self.contour], -1, GREEN, 2)
            cv2.circle(image, (self.img_x, self.img_y), 4, RED, -1)
            text += " ({0}, {1})".format(self.img_x, self.img_y)
            text += " {0}".format(self.area)

        # Draw the alignment lines
        if self.vertical_lines:
            cv2.line(image, (mid_x - middle_inc, 0), (mid_x - middle_inc, self.height), x_color, 1)
            cv2.line(image, (mid_x + middle_inc, 0), (mid_x + middle_inc, self.height), x_color, 1)
        if self.horizontal_lines:
            cv2.line(image, (0, mid_y - middle_inc), (self.width, mid_y - middle_inc), y_color, 1)
            cv2.line(image, (0, mid_y + middle_inc), (self.width, mid_y + middle_inc), y_color, 1)
        if self.display_text:
            cv2.putText(image, text, defs.TEXT_LOC, defs.TEXT_FONT, defs.TEXT_SIZE, RED, 1)
