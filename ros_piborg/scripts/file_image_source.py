import cv2

import cli_args  as cli


class FileImageSource(object):
    args = [cli.filename]

    def __init__(self, filename):
        self.__cv2_img = cv2.imread(filename)
        self.stopped = False

    def get_image(self):
        return self.__cv2_img
