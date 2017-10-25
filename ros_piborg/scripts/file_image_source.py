import cv2

import cli_args  as cli


class FileImageSource(object):
    args = [cli.filename]

    def __init__(self, filename, fps_rate=30):
        self.__cv2_img = cv2.imread(filename)

    def start(self):
        pass

    def stop(self):
        pass

    def get_image(self):
        return self.__cv2_img
