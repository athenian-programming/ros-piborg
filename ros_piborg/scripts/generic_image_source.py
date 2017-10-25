class GenericImageSource(object):
    def __init__(self):
        self.stopped = False

    def start(self):
        raise Exception("Should be implemented by subclass")

    def stop(self):
        raise Exception("Should be implemented by subclass")

    def get_image(self):
        raise Exception("Should be implemented by subclass")
