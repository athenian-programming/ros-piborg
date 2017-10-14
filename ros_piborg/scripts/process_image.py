#!/usr/bin/env python

import rospy
from sensor_msgs.CompressedImage import CompressedImage


def update_image(msg):
    format = msg.format
    image = msg.data
    print("Got {0} pic of length {1}".format(format, len(image)))


if __name__ == '__main__':
    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, update_image)
    rospy.spin()
