#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


def flip(image):
    img = image
    if True: #self.__flip_x:
        img = cv2.flip(img, 0)
    #if self.__flip_y:
    #    img = cv2.flip(img, 1)
    return img

def update_image(msg):
    global img_pub
    global bridge
    format = msg.format
    image = msg.data
    print("Got {0} pic of length {1}".format(format, len(image)))

    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        new_image = flip(cv2_img)

        # pub.publish(bridge.cv2_to_compressed_imgmsg(new_image, "jpg"))
        img_pub.publish(bridge.cv2_to_imgmsg(new_image, "bgr8"))
    except CvBridgeError as e:
        print(e)


def update_info(msg):
    global info_pub

    try:
        # pub.publish(bridge.cv2_to_compressed_imgmsg(new_image, "jpg"))
        info_pub.publish(msg)
    except CvBridgeError as e:
        print(e)


if __name__ == '__main__':
    rospy.init_node('process_image')

    # Instantiate CvBridge
    bridge = CvBridge()

    rospy.Subscriber('/raspicam_node/image_raw/compressed', CompressedImage, update_image)
    rospy.Subscriber('/raspicam_node/camera_info', CameraInfo, update_info)

    # img_pub = rospy.Publisher('/test_img/image_raw', CompressedImage, queue_size=5)
    img_pub = rospy.Publisher('/test_img/image_raw', Image, queue_size=5)

    info_pub = rospy.Publisher('/test_img/camera_info', CameraInfo, queue_size=5)

    rospy.spin()
