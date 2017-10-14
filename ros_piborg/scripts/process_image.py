#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



def flip(image):
    img = image
    if True: #self.__flip_x:
        img = cv2.flip(img, 0)
    #if self.__flip_y:
    #    img = cv2.flip(img, 1)
    return img

def update_image(msg):
    global pub
    global bridge
    format = msg.format
    image = msg.data
    print("Got {0} pic of length {1}".format(format, len(image)))

    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)

    new_image = flip(cv2_img)

    try:
        # pub.publish(bridge.cv2_to_compressed_imgmsg(new_image, "jpg"))
        pub.publish(bridge.cv2_to_imgmsg(new_image, "bgr8"))
    except CvBridgeError as e:
        print(e)


if __name__ == '__main__':
    rospy.init_node('process_image')

    # Instantiate CvBridge
    bridge = CvBridge()

    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, update_image)
    #pub = rospy.Publisher('/test_img', CompressedImage, queue_size=5)
    pub = rospy.Publisher('/test_img', Image, queue_size=5)

    rospy.spin()
