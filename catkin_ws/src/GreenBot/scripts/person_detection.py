#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Polygon, Point32
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import imutils

# Should this be top import?
import roslib
roslib.load_manifest("GreenBot")


# ------------------------------------------------------------------------------
# HogDetector CLASS
# ------------------------------------------------------------------------------
class HogDetector:
    def __init__(self):
        self.hog = cv2.HOGDescriptor()
        # Take advantage of OpenCV's pre-trained detector
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.hog_win_stride = (8, 8)
        self.hog_padding = (2, 2)
        self.hog_scale = 1.075
        # Decrease scale to increase speed but missed detection

        return

# ------------------------------------------------------------------------------
    def apply_frame(self, image):
        # Create image pyramid
        people, weights = self.hog.detectMultiScale(image,
                                                    winStride=self.hog_win_stride,
                                                    padding=self.hog_padding,
                                                    scale=self.hog_scale)

        rectangles = []

        for person in people:
            p0 = (person[0], person[1])
            p1 = (p0[0] + person[2], p0[1])
            p2 = (p0[0] + person[2], p0[1] + person[3])
            p3 = (p0[0], p0[1] + person[3])

            rectangles.append((p0, p1, p2, p3))

        return rectangles

# ------------------------------------------------------------------------------
# END OF HogDetector CLASS
# ------------------------------------------------------------------------------


# ------------------------------------------------------------------------------
# PersonDetection CLASS
# ------------------------------------------------------------------------------
class PersonDetection:
    """

    """
    def __init__(self):
        self.detected_people_topic = "person_detection/people"
        self.vid_topic = "frontcam"
        self.visualization_topic = "person_detection/viz"
        self.setup_ros_node()

        self.last_detection = rospy.Time.now()
        self.detection_interval = rospy.Duration(1)  # TODO determine proper interval value

        self.cv_bridge = CvBridge
        self.detector = HogDetector

        return

# ------------------------------------------------------------------------------
    def setup_ros_node(self):
        rospy.init_node('person_detector', anonymous=True)  # Create the detector node

        self.people_pub = rospy.Publisher(self.detected_people_topic, Polygon)
        self.viz_publisher = rospy.Publisher(self.visualization_topic, Image)

        rospy.Subscriber(self.vid_topic, Image, self.process_image)
        return

# ------------------------------------------------------------------------------
    # Check if any Nodes are subscribed to this Node
    # Skip processing if not to save resources
    def no_one_listening(self):
        return self.people_pub.get_num_connections() < 1 \
               and self.viz_publisher.get_num_connections() < 1

# ------------------------------------------------------------------------------
    def process_image(self, data):
        if self.no_one_listening():
            return

        begin_processing = rospy.Time.now()

        if begin_processing - self.last_detection < self.detection_interval:
            return

        # Unpackage image from ros message
        cv_im = self.unpack_image(data)

        rectangles = self.detector.apply_frame(cv_im)

        # Use Super Polygon
        super_polygon = []
        for rect in rectangles:
            super_polygon.append(Point32(x=rect[0][0], y=rect[0][1], z=0))
            super_polygon.append(Point32(x=rect[1][0], y=rect[1][1], z=0))
            super_polygon.append(Point32(x=rect[2][0], y=rect[2][1], z=0))
            super_polygon.append(Point32(x=rect[3][0], y=rect[3][1], z=0))

        self.people_pub.publish(Polygon(super_polygon))
        self.publish_viz(rectangles, cv_im)

        return

# ------------------------------------------------------------------------------
    def publish_viz(self, rectangles, img):
        if self.viz_publisher.get_num_connections() < 1:
            return

        for rect in rectangles:
            cv2.rectangle(img, rect[0], rect[2], (255, 255, 255))

        msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.viz_publisher.publish(msg)

        return

# ------------------------------------------------------------------------------
    def unpack_image(self, message):
        try:
            unpacked_im = self.cv_bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError, e:
            print e

        return np.asarray(unpacked_im[:, :])

# ------------------------------------------------------------------------------
# END OG PersonDetection CLASS
# ------------------------------------------------------------------------------


if __name__ == '__main__':
    detect = PersonDetection()


