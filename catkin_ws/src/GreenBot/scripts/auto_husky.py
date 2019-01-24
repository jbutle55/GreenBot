#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Polygon, Point32
from sensor_msgs.msg import Joy
import numpy as np
import serial
import subprocess
import re

# Should this be top import?
import roslib
roslib.load_manifest("GreenBot")


# ------------------------------------------------------------------------------
# Autonomous Navigation CLASS
# ------------------------------------------------------------------------------
class AutoHusky:
    def __init__(self):
        rospy.init_node('auto_husky')  # Initiate teleop node

        # Subscriber Initiations
        rospy.Subscriber('person_detection/person', Polygon, self.handle_polygon)
        rospy.Subscriber('my_cam', )

        # Publisher Initiations
        self.vel_cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Open the serial port
        self.arduino = serial.Serial(self.get_usb_info(), 9600)

        return

# ------------------------------------------------------------------------------
    @staticmethod
    def get_usb_info():
        # regular expression for lsusb output
        device_re = re.compile(r"Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+):\s+"
                               r"ID\s+(?P<id>\w+:\w+)\s*(?P<tag>.+)$", re.I | re.MULTILINE)
        usb_info = subprocess.check_output('lsusb')
        device = 'dev/bus/usb/'

        for info in usb_info.split('\n'):
            if info:
                dev_info = device_re.match(info)

                dev_info['devices'] = '/dev/bus/usb/{bus}/{dev}'.format(bus=dev_info.pop('bus'),
                                                                        dev=dev_info.pop('device'))

        return device

# ------------------------------------------------------------------------------

    def handle_polygon(self, poly_data):
        self.polygon_data = poly_data

        return

    # ------------------------------------------------------------------------------
    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pass

        return

# ------------------------------------------------------------------------------
    def pub_mast_cmd(self, command):
        # encode in the appropriate format and send
        self.arduino.write(command.encode())

        return

    def store_video(self):

        return

    def store_ultra_data(self):

        return

# ------------------------------------------------------------------------------
# END OF Autonomous Navigation CLASS
# ------------------------------------------------------------------------------


if __name__ == '__main__':
    cmd = AutoHusky()
    cmd.start()

