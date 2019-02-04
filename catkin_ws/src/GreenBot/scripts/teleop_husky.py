#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Polygon, Point32
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np
import serial
import subprocess
import re
import time

# Should this be top import?
import roslib
roslib.load_manifest("GreenBot")


# ------------------------------------------------------------------------------
# Teleoperation CLASS
# ------------------------------------------------------------------------------
class TeleOpHusky:
    def __init__(self):
        rospy.init_node('teleop_husky')  # Initiate teleop node

        # Subscriber Initiations
        rospy.Subscriber('joy', Joy, self.handle_joy)
        rospy.Subscriber('person_detection/person', Polygon, self.handle_polygon)
        rospy.Subscriber('my_cam', )
        rospy.Subscriber('qr', String, self.handle_qr)

        # Publisher Initiations
        self.vel_cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Teleop variables
        self.joy_vector = None
        self.joy_data = None
        self.mast_controls = None

        # Automation variables
        self.polygon_data = None
        self.qr_data = None

        self.min_linear = 0.1
        self.max_linear = 0.95
        self.min_angular = 0.1
        self.max_angular = 0.95

        self.magnitude = 1.5
        self.drive_scale = 1
        self.turn_scale = 1
        self.safe_reverse_speed = 0.3

        self.deadman_button = 0
        self.override_buttons = [self.deadman_button]
        self.override = False
        self.safe_motion = False
        self.detect_qr = False

        self.image_time_start = None
        self.image_duration = 10  # 10s to image plant
        self.imaging = False
        self.at_plant = False

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
    def handle_joy(self, joy_data):
        self.joy_data = joy_data
        self.override = True

        #for button in self.override_buttons:
        #    if joy_data.buttons[button] == 0:
        #        self.override = False

        #self.safe_motion = not self.override and joy_data.buttons[self.deadman_button] != 0
        self.safe_motion = joy_data.buttons[self.deadman_button] != 0

        x = joy_data.axes[5]
        y = joy_data.axes[4]
        joy_vector = np.array([x, y])
        joy_vector /= np.linalg.norm(joy_vector)
        joy_vector *= self.magnitude

        self.joy_vector = joy_vector

        # Mast controls
        self.mast_controls = joy_data.buttons[2:4]  # TODO change buttons

        return

# ------------------------------------------------------------------------------
    def handle_polygon(self, poly_data):
        self.polygon_data = poly_data

        return

# ------------------------------------------------------------------------------
    def handle_qr(self, data):
        if self.qr_data == data:  # Check if still reading the same QR code
            return  # Already storing this data
        self.qr_data = data

        if self.qr_data is not None:  # TODO is the data None if empty? Empty string?
            self.detect_qr = True
        else:
            self.detect_qr = False

        return

# ------------------------------------------------------------------------------
    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            vel_command = self.compute_motion_cmd()
            mast_command = self.mast_cmd()

            # Publish the most recent command
            if vel_command is not None:
                self.vel_cmd_pub.publish(vel_command)
            elif mast_command is not None:
                self.pub_mast_cmd(mast_command)
            #  rate.sleep()

        return

# ------------------------------------------------------------------------------
    def image_plant(self):
        # Drive forward X meters
        # TODO send drive command

        # Image for X seconds
        self.image_time_start = time.time()
        while time.time() - self.image_duration < self.image_time_start:
            pass  # Wait while plant is being imaged

        # Clear all flags before moving onto next plant once done
        self.imaging = False
        self.detect_qr = False
        self.qr_data = None

        return

# ------------------------------------------------------------------------------
    def compute_motion_cmd(self):
        command = Twist()

        if self.joy_data is None:
            command = None

        elif self.detect_qr is not False:  # Detected QR therefore stop at plant and image
            self.image_plant()
            command = None

        # Don't move if not touching thumb stick
        #elif self.joy_data.axes[5] == 0.0 and self.joy_data.axes[4] == 0.0:
        #    command = None

        #elif self.override:
        #    command = Twist()
        #    # TODO Double check axes order
        #    command.linear.x = self.joy_data.axes[1] * self.drive_scale
        #    command.angular.z = self.joy_data.axes[0] * self.turn_scale

        elif self.safe_motion:
            vector_sum = self.joy_vector
            vector_sum /= np.linalg.norm(self.joy_vector)

            joy_cmd_vector = np.array([self.joy_data.axes[5], self.joy_data.axes[4]])
            vector_sum *= np.linalg.norm(joy_cmd_vector)

            # Reduce reverse motion speed (Can't see behind us)
            if joy_cmd_vector[0] >= 0:
                vector_sum[0] = max(0, vector_sum)
            else:
                vector_sum[0] = max(-self.safe_reverse_speed, vector_sum[0])

            command = Twist()
            command.linear.x = vector_sum[5] * self.drive_scale
            command.angular.z = vector_sum[4] * -self.turn_scale

        if command is not None:
            return self.clip_velocity(command)
        else:
            return None

    def clip_velocity(self, command):
        x = command.linear.x
        w = command.angular.z

        # TODO Maybe remove this?
        # Forward motion
        if abs(x) < self.min_linear:
            x = 0
        if abs(w) < self.min_angular:
            w = 0

        # Reverse motion
        if x < -self.min_linear:
            x = -self.min_linear
        elif x > self.max_linear:
            x = self.max_linear

        # Turning motion
        if w < -self.min_angular:
            w = -self.min_angular
        elif w > self.max_angular:
            w = self.max_angular

        command.linear.x = x
        command.angular.z = w

        return command

# ------------------------------------------------------------------------------
    def mast_cmd(self):

        # TODO fix button order
        if self.mast_controls[0] != 0:  # 'A' button for UP
            command = 'u'
        elif self.mast_controls[1] != 0:  # 'B' button for DOWN
            command = 'd'
        elif self.mast_controls[2] != 0:  # 'Left Bumper' for LEFT
            command = 'l'
        elif self.mast_controls[3] != 0:  # 'Right Bumper' for RIGHT
            command = 'r'
        elif self.mast_controls[4] != 0:  # 'Y' for STOP
            command = 's'
        else:
            command = None

        return command

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
# END OF Teleoperation CLASS
# ------------------------------------------------------------------------------


if __name__ == '__main__':
    cmd = TeleOpHusky()
    cmd.start()

