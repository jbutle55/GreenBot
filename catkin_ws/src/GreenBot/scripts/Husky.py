#!/usr/bin/env python

import sys
import os
import husky_msgs
import rospy
import roslaunch
import theora_image_transport
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


# NODES
# ---------------------------------------
# Control pad Node
# Husky Node to listen for control pad messages
# QR Code node
# Ultrasound/laser sensor Node
# Front camera Node


def launch():  # Launch the GreenBot process
    # TODO does the launch file negate this?
    # See: http://wiki.ros.org/roslaunch/API%20Usage
    print 'Hello World'
    package = 'GreenBot'
    executable = 'GreenBot'
    node = roslaunch.core.Node(package, executable)
    print 'Hello World'
    greenbot = roslaunch.scriptapi.ROSLaunch()
    greenbot.start()
    print 'Hello World'
    greenbot_process = greenbot.launch(node)
    print greenbot_process.is_alive()
    return greenbot_process


def land(greenbot_process):  # Kill the GreenBot process
    # TODO Might need to included in same function as launch
    greenbot_process.stop()
    return


class GreenBot:
    """A node for the Husky A200. Subscribe to velocity commands, QR code detection,
     rangefinder sensors."""

    def __init__(self):
        global vel_cmd
        rospy.init_node('Husky', anonymous=True)  # Create a Husky node
        self.sub_controller()
        self.sub_prox_sensor()

        rospy.spin()
        return

    # Subsciber to joystick input commands
    def sub_controller(self):
        rospy.Subscriber("joy", Joy, self.callback_controller)  # Receive joystick input from joy topic
        return

    @staticmethod
    def callback_controller(data):
        if len(data) == 0:
            return  # No controller input

        # Convert data to Twist message
        twist = Twist()
        twist.linear.x = 4 * data.axes[1]  # Left stick vertical
        twist.angular.z = 4 * data.axes[0]  # Left stick horizontal

        # Publish w. velocity command publisher
        vel_cmd.publish(twist)
        return

    def sub_prox_sensor(self):
        rospy.Subscriber("prox1", Twist, self.callback_prox())  # TODO probably not Twist

        return

    @staticmethod
    def callback_prox(data):
        if len(data) == 0:
            return  # No sensor input

        twist = Twist()
        # TODO data here then publish
        return

    # Publisher of velocity commands to the GreenBot motor system
    @staticmethod
    def pub_cmb_vel():
        rospy.init_node('control_to_direction', anonymous=True)
        vel_cmd = rospy.Publisher('cmd_vel', Twist)  # Send velocity commands to cmd_vel Topic
        rospy.spin()
        return





    def main(self):

        return


if __name__ == '__main__':
    green = GreenBot()
    green.main()

