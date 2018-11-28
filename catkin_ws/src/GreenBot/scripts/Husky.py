#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import argparse

# Should this be top import?
import roslib
roslib.load_manifest("GreenBot")


# NODES
# -------------------
# Control pad Node
# Husky Node (MASTER)
# Mux Node
# QR Code node
# Ultrasound/laser sensor Node
# Front camera Node
# Remote laptop Node

# ------------------------------------------------------------------------------
# GreenBot CLASS
# ------------------------------------------------------------------------------
class GreenBot:
    """
    A node for the Husky A200
    """

    def __init__(self):
        self.setup_ros_node()
        self.control_rate = 30  # TODO determine proper publish rate (Control_rate) below
        # Publish on husky_velocity_controller/cmd_vel topic
        self.control_pub = rospy.Publisher("cmd_vel_topic", Twist)
        # husky_velocity_controller/cmd_vel instead of vel_topic

        self.mux_twist = Twist()
        self.twist = Twist()

        return

# ------------------------------------------------------------------------------
    def setup_ros_node(self):
        rospy.init_node('Husky', anonymous=True)  # Create the Husky node

        # Subscriber initiations
        # self.subscribe_node("controller")  # Joystick controller Node
        self.subscribe_node("mux")  # Mux Node
        self.subscribe_node("prox")  # Proximity sensor Node
        self.subscribe_node("vid")  # Video stream Node
        self.subscribe_node('barcode')  # Barcode detector Node

        return

# ------------------------------------------------------------------------------
    def subscribe_node(self, target):
        if target is "controller":  # TODO Does sub mux replace this?
            rospy.Subscriber("husky/cmd_vel", Joy, self.callback_controller)
        elif target is "prox":
            # TODO Create proximity sensor sub once using sensors
            pass
        elif target is "mux":  # TODO Is mux better to use?
            rospy.Subscriber('cmd_vel', Twist, self.callback_mux)
        elif target is "vid":
            pass
        elif target is "people":  # Detected person in auto mode
            # TODO Implement people detector sub once developed
            pass
        elif target is "barcode":
            # TODO Implement barcode sub once developed
            pass
        return

    @staticmethod
    def callback_controller(data):
        if len(data) == 0:
            return  # No controller input

        # Convert data to Twist message
        twist = Twist()
        twist.linear.x = 4 * data.axes[1]  # Left stick vertical
        twist.angular.z = 4 * data.axes[0]  # Left stick horizontal

        return

    @staticmethod
    def callback_mux(data):
        if len(data) == 0:
                return  # No data input

        self.mux_twist.linear.x = data[0]
        self.mux_twist.angular.z = data[1]

        return

# ------------------------------------------------------------------------------
    @staticmethod
    def callback_prox(data):
        # TODO Implement proximity data breakout
        pass
        return

# ------------------------------------------------------------------------------
    def publish_move(self, command):
        # Receive Twist type command and publish to Husky controller
        self.control_pub.publish(command)
        return

# ------------------------------------------------------------------------------
    def compute_vel(self):
        cmd = Twist()

        if self.mux_twist is not None:
            cmd.linear.x = self.mux_twist.linear.x
            cmd.angular.z = self.mux_twist.angular.z

        elif self.twist is not None:
            cmd.linear.x = self.twist.linear.x
            cmd.angular.z = self.twist.angular.z

        return cmd

# ------------------------------------------------------------------------------
    def main(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            command = self.compute_vel()

            if command.linear.x != 0 or command.angular.z != 0:
                self.publish_move(command)

        rate.sleep()

        return
# ------------------------------------------------------------------------------
# END OF GreenBot CLASS
# ------------------------------------------------------------------------------


def run_greenbot(args):
    config = None
    if args.mode is 'tele':
        # Add dict keys relevant to teleoperation mode
        pass
    else:
        # Add dict keys relevant to autonomous mode
        pass
    green = GreenBot(config)
    green.main()
    return


if __name__ == '__main__':
    # parser = argparse.ArgumentParser('Determine the mode the GreenBot should start in.')
    # parser.add_argument('mode', choices=['tele', 'auto'],
    #                    help="Enter 'Tele' to run GreenBot in teleoperation mode. Enter 'auto' for"
    #                         "the GreenBot to autonomously patrol the greenhouse.")
    # arguments = parser.parse_args()
    # sys.exit(run_greenbot(arguments))
    green = GreenBot()
    green.main()

