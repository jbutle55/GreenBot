#!/usr/bin/env python


import sys, os
import husky_msgs
import rospy
import roslaunch
import theora_image_transport
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import twist_mux_msgs
import argparse

# Should this be top import?
import roslib
roslib.load_manifest("GreenBot")


# NODES
# ---------------------------------------
# Control pad Node
# Husky Node
# Mux Node
# QR Code node
# Ultrasound/laser sensor Node
# Front camera Node


# ------------------------------------------------------------------------------
class GreenBot:
    """A node for the Husky A200
    Subscribe to mux, QR code detection, and rangefinder sensors
    Publish velocity commands
    """

    def __init__(self, config):
        self.__dict__.update(config)
        self.detection_timeout = rospy.Duration(self.detection_timeout)
        self.setup_ros_node()

        # Publish on husky_velocity_controller/cmd_vel topic
        self.control_pub = rospy.Publisher('husky_velocity_controller/cmd_vel', Twist)

        return

# ------------------------------------------------------------------------------
    def setup_ros_node(self):
        rospy.init_node('Husky', anonymous=True)  # Create the Husky node

        # Subscriber initiations
        self.subscribe_node("controller")
        self.subscribe_node("mux")
        self.subscribe_node("prox")
        self.subscribe_node("vid")

        return

# ------------------------------------------------------------------------------
    def subscribe_node(self, target):
        if target is "controller":  # Does sub mux replace this?
            rospy.Subscriber(self.controller_topic, Joy, self.callback_controller)
        elif target is "prox":
            # TODO Create proximity sensor sub once using sensors
            pass
        elif target is "mux":
            rospy.Subscriber('cmd_vel', Twist, self.callback_mux)
        elif target is "vid":
            rospy.Subscriber()
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

        mux_twist = Twist()
        mux_twist.linear.x = data[0]
        mux_twist.angular.z = data[1]

        return mux_twist

    @staticmethod
    def callback_prox(data):
        pass
        return

    # Publisher of velocity commands to the GreenBot motor system
    @staticmethod
    def pub_cmd_vel():
        # Send velocity commands to cmd_vel Topic
        rospy.Publisher('husky_velocity_controller/cmd_vel', Twist)
        return

# ------------------------------------------------------------------------------
    def publish_move(self, linear, angular):
        
        return

# ------------------------------------------------------------------------------
    def compute_vel(self):
        linear_cmd, angular_cmd = 0, 0

        return linear_cmd, angular_cmd

# ------------------------------------------------------------------------------
    def main(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            linear, angular = self.compute_vel()

            if linear != 0 or angular != 0:
                self.publish_move(linear, angular)

        rate.sleep()

        return


def run_greenbot(args):
    # Standard config dict for both operation modes
    config = dict(vel_topic=rospy.get_param("~cmd_vel_topic", "husky/plan_cmd_vel"),
                  drive_rate=rospy.get_param("~drive_rate", 1.0),
                  turn_rate=rospy.get_param("~turn_rate", 0.5),
                  control_rate=rospy.get_param("~control_rate", 30),  # 30Hz publish rate
                  controller_topic=rospy.get_param("joystick"))  # ~?

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
    parser = argparse.ArgumentParser('Determine the mode the GreenBot should start in.')
    parser.add_argument('mode', choices=['tele', 'auto'],
                        help="Enter 'Tele' to run GreenBot in teleoperation mode. Enter 'auto' for"
                             "the GreenBot to autonomously patrol the greenhouse.")
    arguments = parser.parse_args()
    sys.exit(run_greenbot(arguments))

