#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

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

        # Subscriber calls
        rospy.Subscriber("cmd_vel", Joy, self.callback_controller)
        # rospy.Subscriber('cmd_vel', Twist, self.callback_mux)

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

    def callback_mux(self, data):
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


if __name__ == '__main__':
    green = GreenBot()
    green.main()

