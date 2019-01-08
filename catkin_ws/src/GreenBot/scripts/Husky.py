#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Should this be top import?
import roslib
roslib.load_manifest("GreenBot")


# NODES
# -------------------
# Joy pad Node
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
        rospy.init_node('Husky')  # Create the Husky node

        #  Initiations
        self.control_rate = 10  # TODO determine proper publish rate
        self.mux_twist = Twist()
        self.twist = Twist()

        # Subscriber Initiations
        rospy.Subscriber("husky/cmd_vel", Joy, self.callback_controller)  # cmd_vel
        # Publisher Initiations
        self.control_pub = rospy.Publisher("cmd_vel_topic", Twist, queue_size=10)

        return

# ------------------------------------------------------------------------------

    @staticmethod
    def callback_controller(data):
        if len(data) == 0:
            return  # No controller input

        # Convert data to Twist message
        twist = Twist()
        twist.linear.x = 4 * data.axes[1]  # Left stick vertical
        twist.angular.z = 4 * data.axes[0]  # Left stick horizontal
        return

# ------------------------------------------------------------------------------

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

    def control_mast(self):
        
        return

# ------------------------------------------------------------------------------
    def main(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            mast_command = self.control_mast()  # Check for mast command
            vel_command = self.compute_vel()  # Check for teleoperation command

            if vel_command.linear.x != 0 or vel_command.angular.z != 0:
                self.publish_move(vel_command)

            #rate.sleep()
        return
# ------------------------------------------------------------------------------
# END OF GreenBot CLASS
# ------------------------------------------------------------------------------


if __name__ == '__main__':
    green = GreenBot()
    green.main()

