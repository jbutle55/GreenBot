#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np

# Should this be top import?
import roslib
roslib.load_manifest("GreenBot")


# ------------------------------------------------------------------------------
# HogDetector CLASS
# ------------------------------------------------------------------------------
class TeleOpHusky:
    def __init__(self):
        rospy.init_node('teleop_husky')
        rospy.Subscriber('joy', Joy, self.handle_joy)

        self.vel_cmd_pub = rospy.Publisher('cmd_vel', Twist)

        self.joy_vector = None
        self.joy_data = None

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

        return

# ------------------------------------------------------------------------------
    def handle_joy(self, joy_data):
        self.joy_data = joy_data
        self.override = True

        for button in self.override_buttons:
            if joy_data.buttons[button] == 0:
                self.override = False

        self.safe_motion = not self.override and joy_data.buttons[self.deadman_button] != 0

        x = joy_data.axes[1]
        y = joy_data.axes[0]
        joy_vector = np.array([x, y])
        joy_vector /= np.linalg.norm(joy_vector)
        joy_vector *= self.magnitude

        self.joy_vector = joy_vector

        return

# ------------------------------------------------------------------------------
    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            command = self.compute_motion_cmd()

            # Publish the most recent command
            if command is not None:
                self.vel_cmd_pub.publish(command)
            rate.sleep()

        return

# ------------------------------------------------------------------------------
    def compute_motion_cmd(self):
        command = Twist()

        if self.joy_data is None:
            command = None

        # Don't move if not touching thumb stick
        elif self.joy_data.axes[1] == 0.0 and self.joy_data.axes[0] == 0.0:
            command = None

        elif self.override:
            command = Twist()
            # TODO Double check axes order
            command.linear.x = self.joy_data.axes[0] * self.drive_scale
            command.angular.z = self.joy_data.axes[1] * self.turn_scale

        elif self.safe_motion:
            vector_sum = self.joy_vector
            vector_sum /= np.linalg.norm(self.joy_vector)

            joy_cmd_vector = np.array([self.joy_data.axes[1], self.joy_data.axes[0]])
            vector_sum *= np.linalg.norm(joy_cmd_vector)

            # Reduce reverse motion speed (Can't see behind us)
            if joy_cmd_vector[0] >= 0:
                vector_sum[0] = max(0, vector_sum)
            else:
                vector_sum[0] = max(-self.safe_reverse_speed, vector_sum[0])

            command = Twist()
            command.linear.x = vector_sum[0] * self.drive_scale
            command.angular.z = vector_sum[1] * -self.turn_scale
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
# END OF HogDetector CLASS
# ------------------------------------------------------------------------------


if __name__ == '__main__':
    cmd = TeleOpHusky()
    cmd.start()

