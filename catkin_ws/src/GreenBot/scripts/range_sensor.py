#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np
import re
import serial

# Should this be top import?
import roslib
roslib.load_manifest("GreenBot")


# The class is used to manage a Node for an individual HC-SR04 ultrasonic sensor
# The Husky will use 4 of these sensors (1 @ each corner above the wheel)
class RangeSense:

    def __init__(self):
        rospy.init_node('range_sensor')  # Initiate range sensor node
        self.udoo_serial = self.setup_serial()  # Initiate serial communication with x86 Arduio

        # TODO change topic name, topic type (Probably not Twist)
        self.sensor_pub = rospy.Publisher('topic', Twist, queue_size=10)  # Publish sensor detection

        self.sensor_i2c = None
        self.sensed_range = None
        self.sensed_status = False
        self.sensed_data = {'FL': None, 'FR': None, 'BL': None, 'BR': None}  # In cm

        return

# ------------------------------------------------------------------------------

    @staticmethod
    def setup_serial():
        ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
        ser.flushOutput()

        return ser

# ------------------------------------------------------------------------------

    # Read the sensor output via arduino digital pins
    def read_sensor(self):
        raw_data = None
        sense_data = {}

        # Use arduino to handle time keeping and time calculations
        # Use Braswell to manage 4 sensors results and publish data

        self.udoo_serial.write(b'Read Sensors')  # Signal arduino to read sensors
        self.udoo_serial.read_until('Data Ready')  # Poll serial until ranges calculated
        raw_data = self.udoo_serial.read_until('Data Complete')  # Read serial until all data read

        # Seperate raw data
        range_re = re.compile(r"(?P<position>\w+): (?P<range>\d+)")
        range_data = range_re.match(raw_data)

        for data in range_data.groupdict():
            sense_data[data] = range_data[data]

        # TODO might be able to just return range_data.groupdict()?

        return sense_data  # Dictionary of distance measured (in cm) from each sensor

# ------------------------------------------------------------------------------

    def start(self):
        while not rospy.is_shutdown():
            self.sensed_range = self.read_sensor()  # Read the range sensors

            if self.sensed_range is not None:
                self.sensor_pub.publish(self.sensed_range)

        return

# ------------------------------------------------------------------------------
# END OF RangeSense CLASS
# ------------------------------------------------------------------------------


if __name__ == '__main__':
    sensor = RangeSense()
    sensor.start()
