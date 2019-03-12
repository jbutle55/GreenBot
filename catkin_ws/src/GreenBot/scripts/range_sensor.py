#!/usr/bin/env python
import rospy
from rospy_message_converter import message_converter
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import re
import serial
import time
import csv
import os

# Should this be top import?
import roslib
roslib.load_manifest("GreenBot")


# The class is used to manage a Node for an individual HC-SR04 ultrasonic sensor
# The Husky will use 4 of these sensors (1 @ each corner above the wheel)
class RangeSense:

    def __init__(self):
        rospy.init_node('Range_Sensor')  # Initiate range sensor node
        self.udoo_serial = self.setup_serial()  # Initiate serial communication with x86 Arduio

        # Publish sensor detection
        self.sensor_pub = rospy.Publisher('Range_Data', String, queue_size=10)

        self.sensed_range = None
        self.sensed_data = {'FL': None, 'FR': None, 'BL': None, 'BR': None}  # In cm

        return

# ------------------------------------------------------------------------------

    @staticmethod
    def setup_serial():
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        ser.flushOutput()

        return ser

# ------------------------------------------------------------------------------

    # Read the sensor output via Arduino digital pins
    def read_sensor(self):
        sense_data = {}

        # Use arduino to handle time keeping and time calculations
        # Use Braswell to manage 4 sensors results and publish data

        self.udoo_serial.write(b'Read Sensors')  # Signal Arduino to read sensors
        self.udoo_serial.read_until(b'Data Ready')  # Poll serial until ranges calculated
        raw_data = self.udoo_serial.read_until('Data Complete')  # Read serial until all data read

        # Separate raw data
        range_re = re.compile(r"FR:\s(?P<FR>\d+);\sBR:\s(?P<BR>\d+);\sFL:\s(?P<FL>\d+);"
                              r"\sBL:\s(?P<BL>\d+);$", re.MULTILINE)
        range_data = range_re.match(raw_data)

        for data in range_data.groupdict():
            sense_data[data] = range_data[data]

        # self.store_range_csv(sense_data)  # Store range data and time in the csv
        string_sense_data = message_converter.convert_dictionary_to_ros_message('std_msgs/String',
                                                                                sense_data)
        # String of dictionary of distance measured (in cm) from each sensor
        return string_sense_data

# ------------------------------------------------------------------------------

    def start(self):
        while not rospy.is_shutdown():
            self.sensed_range = self.read_sensor()  # Read the range sensors

            if self.sensed_range is not None:
                self.sensor_pub.publish(self.sensed_range)  # Publish sensor data to ultra topic

        return

# ------------------------------------------------------------------------------
# END OF RangeSense CLASS
# ------------------------------------------------------------------------------


if __name__ == '__main__':
    sensor = RangeSense()
    sensor.start()
