#!/usr/bin/env python
import rospy
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
        rospy.init_node('range_sensor')  # Initiate range sensor node
        self.udoo_serial = self.setup_serial()  # Initiate serial communication with x86 Arduio
        self.create_csv()

        # TODO change topic name, topic type (Probably not Twist)
        # Publish sensor detection
        self.sensor_pub = rospy.Publisher('ultra_data', String, queue_size=10)

        self.sensed_range = None
        self.sensed_data = {'FL': None, 'FR': None, 'BL': None, 'BR': None}  # In cm

        return

# ------------------------------------------------------------------------------

    @staticmethod
    def setup_serial():
        ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
        ser.flushOutput()

        return ser

# ------------------------------------------------------------------------------
    @staticmethod
    def create_csv():
        if os.path.isfile('nav_data.csv'):  # Avoid overwriting existing data
            with open('nav_data.csv', 'a') as nav:
                writer = csv.writer(nav, delimeter=',')
                writer.writerow('')  # Write empty row
        else:
            with open('nav_data.csv', 'wb') as nav:
                writer = csv.writer(nav, delimeter=',')
                writer.writerow(['Time', 'FR', 'FL', 'BR', 'BL'])  # Create headers

        return

# ------------------------------------------------------------------------------

    @staticmethod
    def store_range_csv(range_dict):
        data = [time.time(), range_dict['FR'], range_dict['FL'], range_dict['BR'], range_dict['BL']]
        with open('nav_data.csv', 'a') as nav:  # Store the range data and time in an appended csv
            writer = csv.writer(nav, delimeter=',')
            writer.writerow(data)  # Write data

        return
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
        range_re = re.compile(r"(?P<position>\w+): (?P<range>\d+)")
        range_data = range_re.match(raw_data)

        for data in range_data.groupdict():
            sense_data[data] = range_data[data]

        self.store_range_csv(sense_data)  # Store range data and time in the csv

        # TODO might be able to just return range_data.groupdict()?
        return sense_data  # Dictionary of distance measured (in cm) from each sensor

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
