#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np
import serial
from smbus2 import SMBusWrapper


# Should this be top import?
import roslib
roslib.load_manifest("GreenBot")


# The class used to manage a Node for an individual Modern Robotics Range Sensor
# The Husky will use 4 of these sensors
class RangeSense:

    def __init__(self):
        rospy.init_node('range_sensor')  # Initiate range sensor node

        # TODO change topic name, topic type (Probably not Twist)
        self.sensor_pub = rospy.Publisher('topic', Twist, queue_size=10)  # Publish sensor detection

        self.i2c_addresses = {'FL': '0x28', 'FR': '0x', 'BL': '0x', 'BR': '0x'}
        self.sensor_id = '0x55'
        self.sensor_registers = {'ultrasonic': '0x04', 'optical': '0x05'}

        self.sensor_i2c = None
        self.sensed_range = None
        self.sensed_status = False
        self.sensed_data = {'FL': None, 'FR': None, 'BL': None, 'BR': None}  # In cm

        return

# ------------------------------------------------------------------------------

    # Read the sensor output via I2C and determine range
    def read_sensor(self):
        raw_data = None
        sense_data = {}

        # Read ultrasonic register first then optical # TODO swap order for better safety?
        # The ultrasonic register value is in cm and is linear
        # The optical sensor is exponential

        for position, address in self.i2c_addresses:
            for measurement_type, reg in self.sensor_registers:
                # Read register
                # TODO implement I2C reading
                with SMBusWrapper(1) as bus:  # TODO Change 1 to actual i2c bus number
                    raw_data = bus.read_i2c_block_data(address, reg, 16)  # TODO Change length from 16?

                if measurement_type is 'ultrasonic' and 5 <= raw_data < 255:
                    sense_data[position] = raw_data
                elif measurement_type is 'optical' and 0 < raw_data < 5:
                    # TODO determine optical exponential curve
                    pass
            # TODO determine topic type and format to be used in publish()

        return sense_data  # Dictionary of distance measured (in cm) from each sensor

# ------------------------------------------------------------------------------

    def start(self):
        while not rospy.is_shutdown():
            self.sensed_range = self.read_sensor()

            if self.sensed_range is not None:
                self.sensor_pub.publish(self.sensed_range)

        return

# ------------------------------------------------------------------------------
# END OF RangeSense CLASS
# ------------------------------------------------------------------------------


if __name__ == '__main__':
    sensor = RangeSense()
    sensor.start()
