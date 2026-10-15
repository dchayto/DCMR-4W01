#!/usr/bin/env python3

#								imu_handler.py							
#	ros node for interfacing with onboard imu
#
#	node will take in current imu values (read into pi from imu)
#
#	pose will be published on imu data topic, to be consumed by sensor
#	fusion node as part of estimating actual robot pose/motion 
#
#	need to use virtual environment to use pip PM; need to source VE before
#	building IMU package
#	python3 -m venv --system-site-packages ~/imu_venv
#	source ~/imu_venv/bin/activate
#
#	note: need to make sure BNO08x and other adafruit libraries installed:
#	pip install adafruit-blinka
#	pip install adafruit-circuitpython-bno08x
#	pip install adafruit-extended-bus
#
#	need some rpi libraries as well:
#	pip install RPi.GPIO
#
#	also need to add the following to /boot/firmware/config.txt and reboot:
#	dtoverlay=i2c-gpio,i2c_gpio_sda=2,i2c_gpio_scl=3,bus=8
#	wiring up to regular i2c sda and scl lines, but doing software i2c to get
#	around known hardware bug with sensor. note that numbers above are BCM
#	numbering, not wiringpi or J8 header standard numbering
#
#
#	stole a lot of this from chatGPT ngl, just tweaking as required	


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_extended_bus import ExtendedI2C as I2C


class BNO085Node(Node):
    def __init__(self):
        super().__init__('imu_node')
        i2c = I2C(1)						# hardware I2C on bus 1
        self.bno = BNO08X_I2C(i2c)
        report_interval = 10_000			# 100 Hz

        self.bno.enable_feature(
            adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR,
            report_interval
        )

        self.bno.enable_feature(
            adafruit_bno08x.BNO_REPORT_GYROSCOPE,
            report_interval
        )

        self.bno.enable_feature(
            adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION,
            report_interval
        )

        self.publisher = self.create_publisher(
            Imu,
            '/imu/data',
            1
        )

        self.timer = self.create_timer(
            0.01,  # 100 Hz
            self.poll_imu
        )

    def poll_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # COVARIANCES:
        # set somewhat arbitrarily, figure out more reasonable values later
        msg.orientation_covariance = [
        #	x		y		z
            3e-3,	0.0,	0.0,	# x	
            0.0,	0.0,	0.0,	# y	
            0.0,	0.0,	0.0,	# z	
        ]	# magnetometer + onboard fusion

        msg.angular_velocity_covariance = [
        #	x		y		z
            2e-4,	0.0,	0.0,	# x	
            0.0,	2e-4,	0.0,	# y	
            0.0,	0.0,	2e-4	# z	
        ]	# gyro + onboard fusion

		# weighting y accelerometer readings higher - wheels slip along roller
		# axis, can't trust encoder readings.
        msg.linear_acceleration_covariance = [
        #	x		y		z
            3e-2,	0.0,	0.0,	# x	
            0.0,	3e-2,	0.0,	# y	
            0.0,	0.0,	4e-2,	# z	
        ]	# accelerometer + onboard fusion


        # Orientation
        x, y, z, w = self.bno.quaternion

        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w

        # Angular velocity
        x, y, z = self.bno.gyro

        msg.angular_velocity.x = x
        msg.angular_velocity.y = y
        msg.angular_velocity.z = z

        # Linear acceleration
        x, y, z = self.bno.linear_acceleration

        msg.linear_acceleration.x = x
        msg.linear_acceleration.y = y
        msg.linear_acceleration.z = z

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = BNO085Node()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
