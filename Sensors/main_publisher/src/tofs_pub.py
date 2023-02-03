#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# PFE - Autonomous vehicle
# Author : Tom DA SILVA-FARIA

import rospy
import VL53L1X
import RPi.GPIO as GPIO
import time, sys

from std_msgs.msg import Int16MultiArray, Bool

class TofsPub :

	def __init__(self, addresses, xshuts, rate=100) :

		# ROS publisher
		self.tofs_scan = Int16MultiArray()
		self.tofs_pub  = rospy.Publisher("/TofsScan", Int16MultiArray, queue_size=1)

		# Setting up GPIO
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		for xshut in xshuts :
			GPIO.setup(xshut, GPIO.OUT)
			GPIO.output(xshut, False)

		# Setting up i2c addresses and tofs
		self.tofs = []
		for cpt, addr in enumerate(addresses) :
			GPIO.output(xshuts[cpt], True)
			self.tofs.append(VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29))
			self.tofs[cpt].open()
			self.tofs[cpt].change_address(new_address = addr)
			self.tofs[cpt].open()
			self.tofs[cpt].set_timing(66000, rate)
		rospy.loginfo("[INFO] -- {} tofs initialized".format(len(self.tofs)))
		for tof in self.tofs : tof.start_ranging(0)

		# Setting up keyboard entry for process aborting
		self.check_stop = False

	def publish_scan(self) :
		""" Publishing sensor scan """
		rospy.loginfo("[INFO] -- Sensors scan (distances in mm) are published in topic /TofsScan")
		while not self.check_stop :
			distances = [tof.get_distance() for tof in self.tofs]
			self.tofs_scan.data = distances
			self.tofs_pub.publish(self.tofs_scan)

			if self.check_stop :
				rospy.loginfo("[STOP] -- Aborting tofs scan publication")
				for tof in self.tofs : tof.stop_ranging()
				sys.exit()
				break

	def callback(self, msg) :
		""" Receiving Stop message -> endinf tofs scan publication """
		rospy.loginfo("[STOP] -- Stop message received")
		self.check_stop = True
		sys.exit()

def listener(p) :
	topic = rospy.get_param("stop_publisher", default="/StopPub")
	rospy.Subscriber(topic, Bool, p.callback)


if __name__ == "__main__" :
	rospy.init_node("tofs_pub", anonymous=True)
	addr = [0x27, 0x28, 0x2a]
	xshuts = [12, 16, 20]
	tofs_pub = TofsPub(addr, xshuts, rate=100)
	listener(tofs_pub)
	tofs_pub.publish_scan()
