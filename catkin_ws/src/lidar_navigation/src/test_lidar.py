#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet TX : Voiture autonome
@Author : Tom DA SILVA-FARIA
"""

import rospy
import sys
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

class LaserTest :
	def __init__(self) :
		self.test_data = Float32MultiArray()
		self.pub = rospy.Publisher("/TestLidar", Float32MultiArray, queue_size = 1)

	def get_scan(self, msg) :
		self.test_data.data = msg.ranges
		self.pub.publish(self.test_data)

def listener(t) :
	topic = rospy.get_param("topic_scan", default="/scan")
	rospy.Subscriber(topic, LaserScan, t.get_scan)
	rospy.spin()

if __name__=="__main__" :
	rospy.init_node('lidar_data', anonymous = True)
	t = LaserTest()
	try : listener(t)
	except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit()



