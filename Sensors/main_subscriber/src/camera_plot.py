#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# PFE - Autonomous vehicle
# Author : Tom DA SILVA-FARIA

import rospy
import sys, io
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImagePlot :

	def __init__(self, w, h) :
		self.fig, self.ax = plt.subplots(figsize=(7, 4))
		self.w, self.h = w, h
		self.image = np.zeros((self.h, self.w))
		self.ln = plt.imshow(self.image)
		
		self.cv_bridge = CvBridge()

	def initPlot(self)  :
		self.ax.set_title("Camera video")
		self.fig.set_facecolor((207/255, 106/255, 4/255))
		return self.ln

	def callback(self, msg) :
		scan = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
		self.image = scan
		#self.image = np.array(scan).reshape((self.h, self.w, 4))

	def updatePlot(self, frame) :
		self.ln.set_data(self.image)
		return self.ln

def listener(p) :
	topic = rospy.get_param("image_datas", default="/ImageScan")
	rospy.Subscriber(topic, SensorImage, p.callback)
	plt.show(block=True)

if __name__ == "__main__" :
	rospy.init_node('camera_plot', anonymous = True)
	w = rospy.get_param("image_width", default=640)
	h = rospy.get_param("image_height", default=480)
	p = ImagePlot(w, h)

	ani = FuncAnimation(p.fig, p.updatePlot, init_func = p.initPlot)
	try : listener(p)
	except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit()
