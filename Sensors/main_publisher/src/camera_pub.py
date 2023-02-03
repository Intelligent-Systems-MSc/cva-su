#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# PFE - Autonomous vehicle
# Author : Tom DA SILVA-FARIA

import rospy
import numpy as np
import time, sys
import cv2

from picamera import PiCamera
from picamera.array import PiRGBArray
from PIL import Image
from sensor_msgs.msg import Image as SensorImage
from std_msgs.msg import Bool

class CamPub :

	def __init__(self, w, h, fr) :

		self.w, self.h, self.fr = w, h, fr
		self.check_stop = False

		rospy.loginfo("[INFO] -- Camera init. : {}x{}px -- Frame rate : {}".format(self.w, self.h, self.fr))
		self.camera = PiCamera()
		self.camera.resolution = (self.w, self.h)
		self.camera.framerate = self.fr
		self.raw_capture = PiRGBArray(self.camera, size=(self.w, self.h))
		time.sleep(0.1)
		rospy.loginfo("[INFO] -- Camera initialisation done.")

		self.image_data = SensorImage()
		self.image_pub  = rospy.Publisher("/ImageScan", SensorImage, queue_size=1)

	def set_white_balance(self, nb_iter=30) :
		""" Automatically setting camera white balance """
		rospy.loginfo("[INFO] -- Setting up camera white-balance ...")
		self.camera.awb_mode = 'off'
		rg, bg = 0.5, 0.5
		self.camera.awb_gains = (rg, bg)
		with PiRGBArray(self.camera, size=(self.w, self.h)) as output :
			for i in range(nb_iter) :
				self.camera.capture(output, format='rgb', use_video_port=True)
				r, g, b = (np.mean(output.array[..., i]) for i in range(3))
				if abs(r-g) > 2 :
					if r > g : rg -= 0.1
					else : rg += 0.1
				if abs(b-g) > 1 :
					if b > g : bg -= 0.1
					else : bg += 0.1
				self.camera.awb_gains = (rg, bg)
				output.seek(0)
				output.truncate()

		rospy.loginfo("[INFO] -- White balancing done.")

	def publish_scan(self) :
		""" Publishing image array in ROS topic at rate : self.fr """
		rospy.loginfo("[INFO] -- Images are published in topic : /ImageScan")
		for frame in self.camera.capture_continuous(self.raw_capture, format="bgr", use_video_port=True) :
			image = np.array(frame.array, dtype=np.uint8)
			image = Image.fromarray(image)
			image = image.convert('RGB')
			self.image_data.header.stamp = rospy.Time.now()
			self.image_data.height, self.image_data.width = image.height, image.width
			self.image_data.encoding = "rgb8"
			self.image_data.is_bigendian = False
			self.image_data.step = 3 * image.width
			self.image_data.data = np.array(image).tobytes()
			self.image_pub.publish(self.image_data)
			self.raw_capture.truncate(0)
			if self.check_stop :
				rospy.loginfo("[STOP] -- Aborting images publication")
				sys.exit()
				break

	def callback(self, msg) :
		""" Receiving Stop message -> ending images publication """
		rospy.loginfo("[STOP] -- Stop message received")
		self.check_stop = True
		sys.exit()

def listener(p) :
	topic = rospy.get_param("stop_publisher", default="/StopPub")
	rospy.Subscriber(topic, Bool, p.callback)

if __name__ == "__main__" :

	rospy.init_node("camera_pub", anonymous=True)
	width  = rospy.get_param("image_width", default=160)
	height = rospy.get_param("image_height", default=128)
	framerate = rospy.get_param("frame_rate", default=32)

	cam_pub = CamPub(width, height, framerate)
	cam_pub.set_white_balance(nb_iter=30)
	listener(cam_pub)
	cam_pub.publish_scan()
