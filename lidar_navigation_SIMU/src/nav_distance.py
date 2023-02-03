#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet TX : Voiture autonome
@Author : Tom DA SILVA-FARIA Yuwang Chen
"""

import rospy
import sys
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

class LaserScanner :

	def __init__(self, nbAngles, limAngle, wide, distMax, distMin) :
		self.nbAngles = nbAngles
		self.limAngle = limAngle
		self.wide = wide
		self.distMax = distMax
		self.distMin = distMin
		self.moyennes = Float32MultiArray() # deux derniere distance a gauche et a droit de la route 
		self.pub = rospy.Publisher("/ScannerData", Float32MultiArray, queue_size = 1)

		self.cpt = 0

	def get_scan(self, msg) :
		scan = msg.data

		if(self.cpt == 0) :
			self.limAngle = int((len(scan)/360)*self.limAngle)
			self.angle_route = np.array([int(len(scan)/4),int(len(scan)*3/4)]) # direction perpendiculaire a vehicule 90 et 270 degree
			listePos = np.arange(int(len(scan)/2)-self.limAngle, int(len(scan)/2))
			listeNeg = np.arange(int(len(scan)/2), int(len(scan)/2)+self.limAngle)
			self.listeAngles = np.concatenate((listePos, listeNeg))
			self.listeAngles = self.listeAngles[::int(len(self.listeAngles)/self.nbAngles)]

			listeAnglesDeg = []
			for i in range(len(self.listeAngles)) : listeAnglesDeg.append(int(self.listeAngles[i]/(((len(scan))/360))))

			rospy.set_param("angles", listeAnglesDeg)

		self.moyennes.data = [0]*(len(self.listeAngles)+len(self.angle_route))

		# distance à gauche et a droite de la route
		for i in range(len(self.angle_route)) :
			cptMoy = 0
			for j in range(self.wide) :
				try :
					data = scan[self.angle_route[i]-1-j]
					if data <= self.distMax and data >= self.distMin :
						self.moyennes.data[-2+i] += data
						cptMoy += 1
				except : self.cpt = 0

			try : self.moyennes.data[-2+i] = self.moyennes.data[-2+i] / cptMoy
			except : self.moyennes.data[-2+i] = 0

		# distance de direction dans l'angle limite
		for i in range(len(self.listeAngles)) :
			cptMoy = 0
			for j in range(self.wide) :
				try :
					data = scan[self.listeAngles[i]-1-j]
					if data <= self.distMax and data >= self.distMin :
						self.moyennes.data[i] += data
						cptMoy += 1
				except :
					self.cpt = 0
			try :
				self.moyennes.data[i] = self.moyennes.data[i] / cptMoy
			except :
				self.moyennes.data[i] = 0

		self.pub.publish(self.moyennes)
		self.cpt += 1

def listener(s) :
	topic = rospy.get_param("topic_scan", default="/LidarScan")
	rospy.Subscriber(topic, Float32MultiArray, s.get_scan)
	rospy.spin()


if __name__ == '__main__' :
	rospy.init_node('lidar_data', anonymous = True)

	nbAngles = rospy.get_param("nb_angles", default=16) # Nombre d'angles étudiés
	limAngle = rospy.get_param("lim_angle", default=90) # Analyse de -limAngle à limAngle (en degrés)
	wide = rospy.get_param("wide", default=4) # Nombre de points utilisés pour une moyenne 

	# Seuils de distances pour lesquels on ne prend plus les données en compte :
	distMax = rospy.get_param("dist_max", default=10)
	distMin = rospy.get_param("dist_min", default=0.15)

	s = LaserScanner(nbAngles, limAngle, wide, distMax, distMin)

	try : listener(s)
	except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit()
