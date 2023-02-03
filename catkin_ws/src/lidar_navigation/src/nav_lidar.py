#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet TX : Voiture autonome
@Author : Tom DA SILVA-FARIA Yuwang Chen

"""

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool, Float32, String

class Navigation :

	def __init__(self, zero, leftAngle, rightAngle, vMin, vMax, seuil, seuil_obs,seuil_voisin, freq, l_lim, r_lim, coeff_correction) :

		self.pub2 = rospy.Publisher("/SpeedCommand", Float32, queue_size=1)
		self.pub3 = rospy.Publisher("/AngleCommand", Float32, queue_size=1)
		self.vit = 0
		self.dir = 0

		self.zero  = zero
		self.l_lim = l_lim
		self.r_lim = r_lim

		self.freq = freq
		self.cpt = 0
		self.vMin = vMin
		self.vMax = vMax
		self.seuil = seuil # Distance au delà de laquelle on va à vMax
		self.seuil_obs = seuil_obs
		self.seuil_voisin = seuil_voisin

	def navigate(self, msg) :
		self.angles = rospy.get_param("angles", default = [0])
		self.listeDirections = np.linspace(-1, 1, len(self.angles))
		list_sort = np.argsort(msg.data[0:-2])
		maxi = list_sort[-1]
		indice_maxi = -1
		while maxi == 0 or maxi == len(self.angles)-1:
			indice_maxi += -1
			maxi = list_sort[indice_maxi]
		maxi_temp = maxi
		mini = list_sort[0]
		coeff_vitesse = 1
		angle_correction = 0
		angle_correction_centre = 0
		if msg.data[int(len(msg.data)/2)-1] > self.seuil and msg.data[int(len(msg.data)/2)-2] > self.seuil_voisin and msg.data[int(len(msg.data)/2)] > self.seuil_voisin: 
			angle_correction_centre = 0.5*np.tanh(msg.data[-1]-msg.data[-2])
			if(angle_correction <= 0) : self.dir = max(-1, self.zero + angle_correction_centre)
			else : self.dir = min(1, self.zero + angle_correction_centre)

		else :
			while (msg.data[maxi-1] < self.seuil_voisin or msg.data[maxi+1] < self.seuil_voisin) and indice_maxi > -10 : 
				indice_maxi += -1
				maxi = list_sort[indice_maxi]
				while maxi == 0 or maxi == len(self.angles)-1:
					indice_maxi += -1
					maxi = list_sort[indice_maxi]
			if indice_maxi == -11 :
				maxi = maxi_temp

			angle_correction = 0.1*np.tanh(msg.data[maxi+1]-msg.data[maxi-1])
			if(msg.data[mini] <= self.seuil_obs) :
				direction = self.listeDirections[maxi] - self.listeDirections[mini]*coeff_correction+angle_correction
				coeff_vitesse = np.exp(abs(direction))
				if direction <= self.r_lim   : direction = -1
				elif direction >= self.l_lim : direction = 1
				self.dir= direction
			else :

				direction = self.listeDirections[maxi]+angle_correction
				coeff_vitesse = np.exp(abs(direction))
				if direction <= self.r_lim   : direction = -1
				elif direction >= self.l_lim : direction = 1
				self.dir= direction

		if msg.data[int(len(msg.data)/2)-1] >= self.seuil/2 : vitesse = 1.0
		else : vitesse = (msg.data[int(len(msg.data)/2)-1]/self.seuil)/(10*(1+coeff_vitesse))
		if self.vit < self.vMin+0.1 :
			if(self.dir <= 0) : self.dir = max(-1, 4*self.dir)
			else : self.dir = min(1, 4*self.dir)

		self.vit = (self.vMax - self.vMin)*vitesse + self.vMin
		
		if(self.cpt == self.freq) :
			self.pub2.publish(self.vit)
			self.pub3.publish(self.dir)
			self.cpt = 0

		self.cpt += 1

def listener(n) :
	# Topic ScannerData : contient les moyennes de distances sur différents angles 
	topic = rospy.get_param("topic_data", default="/ScannerData")
	rospy.Subscriber(topic, Float32MultiArray, n.navigate)
	rospy.spin()

if __name__ == '__main__' :
	rospy.init_node('lidar_nav', anonymous = True)

	# Paramètres pour l'Arduino :
	zero = rospy.get_param("servo_zero", default=0)
	left = rospy.get_param("servo_left", default=1)       # Valeur pour le servo : max turn left
	right = rospy.get_param("servo_right", default=-1)     # Valeur pour le servo : max turn right

	# Limites d'angle au delà desquelles on décide de braquer à fond 
	l_lim = rospy.get_param("left_limite",  default=-0.6) 
	r_lim = rospy.get_param("right_limite", default=0.6)
	vMax = rospy.get_param("esc_vmax", default=1)       # Valeur pour l'ESC : vitesse maxi
	vMin = rospy.get_param("esc_vmin", default=0)       # Valeur pour l'ESC : vitesse mini

	# Seuil de distance au delà de laquelle on estime qu'on peut aller tout droit
	seuil = rospy.get_param("seuil_distance", default=2.0)  
	seuil_obs = rospy.get_param("seuil_obstacle", default=0.3)
	seuil_voisin = rospy.get_param("seuil_voisin", default=0.9) #seuil pour si la direction choisi est "serre" ou pas 

	# Plus freq augmente, MOINS on publie de données (contre-intutif .... :p)
	freq = rospy.get_param("frequence_nav", default=10)
	coeff_correction = rospy.get_param("correction_coeff", default = 0.35)

	n = Navigation(zero, left, right, vMin, vMax, seuil, seuil_obs,seuil_voisin, freq, l_lim, r_lim, coeff_correction)

	try : listener(n)
	except (rospy.ROSInterruptException, KeyboardInterrupt): rospy.sleep(10) ; sys.quit()
