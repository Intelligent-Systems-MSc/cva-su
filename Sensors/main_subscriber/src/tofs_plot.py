#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# PFE - Autonomous vehicle
# Author : Tom DA SILVA-FARIA

import rospy
import sys 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from matplotlib.animation import FuncAnimation 
from std_msgs.msg import Int16MultiArray

class PlotTofs :

	def __init__(self, xlim, ylim) :
		self.fig, self.ax = plt.subplots(figsize=(6, 6))
		self.ln, = plt.plot([], [], '.b')
		self.x_data, self.y_data = [-1, 1, -1, 1], [1, 1] 
		self.xlim, self.ylim = [-xlim, xlim], [-ylim, ylim]
	
	def initPlot(self) :
		self.ax.set_xlim(self.xlim) ; self.ax.set_ylim(self.ylim)
		self.ax.set_xlabel("distance (dm)") ; self.ax.set_ylabel("distance (dm)")
		self.ax.set_title("Sensors plot")
		self.ax.grid(color="gray")
		self.ax.set_facecolor((0.0, 0.0, 0.0))
		self.fig.set_facecolor((207/255, 106/255, 4/255))
		return self.ln
		
	def callback(self, msg) : 
		scan = msg.data
		self.x_data, self.y_data = [-1, 1, -1, 1], [1, 1] 
		for i in range(len(scan)) : 
			value = scan[i] * 0.01 # Tofs scan messages are in mm
			self.y_data.append(value + self.y_data[i])
							
		assert len(self.y_data) == len(self.x_data)	
		
	def updatePlot(self, frame) :
		try : self.ln.set_data(self.x_data, self.y_data)
		except : print("[WRN] - No data got to be plotted")
		return self.ln
		
def listener(p) : 
	topic = rospy.get_param("sensors_datas", default="/TofsScan")
	rospy.Subscriber(topic, Int16MultiArray, p.callback)
	plt.show(block = True)
	
if __name__ == "__main__" :
	rospy.init_node("tofs_plot", anonymous=True)
	xlim, ylim = -10, 10
	p = PlotTofs(xlim, ylim)
	ani = FuncAnimation(p.fig, p.updatePlot, init_func = p.initPlot)
	sens_x, sens_y = [-1, 1], [1, 1]
	p.ax.plot(sens_x, sens_y, ".r")
	p.ax.add_patch(Rectangle((-1, -1), 2, 2))
	try : listener(p)
	except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit() 
	
