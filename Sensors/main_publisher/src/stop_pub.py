#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# PFE - Autonomous vehicle
# Author : Tom DA SILVA-FARIA

import rospy
import threading, sys

from std_msgs.msg import Bool

class KeyboardThread(threading.Thread):

    def __init__(self, input_cbk = None, name='keyboard-input-thread'):
        self.input_cbk = input_cbk
        super(KeyboardThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            self.input_cbk(input("-> 'quit' (or 'q') : killing every process\n-> 'stop' (or 's') : stopping vehicle\n-> 'start' (or 'd') : restarting vehicle after a 'pause' command\nEnter command here : ")) #waits to get input + Return


class StopPub :

	def __init__(self) :
		self.stop_data, self.pause_data = Bool(), Bool()
		self.stop_data.data, self.pause_data.data = False, False
		self.stop_pub  = rospy.Publisher("/StopPub", Bool, queue_size=1)
		self.pause_pub = rospy.Publisher("/PausePub", Bool, queue_size=1)
		self.kthread = KeyboardThread(self.keyboard_stop)

	def keyboard_stop(self, inp) :
		""" Press return button to send stop message """
		if(inp == "quit" or inp== "QUIT" or inp=="q" or inp=="Q") :
			rospy.loginfo("[STOP] -- Sending stop message\n")
			self.stop_data.data, self.pause_data.data = True, True
			self.stop_pub.publish(self.stop_data)
			self.pause_pub.publish(self.pause_data)
			sys.exit()
		elif(inp == "stop" or inp=="STOP" or inp=="s" or inp=="S") :
			rospy.loginfo("[PAUSE] -- Stopping vehicle\n")
			self.pause_data.data = True
			self.pause_pub.publish(self.pause_data)
		elif(inp == "start" or inp=="START" or inp=="d" or inp=="D") :
			rospy.loginfo("[START] -- Starting vehicle\n")
			self.pause_data.data = False
			self.pause_pub.publish(self.pause_data)

if __name__ == "__main__" :

	rospy.init_node("stop_pub", anonymous=True)
	stop_pub = StopPub()
	rospy.spin()
