#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA
"""

import rospy
from std_msgs.msg import Float32, String

class Info :

    def __init__(self) :
        self.speed = 0
        self.angle = 0
        self.mess = ""
        self.speed_rq = False
        self.angle_rq = False
        self.mess_rq = False

        # Init ROS node
        rospy.init_node('sub_topic_info', anonymous=True)
        # Init ROS subscribers
        rospy.Subscriber("/SpeedCommand", Float32, self.callback_speed)
        rospy.Subscriber("/AngleCommand", Float32, self.callback_angle)
        rospy.Subscriber("/TofsNavMessage", String, self.callback_mess)
    
    def callback_mess(self, msg) :
        self.mess = msg.data
        self.mess_rq = True
        self.affiche()

    def callback_speed(self, msg) :
        self.speed = msg.data
        self.speed_rq = True
        self.affiche()
    
    def callback_angle(self, msg) :
        self.angle = msg.data
        self.angle_rq = True
        self.affiche()
    
    def affiche(self, wait_for_all = True) :
        if wait_for_all :
            if self.speed_rq and self.angle_rq and self.mess_rq :
                print(f"Speed : %.2f \t Angle : %.2f \t Mess : {self.mess}" % (self.speed, self.angle))
                self.speed_rq = False
                self.angle_rq = False
                self.mess_rq = False
        else :
            print(f"Speed : {self.speed} | Angle : {self.angle}")


if __name__ == "__main__" :

    info = Info()
    rospy.spin()