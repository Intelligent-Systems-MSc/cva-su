#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32

class Navigation() : 

    def __init__(self) :
        self.speed = 0.0
        self.angle = 0.0
        self.nav_tofs = {"speed" : 0.0, "angle" : 0.0}

        # Init ROS node
        rospy.init_node('navigation_haut_niveau', anonymous=True)

        # Init ROS publishers
        self.pub_speed = rospy.Publisher("/SpeedCommand", Float32, queue_size = 1)
        self.pub_angle = rospy.Publisher("/AngleCommand", Float32, queue_size = 1)

        # Init ROS subscribers
        self.sub_nav_tofs = rospy.Subscriber("/TofsSpeedAngleCommand", Float32MultiArray, self.callback_tofs)
        # other subscribers should be added here


    def callback_tofs(self, msg) :
        self.nav_tofs["speed"] = msg.data[0]
        self.nav_tofs["angle"] = msg.data[1]

    def set_speed_angle(self, speed, angle) : 
        self.speed = speed
        self.angle = angle
        self.pub_speed.publish(self.speed)
        self.pub_angle.publish(self.angle)


    def run(self) :
        """ Main loop of the navigation"""

        # frequency of the loop
        HZ = 25 # Hz
        rate = rospy.Rate(HZ)

        # main loop
        while not rospy.is_shutdown() :
            
            # set the new speed and angle and publish the message
            self.set_speed_angle(self.nav_tofs["speed"], self.nav_tofs["angle"])
            rate.sleep()
        

if __name__ == "__main__" :
    
        nav = Navigation()
        nav.run()

        rospy.spin()