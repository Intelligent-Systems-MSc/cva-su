#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32, String

class Navigation() : 

    def __init__(self, MAX_SPEED=1.0, MAX_ANGLE=1.0, MIN_ANGLE=0.2, MAX_DIST=10.0, MIN_DIST=0.4, MIN_SPEED=0.2) : 
        # variables
        self.speed = 0
        self.angle = 0
        self.dist = [10.0, 10.0, 10.0, 10.0]

        # constants
        self.MAX_SPEED = MAX_SPEED # default max speed of the car
        self.MAX_ANGLE = MAX_ANGLE # default max angle of the car
        self.MIN_ANGLE = MIN_ANGLE # minimum angle when turning
        self.MAX_DIST  = MAX_DIST  # default max distance of the tofs
        self.MIN_DIST  = MIN_DIST  # minimum distance = too close
        self.MIN_SPEED = MIN_SPEED # minimum speed = too slow

        print("PARAMETERS : ")
        print("MAX_SPEED = %.2f" % self.MAX_SPEED)
        print("MAX_ANGLE = %.2f" % self.MAX_ANGLE)
        print("MAX_DIST  = %.2f" % self.MAX_DIST)
        print("MIN_DIST  = %.2f" % self.MIN_DIST)
        print("MIN_SPEED = %.2f" % self.MIN_SPEED)

        # Init ROS node
        rospy.init_node('nav_tofs', anonymous=True)

        # Init ROS PUBLISHERS
        self.pub_nav = rospy.Publisher("/TofsSpeedAngleCommand", Float32MultiArray, queue_size = 1)
        self.pub_message = rospy.Publisher("/TofsNavMessage", String, queue_size = 1)

        # Init ROS SUBSCRIBERS
        self.sub_dist = rospy.Subscriber("/TofsDistance", Float32MultiArray, self.callback_dist)

    def callback_dist(self, msg) :
        """ Callback function for the distance topic """
        self.dist = msg.data

    def set_speed_angle(self, speed, angle) :
        """ Set the speed and angle of the car """
        self.speed = speed
        self.angle = angle
        self.pub_nav.publish(Float32MultiArray(data=[self.speed, self.angle]))


    def run(self) :
        """ Main loop of the navigation running with front and back tofs"""
        from math import pi, atan2

        # lateral distance to the obstacle
        lateral_dist_to_obstacle = 1.5 # the higher the value, the more the car will turn
        recul_dist = 1.0
        backward_speed = 0.5

        # going backwards state
        going_backwards = False

        # coefficients : "how much the new value is important"
        K_angle = 0.90 # proportional coefficient for the angle
        K_speed_des = 0.80 # proportional coefficient for the speed desceleartion
        K_speed_acc = 0.60 # proportional coefficient for the speed acceleration

        # frequency of the loop
        HZ = 25 # Hz
        rate = rospy.Rate(HZ)

        # main loop
        while not rospy.is_shutdown() :

            sensor = {
                "fl" : self.dist[0], # front left
                "fr" : self.dist[1], # front right
                "bl" : self.dist[2], # back left
                "br" : self.dist[3], # back right
                "l" : 10.0, # left
                "r" : 10.0  # right
            }

            # calculate the current shortest distance and normalize it between 0 and 1
            dist = min(sensor["fl"], sensor["fr"])
            norma_dist = dist / self.MAX_DIST

            # if the car is going backwards
            if going_backwards :
                mess = "Going backward"

                # if the obstacle is at recul_distance or more => go forward
                if min(sensor["fl"], sensor["fr"]) > recul_dist :
                    going_backwards = False
                    new_speed = 0.0

                # if the obstacle is too close and no obstacle behind => go backward
                elif sensor["bl"] > self.MIN_DIST and sensor["br"] > self.MIN_DIST :
                    new_speed = -backward_speed
                    # keeping the same angle

                # if the obstacle is too close and an obstacle is behind => stop
                else :
                    new_speed = 0.0
                    new_angle = 0.0
                    mess = "Too close on both sides, stoping"
            
            # elif a front obstacle is detected
            elif norma_dist < 0.999 :
                mess = "Obstacle detected, navigating around it"

                # calculate the new speed and angle
                new_speed = (self.MAX_SPEED - self.MIN_SPEED) * norma_dist + self.MIN_SPEED
                new_angle = (self.MAX_ANGLE - self.MIN_ANGLE) * (atan2(lateral_dist_to_obstacle, dist)) + self.MIN_ANGLE

                # angle limitation
                if new_angle > self.MAX_ANGLE :
                    new_angle = self.MAX_ANGLE
                
                # if the right distance is smaller than the left distance => turn right
                if sensor["fr"] < sensor["fl"] :
                    new_angle = -new_angle

                if min(sensor["fl"], sensor["fr"]) < self.MIN_DIST :
                    # if the obstacle is under MIN_DIST => go backward
                    going_backwards = True
                    if new_angle < 0 : new_angle =  self.MAX_ANGLE
                    else :             new_angle = -self.MAX_ANGLE
                    new_speed = 0.0
            
            # no obstacle detected => go forward
            else :
                mess = "No information, navigating strait"
                new_speed = self.MAX_SPEED
                new_angle = 0.0
            
            # determine the speed coefficient
            if new_speed > self.speed :
                K_speed = K_speed_acc
            else :
                K_speed = K_speed_des

            # application of the coefficients
            new_speed = K_speed * new_speed + (1-K_speed) * self.speed
            new_angle = K_angle * new_angle + (1-K_angle) * self.angle
            
            # set the new speed and angle and publish the message
            self.set_speed_angle(new_speed, new_angle)
            self.pub_message.publish(mess)

            rate.sleep()
        

if __name__ == "__main__" :

        # get the parameters
        max_sp = rospy.get_param("max_speed", default=1.0)
        min_sp = rospy.get_param("min_speed", default=0.2)
        max_ds = rospy.get_param("tofs_default_max_dist",  default=10.0)
        min_ds = rospy.get_param("min_dist",  default=0.4)
        max_ag = rospy.get_param("max_angle", default=1.0)
        min_ag = rospy.get_param("min_angle", default=0.2)

        
        nav = Navigation(MAX_SPEED=max_sp, MAX_ANGLE=max_ag, MIN_ANGLE=min_ag, MAX_DIST=max_ds, MIN_DIST=min_ds, MIN_SPEED=min_sp)
        nav.run()

        rospy.spin()
