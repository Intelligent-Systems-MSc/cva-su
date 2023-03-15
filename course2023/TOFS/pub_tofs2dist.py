#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA
"""

import rospy
from std_msgs.msg import Float32MultiArray

class Distance() : 

    def __init__(self, MAX_DIST=10.0, nb_tofs=4, queue_size=5) : 
        
        self.nb_tofs = nb_tofs
        self.MAX_DIST = MAX_DIST # default max distance of the tofs
        self.dist = [MAX_DIST] * self.nb_tofs
        self.queue = [self.dist] * queue_size

        # Init ROS node
        rospy.init_node('pub_tofs2dist', anonymous=True)

        # Init ROS publishers
        self.pub_dist = rospy.Publisher("/TofsDistance", Float32MultiArray, queue_size = 1)

        # Init ROS subscribers
        self.sub_tofs = rospy.Subscriber("/SensorsScan", Float32MultiArray, self.callback_tofs)

    def callback_tofs(self, msg) :
        # [front_left, front_right, back, left]
        # ??? why not [front_left, front_right, back_left, back_right, left, right] ???
        self.dist = [d if d > 0.001 else self.MAX_DIST for d in msg.data]
        self.movingAverage_filter()
        self.pub_dist.publish(Float32MultiArray(data=self.dist))

    def movingAverage_filter(self) :
        """ Filter the distance with a moving average"""
        self.queue.pop(0)
        self.queue.append(self.dist)
        self.dist = [sum([d[i] for d in self.queue]) / len(self.queue) for i in range(self.nb_tofs)]
    


if __name__ == "__main__" :

    max_ds = rospy.get_param("tofs_default_max_dist",  default=10.0)

    Distance(MAX_DIST=max_ds)
    rospy.spin()