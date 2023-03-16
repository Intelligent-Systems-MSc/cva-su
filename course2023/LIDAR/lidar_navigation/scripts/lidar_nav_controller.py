#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from std_msgs.msg import Float32
import message_filters 

class Controller:
    def __init__(self):
        self.speed=Float32()
        self.ang=Float32()

def angle_regulator_callback(msg_dir,msg_center,c):
    d_center=msg_center.data #centering err : left-right -> d_c>0 : go left, d_c<0 : go right
    d_dir=msg_dir.data #orientation err : desired_orientation-current_orientation -> d_dir>0 : go right, d_dir<0 : go left 

    #controler gains
    k_c=rospy.get_param("kc",default=-0.5)
    k_d=rospy.get_param("kd",default=3.0)

    #le gain pour dir doit etre plus grand que celui du centrage pour garantir evitement d'obstacle avant de se centrer
    u= k_d*d_dir +k_c*d_center 

    #saturator
    c.ang=np.tanh(u)

def speed_regulator_callback(msg_front_dist,c):
    front_dist=msg_front_dist.data
    
    k_s=rospy.get_param("ks",default=1)

    u=k_s*front_dist

    c.speed=np.tanh(u)


if __name__=='__main__':
    try:
        
        #init node
        rospy.init_node("lidar_nav_control")


        #init controller
        c = Controller()
        #c.speed=0.65

        #subscribe to lidar_dir and lidar_center topics
        dir_topic=rospy.get_param("~dir_topic",default="/lidar_dir")
        center_topic=rospy.get_param("~center_topic",default="/lidar_center")

        dir_sub=message_filters.Subscriber(dir_topic,Float32, queue_size=1)
        center_sub=message_filters.Subscriber(center_topic,Float32, queue_size=1)

        ts=message_filters.ApproximateTimeSynchronizer([dir_sub, center_sub], queue_size=1,slop=0.1, allow_headerless=True)

        #callback
        ts.registerCallback(angle_regulator_callback,c)

        #subscribe to front_dist topic
        front_dist_topic="/front_dist"
        front_dist_sub=rospy.Subscriber(front_dist_topic,Float32,speed_regulator_callback,c)

        #define rate
        rate=rospy.Rate(10)

        #publish on angular and speed command topic
        angle_pub=rospy.Publisher("/AngleCommand",Float32,queue_size=1)
        speed_pub=rospy.Publisher("/SpeedCommand",Float32,queue_size=1)


        while not rospy.is_shutdown():

            angle_pub.publish(c.ang)
            speed_pub.publish(c.speed)

            rate.sleep()


        
    

    except rospy.ROSInterruptException:
        pass
